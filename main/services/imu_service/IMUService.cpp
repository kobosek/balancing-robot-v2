#include "IMUService.hpp"

#include "CONFIG_FullConfigUpdate.hpp"
#include "CONFIG_ImuConfigUpdate.hpp"
#include "EventBus.hpp"
#include "FIFOProcessor.hpp"
#include "FIFOTask.hpp"
#include "HealthMonitorTask.hpp"
#include "I2CDevice.hpp"
#include "IMUCalibration.hpp"
#include "IMUHealthMonitor.hpp"
#include "IMU_AttachRequested.hpp"
#include "IMU_AvailabilityChanged.hpp"
#include "IMU_CalibrationCompleted.hpp"
#include "IMU_CalibrationRequest.hpp"
#include "IMU_CalibrationRequestRejected.hpp"
#include "IMU_CommunicationError.hpp"
#include "IMU_GyroOffsetsUpdated.hpp"
#include "MPU6050HardwareController.hpp"
#include "OrientationEstimator.hpp"
#include "SYSTEM_StateChanged.hpp"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/task.h"
#include "mpu6050.hpp"

namespace {
constexpr int64_t AUTO_ATTACH_RETRY_INTERVAL_US = 1000LL * 1000LL;
}

IMUService::IMUService(std::shared_ptr<OrientationEstimator> estimator,
                       const MPU6050Config& config,
                       const SystemBehaviorConfig& behaviorConfig,
                       EventBus& bus) :
    m_i2cDevice(),
    m_driver(),
    m_hardwareController(),
    m_estimator(estimator),
    m_config(config),
    m_behaviorConfig(behaviorConfig),
    m_eventBus(bus),
    m_healthMonitor(),
    m_fifoProcessor(),
    m_calibration(),
    m_fifoTask(),
    m_healthMonitorTask(),
    m_is_calibrating_flag(false),
    m_current_state(IMUState::INITIALIZED),
    m_system_state(SystemState::INIT),
    m_pending_hardware_apply(false),
    m_fault_reported(false),
    m_next_auto_attach_time_us(0),
    m_accel_lsb_per_g(8192.0f),
    m_gyro_lsb_per_dps(65.5f),
    m_sample_period_s(0.001f),
    m_profile() {}

IMUService::~IMUService() {
    stopTasks();
}

esp_err_t IMUService::init() {
    m_i2cDevice = std::make_unique<I2CDevice>();
    m_driver = std::make_unique<MPU6050Driver>(*m_i2cDevice);
    m_hardwareController = std::make_unique<MPU6050HardwareController>(*m_i2cDevice, *m_driver);
    m_healthMonitor = std::make_unique<IMUHealthMonitor>(*this, m_behaviorConfig);
    m_fifoProcessor = std::make_unique<FIFOProcessor>(*m_driver, m_estimator, *m_healthMonitor, *this);
    m_fifoTask = std::make_unique<FIFOTask>(*m_fifoProcessor);
    m_healthMonitorTask = std::make_unique<HealthMonitorTask>(*m_healthMonitor, *this);
    m_calibration = std::make_unique<IMUCalibration>(*m_driver);

    safeConfigUpdate([this]() { refreshDerivedStateLocked(); });
    m_estimator->init(
        m_config.comp_filter_alpha,
        m_sample_period_s,
        m_config.gyro_offset_x,
        m_config.gyro_offset_y,
        m_config.gyro_offset_z);
    m_calibration->setOffsets(m_config.gyro_offset_x, m_config.gyro_offset_y, m_config.gyro_offset_z);
    m_fifoProcessor->setScalingFactors(m_accel_lsb_per_g, m_gyro_lsb_per_dps);
    m_fifoProcessor->configureReadout(m_sample_period_s, m_config.fifo_read_threshold);
    m_healthMonitor->notifyIMUStateChange(IMUState::INITIALIZED);

    esp_err_t ret = attachAndConfigureCurrentProfile(false);
    if (ret != ESP_OK) {
        transitionToState(IMUState::UNAVAILABLE);
    }

    if (!m_fifoTask || !m_healthMonitorTask) {
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

bool IMUService::startTasks() {
    if (!m_fifoTask->start(configMAX_PRIORITIES - 2, 0, 6144)) {
        return false;
    }
    if (!m_healthMonitorTask->start(5, 1, 4096)) {
        m_fifoTask->stop();
        return false;
    }
    return true;
}

void IMUService::stopTasks() {
    if (m_fifoTask) {
        m_fifoTask->stop();
    }
    if (m_healthMonitorTask) {
        m_healthMonitorTask->stop();
    }
}

void IMUService::handleEvent(const BaseEvent& event) {
    switch (event.type) {
        case EventType::CONFIG_FULL_UPDATE:
            handleConfigUpdate(static_cast<const CONFIG_FullConfigUpdate&>(event));
            break;
        case EventType::CONFIG_IMU_UPDATE:
            handleIMUConfigUpdate(static_cast<const CONFIG_ImuConfigUpdate&>(event));
            break;
        case EventType::IMU_CALIBRATION_REQUEST:
            handleCalibrationRequest(static_cast<const IMU_CalibrationRequest&>(event));
            break;
        case EventType::IMU_ATTACH_REQUESTED:
            handleAttachRequested(static_cast<const IMU_AttachRequested&>(event));
            break;
        case EventType::SYSTEM_STATE_CHANGED:
            handleSystemStateChanged(static_cast<const SYSTEM_StateChanged&>(event));
            break;
        default:
            break;
    }
}

void IMUService::onIMUHardFault(esp_err_t errorCode) {
    if (m_current_state.load(std::memory_order_acquire) == IMUState::UNAVAILABLE) {
        return;
    }

    if (errorCode != ESP_ERR_TIMEOUT && !m_healthMonitor->recordCommunicationFailure(errorCode)) {
        return;
    }

    markSensorUnavailable(errorCode, true);
}

void IMUService::pollBackgroundMaintenance() {
    applyPendingHardwareConfigIfSafe();

    if (getCurrentState() != IMUState::UNAVAILABLE ||
        getSystemState() != SystemState::IDLE ||
        m_is_calibrating_flag.load(std::memory_order_acquire)) {
        return;
    }

    const int64_t now = esp_timer_get_time();
    const int64_t nextAttempt = m_next_auto_attach_time_us.load(std::memory_order_acquire);
    if (nextAttempt != 0 && now < nextAttempt) {
        return;
    }

    std::unique_lock<std::mutex> attachLock(m_attach_mutex, std::try_to_lock);
    if (!attachLock.owns_lock()) {
        return;
    }

    if (getCurrentState() != IMUState::UNAVAILABLE || getSystemState() != SystemState::IDLE) {
        return;
    }

    const esp_err_t ret = attachAndConfigureCurrentProfileLocked(true);
    if (ret != ESP_OK) {
        scheduleAutoAttachRetry(AUTO_ATTACH_RETRY_INTERVAL_US);
    }
}

IMUState IMUService::getCurrentState() const {
    std::lock_guard<std::mutex> lock(m_state_mutex);
    return m_current_state.load(std::memory_order_relaxed);
}

SystemState IMUService::getSystemState() const {
    return m_system_state.load(std::memory_order_acquire);
}

void IMUService::updateSystemState(SystemState newState) {
    m_system_state.store(newState, std::memory_order_release);
}

bool IMUService::isAvailable() const {
    return m_current_state.load(std::memory_order_acquire) != IMUState::UNAVAILABLE;
}

const char* IMUService::stateToString(IMUState state) {
    switch (state) {
        case IMUState::INITIALIZED:
            return "INITIALIZED";
        case IMUState::OPERATIONAL:
            return "OPERATIONAL";
        case IMUState::CALIBRATION:
            return "CALIBRATION";
        case IMUState::UNAVAILABLE:
            return "UNAVAILABLE";
        default:
            return "UNKNOWN_IMU_STATE";
    }
}

void IMUService::transitionToState(IMUState newState) {
    std::lock_guard<std::mutex> stateLock(m_state_mutex);
    IMUState oldState = m_current_state.load(std::memory_order_relaxed);
    if (oldState == newState || !isValidTransition(oldState, newState)) {
        return;
    }

    switch (oldState) {
        case IMUState::INITIALIZED:
            (void)exitInitializedState();
            break;
        case IMUState::OPERATIONAL:
            (void)exitOperationalState();
            break;
        case IMUState::CALIBRATION:
            (void)exitCalibrationState();
            break;
        case IMUState::UNAVAILABLE:
            (void)exitUnavailableState();
            break;
    }

    esp_err_t enterRet = ESP_OK;
    switch (newState) {
        case IMUState::INITIALIZED:
            enterRet = enterInitializedState();
            break;
        case IMUState::OPERATIONAL:
            enterRet = enterOperationalState();
            break;
        case IMUState::CALIBRATION:
            enterRet = enterCalibrationState();
            break;
        case IMUState::UNAVAILABLE:
            enterRet = enterUnavailableState();
            break;
    }

    if (enterRet != ESP_OK && newState != IMUState::UNAVAILABLE) {
        newState = IMUState::UNAVAILABLE;
        (void)enterUnavailableState();
    }

    m_current_state.store(newState, std::memory_order_release);
    if (m_healthMonitor) {
        m_healthMonitor->notifyIMUStateChange(newState);
    }
}

esp_err_t IMUService::enterInitializedState() {
    if (!m_i2cDevice || !m_i2cDevice->isOpen()) {
        return ESP_OK;
    }
    return m_fifoProcessor->disableFIFO();
}

esp_err_t IMUService::exitInitializedState() {
    return ESP_OK;
}

esp_err_t IMUService::enterOperationalState() {
    if (!m_i2cDevice || !m_i2cDevice->isOpen()) {
        return ESP_ERR_INVALID_STATE;
    }

    if (m_config.int_pin >= 0 && m_config.int_pin < GPIO_NUM_MAX) {
        return m_fifoProcessor->enableFIFO(m_config.int_pin, m_config.interrupt_active_high);
    }

    return m_fifoProcessor->enableFIFO();
}

esp_err_t IMUService::exitOperationalState() {
    if (!m_i2cDevice || !m_i2cDevice->isOpen()) {
        return ESP_OK;
    }
    return m_fifoProcessor->disableFIFO();
}

esp_err_t IMUService::enterCalibrationState() {
    if (!m_i2cDevice || !m_i2cDevice->isOpen()) {
        return ESP_ERR_INVALID_STATE;
    }
    return m_fifoProcessor->disableFIFO();
}

esp_err_t IMUService::exitCalibrationState() {
    m_is_calibrating_flag.store(false, std::memory_order_release);
    return ESP_OK;
}

esp_err_t IMUService::enterUnavailableState() {
    if (m_i2cDevice && m_i2cDevice->isOpen()) {
        (void)m_fifoProcessor->disableFIFO();
    }
    if (m_hardwareController) {
        m_hardwareController->disconnect();
    }
    scheduleAutoAttachRetry(AUTO_ATTACH_RETRY_INTERVAL_US);
    return ESP_OK;
}

esp_err_t IMUService::exitUnavailableState() {
    m_next_auto_attach_time_us.store(0, std::memory_order_release);
    return ESP_OK;
}

bool IMUService::isValidTransition(IMUState from, IMUState to) {
    switch (from) {
        case IMUState::INITIALIZED:
            return to == IMUState::OPERATIONAL || to == IMUState::UNAVAILABLE;
        case IMUState::OPERATIONAL:
            return to == IMUState::INITIALIZED || to == IMUState::CALIBRATION || to == IMUState::UNAVAILABLE;
        case IMUState::CALIBRATION:
            return to == IMUState::OPERATIONAL || to == IMUState::UNAVAILABLE;
        case IMUState::UNAVAILABLE:
            return to == IMUState::OPERATIONAL || to == IMUState::INITIALIZED;
        default:
            return false;
    }
}

void IMUService::refreshDerivedStateLocked() {
    m_profile = MPU6050Profile::fromConfig(m_config);
    m_accel_lsb_per_g = m_profile.accelLsbPerG;
    m_gyro_lsb_per_dps = m_profile.gyroLsbPerDps;
    m_sample_period_s = m_profile.samplePeriodS;
}

void IMUService::applyConfig(const MPU6050Config& newConfig) {
    bool hardwareConfigChanged = false;
    bool estimatorNeedsInit = false;
    bool gyroOffsetsChanged = false;

    safeConfigUpdate([&]() {
        const MPU6050Config oldConfig = m_config;
        hardwareConfigChanged = oldConfig.requiresHardwareInit(newConfig);
        estimatorNeedsInit =
            oldConfig.comp_filter_alpha != newConfig.comp_filter_alpha ||
            oldConfig.sample_rate_divisor != newConfig.sample_rate_divisor ||
            oldConfig.dlpf_config != newConfig.dlpf_config;
        gyroOffsetsChanged =
            oldConfig.gyro_offset_x != newConfig.gyro_offset_x ||
            oldConfig.gyro_offset_y != newConfig.gyro_offset_y ||
            oldConfig.gyro_offset_z != newConfig.gyro_offset_z;
        m_config = newConfig;
        refreshDerivedStateLocked();
    });

    m_fifoProcessor->setScalingFactors(m_accel_lsb_per_g, m_gyro_lsb_per_dps);
    m_fifoProcessor->configureReadout(m_sample_period_s, m_config.fifo_read_threshold);
    if (estimatorNeedsInit || hardwareConfigChanged) {
        m_estimator->init(
            m_config.comp_filter_alpha,
            m_sample_period_s,
            m_config.gyro_offset_x,
            m_config.gyro_offset_y,
            m_config.gyro_offset_z);
    } else if (gyroOffsetsChanged) {
        m_estimator->updateGyroOffsets(m_config.gyro_offset_x, m_config.gyro_offset_y, m_config.gyro_offset_z);
    }
    m_calibration->setOffsets(m_config.gyro_offset_x, m_config.gyro_offset_y, m_config.gyro_offset_z);

    if (hardwareConfigChanged) {
        m_pending_hardware_apply.store(true, std::memory_order_release);
        applyPendingHardwareConfigIfSafe();
    }
}

void IMUService::applyConfig(const SystemBehaviorConfig& config) {
    safeConfigUpdate([&]() { m_behaviorConfig = config; });
    m_healthMonitor->applyConfig(config);
}

bool IMUService::canApplyHardwareConfigNow() const {
    const SystemState systemState = m_system_state.load(std::memory_order_acquire);
    const IMUState imuState = m_current_state.load(std::memory_order_acquire);
    return systemState != SystemState::BALANCING &&
           !m_is_calibrating_flag.load(std::memory_order_acquire) &&
           imuState != IMUState::CALIBRATION;
}

void IMUService::applyPendingHardwareConfigIfSafe() {
    if (!m_pending_hardware_apply.load(std::memory_order_acquire) || !canApplyHardwareConfigNow()) {
        return;
    }

    m_pending_hardware_apply.store(false, std::memory_order_release);
    const IMUState previousState = getCurrentState();
    if (previousState == IMUState::OPERATIONAL) {
        transitionToState(IMUState::INITIALIZED);
    }

    const esp_err_t ret = attachAndConfigureCurrentProfile(previousState == IMUState::UNAVAILABLE);
    if (ret != ESP_OK) {
        if (previousState != IMUState::UNAVAILABLE) {
            markSensorUnavailable(ret, true);
        } else {
            scheduleAutoAttachRetry(AUTO_ATTACH_RETRY_INTERVAL_US);
        }
    }
}

esp_err_t IMUService::attachAndConfigureCurrentProfile(bool publishAvailabilityEvent) {
    std::lock_guard<std::mutex> attachLock(m_attach_mutex);
    return attachAndConfigureCurrentProfileLocked(publishAvailabilityEvent);
}

esp_err_t IMUService::attachAndConfigureCurrentProfileLocked(bool publishAvailabilityEvent) {
    MPU6050Config configCopy;
    MPU6050Profile profileCopy;
    {
        std::lock_guard<std::mutex> configLock(m_config_mutex);
        configCopy = m_config;
        profileCopy = m_profile;
    }

    const IMUState previousState = m_current_state.load(std::memory_order_acquire);
    esp_err_t ret = m_hardwareController->connectAndConfigure(configCopy, profileCopy);
    if (ret != ESP_OK) {
        return ret;
    }

    safeConfigUpdate([&]() {
        m_profile = profileCopy;
        m_accel_lsb_per_g = profileCopy.accelLsbPerG;
        m_gyro_lsb_per_dps = profileCopy.gyroLsbPerDps;
        m_sample_period_s = profileCopy.samplePeriodS;
    });

    m_fifoProcessor->setScalingFactors(m_accel_lsb_per_g, m_gyro_lsb_per_dps);
    m_fifoProcessor->configureReadout(m_sample_period_s, m_config.fifo_read_threshold);
    m_estimator->init(
        m_config.comp_filter_alpha,
        m_sample_period_s,
        m_config.gyro_offset_x,
        m_config.gyro_offset_y,
        m_config.gyro_offset_z);
    m_calibration->setOffsets(m_config.gyro_offset_x, m_config.gyro_offset_y, m_config.gyro_offset_z);
    m_healthMonitor->resetFaultCounters();
    m_fault_reported.store(false, std::memory_order_release);

    transitionToState(IMUState::OPERATIONAL);
    if (getCurrentState() != IMUState::OPERATIONAL) {
        return ESP_FAIL;
    }

    if (publishAvailabilityEvent && previousState == IMUState::UNAVAILABLE) {
        m_eventBus.publish(IMU_AvailabilityChanged(true));
    }

    return ESP_OK;
}

void IMUService::markSensorUnavailable(esp_err_t errorCode, bool publishErrorEvent) {
    const IMUState previousState = m_current_state.load(std::memory_order_acquire);
    const bool firstFault = !m_fault_reported.exchange(true, std::memory_order_acq_rel);

    if (previousState != IMUState::UNAVAILABLE) {
        transitionToState(IMUState::UNAVAILABLE);
        m_eventBus.publish(IMU_AvailabilityChanged(false));
    }

    if (publishErrorEvent && firstFault) {
        m_eventBus.publish(IMU_CommunicationError(errorCode));
    }
}

void IMUService::handleConfigUpdate(const CONFIG_FullConfigUpdate& event) {
    applyConfig(event.configData.imu);
    applyConfig(event.configData.behavior);
}

void IMUService::handleIMUConfigUpdate(const CONFIG_ImuConfigUpdate& event) {
    applyConfig(event.config);
    if (event.requiresHardwareInit) {
        m_pending_hardware_apply.store(true, std::memory_order_release);
        applyPendingHardwareConfigIfSafe();
    }
}

void IMUService::handleCalibrationRequest(const IMU_CalibrationRequest& event) {
    (void)event;
    if (m_is_calibrating_flag.load(std::memory_order_acquire)) {
        m_eventBus.publish(IMU_CalibrationRequestRejected(IMU_CalibrationRequestRejected::Reason::OTHER, false));
        return;
    }

    if (getCurrentState() == IMUState::UNAVAILABLE) {
        const esp_err_t attachRet = attachAndConfigureCurrentProfile(true);
        if (attachRet != ESP_OK) {
            scheduleAutoAttachRetry(AUTO_ATTACH_RETRY_INTERVAL_US);
            m_eventBus.publish(IMU_CalibrationRequestRejected(IMU_CalibrationRequestRejected::Reason::OTHER, false));
            return;
        }
    }

    transitionToState(IMUState::CALIBRATION);
    const esp_err_t result = performCalibration();
    transitionToState(isAvailable() ? IMUState::OPERATIONAL : IMUState::UNAVAILABLE);
    m_eventBus.publish(IMU_CalibrationCompleted(result));
}

void IMUService::handleAttachRequested(const IMU_AttachRequested& event) {
    (void)event;
    if (getCurrentState() != IMUState::UNAVAILABLE) {
        return;
    }

    const esp_err_t ret = attachAndConfigureCurrentProfile(true);
    if (ret != ESP_OK) {
        scheduleAutoAttachRetry(AUTO_ATTACH_RETRY_INTERVAL_US);
    }
}

void IMUService::handleSystemStateChanged(const SYSTEM_StateChanged& event) {
    m_system_state.store(event.newState, std::memory_order_release);
    if (event.newState == SystemState::IDLE) {
        if (getCurrentState() == IMUState::UNAVAILABLE) {
            scheduleAutoAttachRetry(0);
        }
        applyPendingHardwareConfigIfSafe();
    }
}

esp_err_t IMUService::performCalibration() {
    if (getCurrentState() != IMUState::CALIBRATION) {
        return ESP_ERR_INVALID_STATE;
    }

    m_is_calibrating_flag.store(true, std::memory_order_release);

    MPU6050Profile profileCopy;
    int calibrationSamples = 0;
    {
        std::lock_guard<std::mutex> configLock(m_config_mutex);
        profileCopy = m_profile;
        calibrationSamples = m_config.calibration_samples;
    }

    auto progressCallback = [](int progress, int total) {
        (void)progress;
        (void)total;
        vTaskDelay(pdMS_TO_TICKS(5));
    };

    esp_err_t result = m_calibration->calibrate(profileCopy, calibrationSamples, progressCallback);
    if (result == ESP_OK) {
        safeConfigUpdate([&]() {
            m_config.gyro_offset_x = m_calibration->getGyroOffsetXDPS();
            m_config.gyro_offset_y = m_calibration->getGyroOffsetYDPS();
            m_config.gyro_offset_z = m_calibration->getGyroOffsetZDPS();
        });

        m_estimator->updateGyroOffsets(
            m_calibration->getGyroOffsetXDPS(),
            m_calibration->getGyroOffsetYDPS(),
            m_calibration->getGyroOffsetZDPS());
        m_calibration->setOffsets(
            m_calibration->getGyroOffsetXDPS(),
            m_calibration->getGyroOffsetYDPS(),
            m_calibration->getGyroOffsetZDPS());
        m_eventBus.publish(IMU_GyroOffsetsUpdated(
            m_calibration->getGyroOffsetXDPS(),
            m_calibration->getGyroOffsetYDPS(),
            m_calibration->getGyroOffsetZDPS()));
    } else if (result != ESP_FAIL) {
        markSensorUnavailable(result, true);
    }

    m_is_calibrating_flag.store(false, std::memory_order_release);
    return result;
}

void IMUService::scheduleAutoAttachRetry(int64_t delayUs) {
    const int64_t clampedDelayUs = (delayUs <= 0) ? 0 : delayUs;
    m_next_auto_attach_time_us.store(esp_timer_get_time() + clampedDelayUs, std::memory_order_release);
}

void IMUService::safeConfigUpdate(const std::function<void()>& updateFunc) {
    std::lock_guard<std::mutex> configLock(m_config_mutex);
    updateFunc();
}
