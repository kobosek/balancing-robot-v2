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
#include "IMU_SystemPolicyChanged.hpp"
#include "MPU6050HardwareController.hpp"
#include "OrientationEstimator.hpp"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/task.h"
#include "mpu6050.hpp"

namespace {
constexpr int64_t AUTO_ATTACH_RETRY_INTERVAL_US = 1000LL * 1000LL;

class ScopedAtomicFlagReset {
public:
    explicit ScopedAtomicFlagReset(std::atomic<bool>& flag) : m_flag(flag) {}
    ~ScopedAtomicFlagReset() {
        m_flag.store(false, std::memory_order_release);
    }

    ScopedAtomicFlagReset(const ScopedAtomicFlagReset&) = delete;
    ScopedAtomicFlagReset& operator=(const ScopedAtomicFlagReset&) = delete;

private:
    std::atomic<bool>& m_flag;
};
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
    m_calibration_allowed(false),
    m_auto_attach_allowed(false),
    m_hardware_config_apply_allowed(true),
    m_pending_hardware_apply(false),
    m_fault_reported(false),
    m_next_auto_attach_time_us(0),
    m_initialized(false),
    m_profile() {}

IMUService::~IMUService() {
    stopTasks();
}

esp_err_t IMUService::init() {
    if (m_initialized.load(std::memory_order_acquire)) {
        ESP_LOGI(TAG, "IMU service already initialized");
        return ESP_OK;
    }
    if (!m_estimator) {
        ESP_LOGE(TAG, "Cannot initialize IMU service without an orientation estimator");
        return ESP_ERR_INVALID_ARG;
    }

    auto i2cDevice = std::make_unique<I2CDevice>();
    if (!i2cDevice) {
        return ESP_ERR_NO_MEM;
    }

    auto driver = std::make_unique<MPU6050Driver>(*i2cDevice);
    if (!driver) {
        return ESP_ERR_NO_MEM;
    }

    auto hardwareController = std::make_unique<MPU6050HardwareController>(*i2cDevice, *driver);
    auto healthMonitor = std::make_unique<IMUHealthMonitor>(*this, m_behaviorConfig);
    auto fifoProcessor = std::make_unique<FIFOProcessor>(*driver, *m_estimator, *healthMonitor, *this);
    auto fifoTask = std::make_unique<FIFOTask>(*fifoProcessor);
    auto healthMonitorTask = std::make_unique<HealthMonitorTask>(*healthMonitor, *this);
    auto calibration = std::make_unique<IMUCalibration>(*driver);
    if (!hardwareController || !healthMonitor || !fifoProcessor || !fifoTask || !healthMonitorTask || !calibration) {
        return ESP_ERR_NO_MEM;
    }

    m_i2cDevice = std::move(i2cDevice);
    m_driver = std::move(driver);
    m_hardwareController = std::move(hardwareController);
    m_healthMonitor = std::move(healthMonitor);
    m_fifoProcessor = std::move(fifoProcessor);
    m_fifoTask = std::move(fifoTask);
    m_healthMonitorTask = std::move(healthMonitorTask);
    m_calibration = std::move(calibration);

    MPU6050Config configCopy;
    MPU6050Profile profileCopy;
    safeConfigUpdate([&]() {
        refreshDerivedStateLocked();
        configCopy = m_config;
        profileCopy = m_profile;
    });
    applyRuntimeProfile(configCopy, profileCopy, true);
    m_healthMonitor->notifyIMUStateChange(IMUState::INITIALIZED);

    const esp_err_t ret = attachAndConfigureCurrentProfile(false);
    if (ret != ESP_OK) {
        transitionToState(IMUState::UNAVAILABLE);
    }

    m_initialized.store(true, std::memory_order_release);
    return ESP_OK;
}

bool IMUService::startTasks() {
    if (!m_initialized.load(std::memory_order_acquire)) {
        ESP_LOGE(TAG, "Cannot start IMU tasks before init");
        return false;
    }
    if (!m_fifoTask || !m_healthMonitorTask) {
        ESP_LOGE(TAG, "Cannot start IMU tasks because task objects are missing");
        return false;
    }

    const bool fifoRunning = m_fifoTask->isRunning();
    const bool healthRunning = m_healthMonitorTask->isRunning();
    if (fifoRunning && healthRunning) {
        return true;
    }
    if (fifoRunning != healthRunning) {
        ESP_LOGW(TAG, "IMU task set was partially running, restarting both tasks");
        stopTasks();
    }

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
    if (event.is<CONFIG_FullConfigUpdate>()) {
        handleConfigUpdate(event.as<CONFIG_FullConfigUpdate>());
    } else if (event.is<CONFIG_ImuConfigUpdate>()) {
        handleIMUConfigUpdate(event.as<CONFIG_ImuConfigUpdate>());
    } else if (event.is<IMU_CalibrationRequest>()) {
        handleCalibrationRequest(event.as<IMU_CalibrationRequest>());
    } else if (event.is<IMU_AttachRequested>()) {
        handleAttachRequested(event.as<IMU_AttachRequested>());
    } else if (event.is<IMU_SystemPolicyChanged>()) {
        handleSystemPolicyChanged(event.as<IMU_SystemPolicyChanged>());
    }
}

void IMUService::onIMUHardFault(esp_err_t errorCode) {
    if (!m_healthMonitor) {
        ESP_LOGW(TAG, "Ignoring IMU hard fault before health monitor initialization: %s", esp_err_to_name(errorCode));
        return;
    }

    const IMUState currentState = getCurrentState();
    if (currentState == IMUState::UNAVAILABLE) {
        return;
    }

    if (errorCode != ESP_ERR_TIMEOUT && !m_healthMonitor->recordCommunicationFailure(errorCode)) {
        return;
    }

    ESP_LOGE(TAG,
             "Promoting IMU fault to unavailable (state=%s code=%s)",
             stateToString(currentState),
             esp_err_to_name(errorCode));
    markSensorUnavailable(errorCode, true);
}

void IMUService::pollBackgroundMaintenance() {
    if (!m_initialized.load(std::memory_order_acquire) || !m_hardwareController || !m_fifoProcessor) {
        return;
    }

    applyPendingHardwareConfigIfSafe();

    if (getCurrentState() != IMUState::UNAVAILABLE ||
        !m_auto_attach_allowed.load(std::memory_order_acquire) ||
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

    if (getCurrentState() != IMUState::UNAVAILABLE ||
        !m_auto_attach_allowed.load(std::memory_order_acquire)) {
        return;
    }

    bool shouldPublishAvailabilityEvent = false;
    const esp_err_t ret = attachAndConfigureCurrentProfileLocked(true, &shouldPublishAvailabilityEvent);
    attachLock.unlock();

    if (ret != ESP_OK) {
        scheduleAutoAttachRetry(AUTO_ATTACH_RETRY_INTERVAL_US);
        return;
    }

    publishAvailabilityIfOperational(shouldPublishAvailabilityEvent);
}

IMUState IMUService::getCurrentState() const {
    std::lock_guard<std::mutex> lock(m_state_mutex);
    return m_current_state;
}

bool IMUService::isAvailable() const {
    return getCurrentState() != IMUState::UNAVAILABLE;
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

bool IMUService::transitionToState(IMUState newState) {
    std::unique_lock<std::mutex> stateLock(m_state_mutex);
    IMUState oldState = m_current_state;
    if (oldState == newState) {
        return false;
    }
    if (!isValidTransition(oldState, newState)) {
        ESP_LOGW(TAG, "Rejected invalid IMU state transition %s -> %s", stateToString(oldState), stateToString(newState));
        return false;
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
        ESP_LOGW(TAG,
                 "Failed entering IMU state %s from %s: %s; falling back to UNAVAILABLE",
                 stateToString(newState),
                 stateToString(oldState),
                 esp_err_to_name(enterRet));
        newState = IMUState::UNAVAILABLE;
        (void)enterUnavailableState();
    }

    const bool stateChanged = oldState != newState;
    m_current_state = newState;
    stateLock.unlock();

    if (stateChanged) {
        ESP_LOGI(TAG, "IMU state transition %s -> %s", stateToString(oldState), stateToString(newState));
    }
    if (m_healthMonitor) {
        m_healthMonitor->notifyIMUStateChange(newState);
    }
    return stateChanged;
}

esp_err_t IMUService::enterInitializedState() {
    if (!m_fifoProcessor) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!m_i2cDevice || !m_i2cDevice->isOpen()) {
        return ESP_OK;
    }
    return m_fifoProcessor->disableFIFO();
}

esp_err_t IMUService::exitInitializedState() {
    return ESP_OK;
}

esp_err_t IMUService::enterOperationalState() {
    if (!m_fifoProcessor || !m_i2cDevice || !m_i2cDevice->isOpen()) {
        return ESP_ERR_INVALID_STATE;
    }

    MPU6050Config configCopy;
    {
        std::lock_guard<std::mutex> configLock(m_config_mutex);
        configCopy = m_config;
    }

    if (configCopy.int_pin >= 0 && configCopy.int_pin < GPIO_NUM_MAX) {
        return m_fifoProcessor->enableFIFO(configCopy.int_pin, configCopy.interrupt_active_high);
    }

    return m_fifoProcessor->enableFIFO();
}

esp_err_t IMUService::exitOperationalState() {
    if (!m_fifoProcessor || !m_i2cDevice || !m_i2cDevice->isOpen()) {
        return ESP_OK;
    }
    return m_fifoProcessor->disableFIFO();
}

esp_err_t IMUService::enterCalibrationState() {
    if (!m_fifoProcessor || !m_i2cDevice || !m_i2cDevice->isOpen()) {
        return ESP_ERR_INVALID_STATE;
    }
    return m_fifoProcessor->disableFIFO();
}

esp_err_t IMUService::exitCalibrationState() {
    return ESP_OK;
}

esp_err_t IMUService::enterUnavailableState() {
    if (m_fifoProcessor && m_i2cDevice && m_i2cDevice->isOpen()) {
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
}

void IMUService::applyRuntimeProfile(const MPU6050Config& config,
                                     const MPU6050Profile& profile,
                                     bool reinitializeEstimator) {
    if (!m_fifoProcessor || !m_calibration || !m_estimator) {
        return;
    }

    m_fifoProcessor->setScalingFactors(profile.accelLsbPerG, profile.gyroLsbPerDps);
    m_fifoProcessor->configureReadout(profile.samplePeriodS, config.fifo_read_threshold);
    if (reinitializeEstimator) {
        m_estimator->init(
            config.comp_filter_alpha,
            profile.samplePeriodS,
            config.gyro_offset_x,
            config.gyro_offset_y,
            config.gyro_offset_z);
    }
    m_calibration->setOffsets(config.gyro_offset_x, config.gyro_offset_y, config.gyro_offset_z);
}

bool IMUService::applyConfig(const MPU6050Config& newConfig) {
    bool hardwareConfigChanged = false;
    bool estimatorNeedsInit = false;
    bool gyroOffsetsChanged = false;
    MPU6050Config configCopy;
    MPU6050Profile profileCopy;

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
        configCopy = m_config;
        profileCopy = m_profile;
    });

    if (!m_fifoProcessor || !m_calibration || !m_estimator) {
        return hardwareConfigChanged;
    }

    applyRuntimeProfile(configCopy, profileCopy, estimatorNeedsInit || hardwareConfigChanged);
    if (!estimatorNeedsInit && !hardwareConfigChanged && gyroOffsetsChanged) {
        m_estimator->updateGyroOffsets(newConfig.gyro_offset_x, newConfig.gyro_offset_y, newConfig.gyro_offset_z);
    }

    if (hardwareConfigChanged) {
        m_pending_hardware_apply.store(true, std::memory_order_release);
        applyPendingHardwareConfigIfSafe();
    }

    return hardwareConfigChanged;
}

void IMUService::applyConfig(const SystemBehaviorConfig& config) {
    safeConfigUpdate([&]() { m_behaviorConfig = config; });
    if (m_healthMonitor) {
        m_healthMonitor->applyConfig(config);
    }
}

bool IMUService::canApplyHardwareConfigNow() const {
    const IMUState imuState = getCurrentState();
    return m_hardware_config_apply_allowed.load(std::memory_order_acquire) &&
           !m_is_calibrating_flag.load(std::memory_order_acquire) &&
           imuState != IMUState::CALIBRATION;
}

void IMUService::applyPendingHardwareConfigIfSafe() {
    if (!m_initialized.load(std::memory_order_acquire) ||
        !m_hardwareController ||
        !m_fifoProcessor ||
        !canApplyHardwareConfigNow()) {
        return;
    }

    bool expected = true;
    if (!m_pending_hardware_apply.compare_exchange_strong(expected, false, std::memory_order_acq_rel)) {
        return;
    }

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

void IMUService::publishAvailabilityIfOperational(bool shouldPublishAvailabilityEvent) {
    if (shouldPublishAvailabilityEvent && getCurrentState() == IMUState::OPERATIONAL) {
        m_eventBus.publish(IMU_AvailabilityChanged(true));
    }
}

esp_err_t IMUService::attachAndConfigureCurrentProfile(bool publishAvailabilityEvent) {
    if (!m_hardwareController || !m_fifoProcessor || !m_healthMonitor || !m_calibration) {
        return ESP_ERR_INVALID_STATE;
    }

    bool shouldPublishAvailabilityEvent = false;
    const esp_err_t ret = [&]() {
        std::lock_guard<std::mutex> attachLock(m_attach_mutex);
        return attachAndConfigureCurrentProfileLocked(publishAvailabilityEvent, &shouldPublishAvailabilityEvent);
    }();

    publishAvailabilityIfOperational(shouldPublishAvailabilityEvent);
    return ret;
}

esp_err_t IMUService::attachAndConfigureCurrentProfileLocked(bool publishAvailabilityEvent,
                                                            bool* shouldPublishAvailabilityEvent) {
    MPU6050Config configCopy;
    MPU6050Profile profileCopy;
    if (shouldPublishAvailabilityEvent != nullptr) {
        *shouldPublishAvailabilityEvent = false;
    }
    {
        std::lock_guard<std::mutex> configLock(m_config_mutex);
        configCopy = m_config;
        profileCopy = m_profile;
    }

    const IMUState previousState = getCurrentState();
    const esp_err_t ret = m_hardwareController->connectAndConfigure(configCopy, profileCopy);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG,
                 "IMU attach/configure failed from state %s: %s",
                 stateToString(previousState),
                 esp_err_to_name(ret));
        return ret;
    }

    safeConfigUpdate([&]() {
        m_profile = profileCopy;
    });

    applyRuntimeProfile(configCopy, profileCopy, true);
    m_healthMonitor->resetFaultCounters();
    m_fault_reported.store(false, std::memory_order_release);

    transitionToState(IMUState::OPERATIONAL);
    if (getCurrentState() != IMUState::OPERATIONAL) {
        ESP_LOGE(TAG, "IMU attach completed but service failed to enter OPERATIONAL");
        return ESP_FAIL;
    }

    if (shouldPublishAvailabilityEvent != nullptr &&
        publishAvailabilityEvent &&
        previousState == IMUState::UNAVAILABLE) {
        *shouldPublishAvailabilityEvent = true;
    }

    return ESP_OK;
}

void IMUService::markSensorUnavailable(esp_err_t errorCode, bool publishErrorEvent) {
    const IMUState stateBeforeFault = getCurrentState();
    const bool firstFault = !m_fault_reported.exchange(true, std::memory_order_acq_rel);
    ESP_LOGE(TAG,
             "Marking IMU unavailable (state=%s code=%s first_fault=%s publish_error=%s)",
             stateToString(stateBeforeFault),
             esp_err_to_name(errorCode),
             firstFault ? "true" : "false",
             publishErrorEvent ? "true" : "false");
    const bool availabilityChanged = transitionToState(IMUState::UNAVAILABLE);

    if (availabilityChanged) {
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
    const bool hardwareConfigChanged = applyConfig(event.config);
    if (event.requiresHardwareInit != hardwareConfigChanged) {
        ESP_LOGW(TAG,
                 "IMU config update hardware-init hint mismatch (event=%s computed=%s)",
                 event.requiresHardwareInit ? "true" : "false",
                 hardwareConfigChanged ? "true" : "false");
    }
}

void IMUService::handleCalibrationRequest(const IMU_CalibrationRequest& event) {
    (void)event;
    if (!m_initialized.load(std::memory_order_acquire) || !m_calibration) {
        m_eventBus.publish(IMU_CalibrationRequestRejected(IMU_CalibrationRequestRejected::Reason::OTHER, false));
        return;
    }
    if (!m_calibration_allowed.load(std::memory_order_acquire)) {
        m_eventBus.publish(IMU_CalibrationRequestRejected(IMU_CalibrationRequestRejected::Reason::NOT_IDLE, true));
        return;
    }

    bool expected = false;
    if (!m_is_calibrating_flag.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
        m_eventBus.publish(IMU_CalibrationRequestRejected(IMU_CalibrationRequestRejected::Reason::OTHER, false));
        return;
    }
    ScopedAtomicFlagReset calibrationFlagReset(m_is_calibrating_flag);

    const bool wasUnavailable = getCurrentState() == IMUState::UNAVAILABLE;
    if (wasUnavailable) {
        const esp_err_t attachRet = attachAndConfigureCurrentProfile(false);
        if (attachRet != ESP_OK) {
            scheduleAutoAttachRetry(AUTO_ATTACH_RETRY_INTERVAL_US);
            m_eventBus.publish(IMU_CalibrationRequestRejected(IMU_CalibrationRequestRejected::Reason::OTHER, false));
            return;
        }
    }

    transitionToState(IMUState::CALIBRATION);
    if (getCurrentState() != IMUState::CALIBRATION) {
        m_eventBus.publish(IMU_CalibrationRequestRejected(IMU_CalibrationRequestRejected::Reason::OTHER, false));
        return;
    }

    const esp_err_t result = performCalibration();
    transitionToState(isAvailable() ? IMUState::OPERATIONAL : IMUState::UNAVAILABLE);
    m_eventBus.publish(IMU_CalibrationCompleted(result));

    if (wasUnavailable && result == ESP_OK && getCurrentState() == IMUState::OPERATIONAL) {
        m_eventBus.publish(IMU_AvailabilityChanged(true));
    }
}

void IMUService::handleAttachRequested(const IMU_AttachRequested& event) {
    (void)event;
    if (!m_initialized.load(std::memory_order_acquire)) {
        return;
    }
    if (getCurrentState() != IMUState::UNAVAILABLE) {
        return;
    }

    const esp_err_t ret = attachAndConfigureCurrentProfile(true);
    if (ret != ESP_OK) {
        scheduleAutoAttachRetry(AUTO_ATTACH_RETRY_INTERVAL_US);
    }
}

void IMUService::handleSystemPolicyChanged(const IMU_SystemPolicyChanged& event) {
    m_calibration_allowed.store(event.calibrationAllowed, std::memory_order_release);
    m_auto_attach_allowed.store(event.autoAttachAllowed, std::memory_order_release);
    m_hardware_config_apply_allowed.store(event.hardwareConfigApplyAllowed, std::memory_order_release);

    if (event.autoAttachAllowed && getCurrentState() == IMUState::UNAVAILABLE) {
        scheduleAutoAttachRetry(0);
    }
    applyPendingHardwareConfigIfSafe();
}

esp_err_t IMUService::performCalibration() {
    if (getCurrentState() != IMUState::CALIBRATION) {
        return ESP_ERR_INVALID_STATE;
    }

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
