#include "IMUService.hpp"
#include "IMUTask.hpp"
#include "I2CDevice.hpp"
#include "mpu6050.hpp"
#include "MPU6050HardwareController.hpp"
#include "FIFOProcessor.hpp"
#include "IMUCalibration.hpp"
#include "OrientationEstimator.hpp"
#include "EventBus.hpp"
#include "CONFIG_ImuConfigUpdate.hpp"
#include "CONFIG_BehaviorConfigUpdate.hpp"
#include "IMU_AttachRequested.hpp"
#include "IMU_SystemPolicyChanged.hpp"
#include "IMU_AvailabilityChanged.hpp"
#include "IMU_CalibrationRequest.hpp"
#include "IMU_CalibrationRequestRejected.hpp"
#include "IMU_CalibrationCompleted.hpp"
#include "IMU_GyroOffsetsUpdated.hpp"
#include "IMU_CommunicationError.hpp"
#include "esp_timer.h"
#include "UI_Stop.hpp"
#include "esp_log.h"
#include <algorithm>

IMUService::IMUService(std::shared_ptr<OrientationEstimator> estimator, const MPU6050Config& config,
    const SystemBehaviorConfig& behavior, EventBus& bus)
    : m_bus(bus), m_estimator(estimator), m_desired(config), m_applied(config), m_behavior(behavior) {}
IMUService::~IMUService() { stopTasks(); }
esp_err_t IMUService::init() {
    if (m_initialized) return ESP_OK;
    if (!m_estimator) return ESP_ERR_INVALID_ARG;
    m_device = std::make_unique<I2CDevice>();
    m_driver = std::make_unique<MPU6050Driver>(*m_device);
    m_hardware = std::make_unique<MPU6050HardwareController>(*m_device, *m_driver);
    m_fifo = std::make_unique<FIFOProcessor>(*m_driver, *m_estimator);
    m_calibration = std::make_unique<IMUCalibration>(*m_driver);
    m_task = std::make_unique<IMUTask>(*this);
    m_initialized = m_device && m_driver && m_hardware && m_fifo && m_calibration && m_task;
    return m_initialized ? ESP_OK : ESP_ERR_NO_MEM;
}
bool IMUService::startTasks() { return m_initialized && m_task->start(); }
void IMUService::stopTasks() { if (m_task) m_task->stop(); }
const char* IMUService::stateToString(IMUState state) {
    switch (state) {
        case IMUState::INITIALIZED: return "INITIALIZED";
        case IMUState::VALIDATING: return "VALIDATING";
        case IMUState::OPERATIONAL: return "OPERATIONAL";
        case IMUState::CALIBRATION: return "CALIBRATION";
        default: return "UNAVAILABLE";
    }
}
IMUState IMUService::getCurrentState() const { std::lock_guard<std::mutex> lock(m_mutex); return m_state; }
IMUStatusSnapshot IMUService::getStatusSnapshot() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    auto status = m_status;
    const auto sample = m_estimator->getOrientation();
    status.sampleTimestampUs = sample.sample_timestamp_us;
    status.sampleSequence = sample.sample_sequence;
    status.fifoRemainingPackets = sample.fifoRemainingPackets;
    status.ready = status.ready && sample.fresh(esp_timer_get_time(), m_behavior.imu_max_sample_age_ms * 1000LL);
    status.configurationPending = m_configPending;
    return status;
}
bool IMUService::isAvailable() const { return getStatusSnapshot().ready; }
bool IMUService::reserveMotion(uint32_t& generation, const char** rejectionReason) {
    std::lock_guard<std::mutex> lock(m_mutex);
    const auto sample = m_estimator->getOrientation();
    const char* reason = m_otaReserved ? "ota-reserved" :
        m_calibrationPending ? "calibration-pending" : m_configPending ? "configuration-pending" :
        !m_status.ready ? "imu-not-operational" : m_status.busy ? "imu-busy" :
        !sample.valid ? "estimate-invalid" :
        !sample.fresh(esp_timer_get_time(), m_behavior.imu_max_sample_age_ms * 1000LL) ? "estimate-stale-or-nonfinite" : nullptr;
    if (rejectionReason) *rejectionReason = reason;
    if (reason) return false;
    m_motionReserved = true;
    generation = sample.generation;
    return true;
}
void IMUService::releaseMotion() { std::lock_guard<std::mutex> lock(m_mutex); m_motionReserved = false; }
void IMUService::handleEvent(const BaseEvent& event) {
    bool reject = false;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (event.is<UI_Stop>()) {
            m_cancelCalibration = true;
        } else if (event.is<CONFIG_ImuConfigUpdate>()) {
            const auto& config = event.as<CONFIG_ImuConfigUpdate>().config;
            m_configPending = m_configPending || m_desired != config;
            m_desired = config;
        } else if (event.is<CONFIG_BehaviorConfigUpdate>()) {
            m_behavior = event.as<CONFIG_BehaviorConfigUpdate>().config;
        } else if (event.is<IMU_SystemPolicyChanged>()) {
            const auto& policy = event.as<IMU_SystemPolicyChanged>();
            m_applyAllowed = policy.hardwareConfigApplyAllowed;
            m_attachAllowed = policy.autoAttachAllowed;
            m_calibrationAllowed = policy.calibrationAllowed;
        } else if (event.is<IMU_AttachRequested>()) {
            // Wake the worker; state/policy and reconnect deadline decide eligibility.
        } else if (event.is<IMU_CalibrationRequest>()) {
            reject = m_otaReserved || !m_calibrationAllowed || m_calibrationPending || m_status.busy || m_motionReserved;
            if (!reject) { m_cancelCalibration = false; m_calibrationPending = true; m_status.busy = true; }
        }
    }
    if (reject) m_bus.publish(IMU_CalibrationRequestRejected(IMU_CalibrationRequestRejected::Reason::OTHER, false));
    if (m_task) m_task->wake();
}
void IMUService::transition(IMUState state, esp_err_t error, IMUFaultReason reason) {
    IMUStatusSnapshot status;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_state = state;
        m_status.state = stateToString(state);
        m_status.ready = state == IMUState::OPERATIONAL;
        m_status.busy = state == IMUState::CALIBRATION || state == IMUState::INITIALIZED || state == IMUState::VALIDATING;
        m_status.generation = m_estimator->getOrientation().generation;
        m_status.stateChangedUs = esp_timer_get_time();
        ++m_status.revision;
        if (error != ESP_OK || reason != IMUFaultReason::NONE) {
            m_status.lastError = error;
            m_status.lastReason = reason;
        }
        status = m_status;
    }
    m_bus.publish(IMU_AvailabilityChanged(status.ready, status.generation, status.revision));
}
void IMUService::invalidate(IMUFaultReason reason) {
    m_estimator->reset(); // Publish invalid before IRQ removal, events or I2C cleanup.
    transition(IMUState::INITIALIZED, ESP_OK, reason);
    while (m_irq.deinit() != ESP_OK) {
        // Keep the owner alive if the GPIO service cannot detach its callback.
        vTaskDelay(std::max<TickType_t>(1, pdMS_TO_TICKS(10)));
    }
}
void IMUService::unavailable(esp_err_t error, IMUFaultReason reason) {
    if (getCurrentState() != IMUState::INITIALIZED) invalidate(reason);
    transition(IMUState::UNAVAILABLE, error, reason);
    while (m_hardware->disconnect() != ESP_OK)
        vTaskDelay(std::max<TickType_t>(1, pdMS_TO_TICKS(10)));
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_nextAttemptUs = esp_timer_get_time() + m_behavior.imu_reconnect_interval_ms * 1000LL;
    }
    if (!m_incident) {
        m_incident = true;
        ESP_LOGW(TAG, "IMU unavailable: %s", esp_err_to_name(error));
        m_bus.publish(IMU_CommunicationError(error)); // Diagnostic only.
    }
}
esp_err_t IMUService::startStream(IMUTask& task) {
    // GPIO setup and synchronous subscribers may log or block. Finish them before
    // enabling sample production, otherwise startup itself creates a stale FIFO.
    esp_err_t ret = ESP_OK;
    if (m_applied.int_pin >= 0)
        ret = m_irq.init(static_cast<gpio_num_t>(m_applied.int_pin), m_applied.interrupt_active_high, IMUTask::interrupt, &task);
    if (ret != ESP_OK) return ret;
    m_validationSamples = 0;
    transition(IMUState::VALIDATING);
    ret = m_fifo->resync();
    m_lastProgressUs = esp_timer_get_time();
    m_validationDeadlineUs = m_lastProgressUs + 250000;
    return ret;
}
esp_err_t IMUService::attach(const MPU6050Config& config, IMUTask& task) {
    m_fullAttachValidating = true;
    invalidate(IMUFaultReason::CONFIGURATION);
    m_device->setConfig({10});
    { std::lock_guard<std::mutex> lock(m_mutex); ++m_status.reconnectAttempts; }
    const auto ret = m_hardware->connectAndConfigure(config, m_profile);
    if (ret != ESP_OK) return ret;
    m_applied = config;
    m_estimator->init(config.comp_filter_alpha, m_profile.samplePeriodS,
        config.gyro_offset_x, config.gyro_offset_y, config.gyro_offset_z);
    m_calibration->setOffsets(config.gyro_offset_x, config.gyro_offset_y, config.gyro_offset_z);
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_configPending = m_desired != config;
        m_status.busFrequencyHz = config.i2c_freq_hz;
        m_status.samplePeriodUs = m_profile.samplePeriodUs;
        m_fifo->configure(m_profile, config.fifo_read_threshold);
    }
    m_device->setConfig({5});
    m_resyncUsed = false;
    return startStream(task);
}
void IMUService::calibrate(IMUTask& task) {
    m_fullAttachValidating = false;
    invalidate(IMUFaultReason::CALIBRATION);
    transition(IMUState::CALIBRATION);
    esp_err_t result = m_driver->disableFIFO();
    if (result == ESP_OK) result = m_calibration->calibrate(m_profile, m_applied.calibration_samples,
        nullptr, [&] { return task.stopping() || m_cancelCalibration.load() || !m_calibrationAllowed.load(); });
    if (result == ESP_OK) {
        const float x = m_calibration->getGyroOffsetXDPS(), y = m_calibration->getGyroOffsetYDPS(), z = m_calibration->getGyroOffsetZDPS();
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_desired.gyro_offset_x = m_applied.gyro_offset_x = x;
            m_desired.gyro_offset_y = m_applied.gyro_offset_y = y;
            m_desired.gyro_offset_z = m_applied.gyro_offset_z = z;
        }
        m_bus.publish(IMU_GyroOffsetsUpdated(x, y, z));
    }
    m_estimator->init(m_applied.comp_filter_alpha, m_profile.samplePeriodS,
        m_applied.gyro_offset_x, m_applied.gyro_offset_y, m_applied.gyro_offset_z);
    if (result != ESP_OK) unavailable(result, IMUFaultReason::CALIBRATION);
    else {
        const auto ret = startStream(task);
        if (ret != ESP_OK) unavailable(ret, IMUFaultReason::CALIBRATION);
    }
    m_bus.publish(IMU_CalibrationCompleted(result));
}
void IMUService::applySoftwareConfiguration(const MPU6050Config& config) {
    const bool estimatorChanged = config.comp_filter_alpha != m_applied.comp_filter_alpha ||
        config.gyro_offset_x != m_applied.gyro_offset_x || config.gyro_offset_y != m_applied.gyro_offset_y ||
        config.gyro_offset_z != m_applied.gyro_offset_z;
    m_applied = config;
    m_fifo->configure(m_profile, config.fifo_read_threshold);
    m_calibration->setOffsets(config.gyro_offset_x, config.gyro_offset_y, config.gyro_offset_z);
    if (estimatorChanged) {
        m_estimator->init(config.comp_filter_alpha, m_profile.samplePeriodS,
            config.gyro_offset_x, config.gyro_offset_y, config.gyro_offset_z);
        m_validationSamples = 0;
        m_validationDeadlineUs = esp_timer_get_time() + 250000;
        m_fullAttachValidating = false;
        transition(IMUState::VALIDATING);
    }
    std::lock_guard<std::mutex> lock(m_mutex);
    m_configPending = m_desired != config;
    m_status.busy = m_state != IMUState::OPERATIONAL;
}
void IMUService::runWorker(IMUTask& task) {
    bool initial = true;
    while (!task.stopping()) {
        const auto iterationStartUs = esp_timer_get_time();
        bool moreData = false;
        MPU6050Config config;
        SystemBehaviorConfig behavior;
        bool doAttach = false, doCalibration = false;
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            behavior = m_behavior;
            const bool unavailableState = m_state == IMUState::UNAVAILABLE;
            doAttach = !m_otaReserved && !m_motionReserved && (initial ||
                (!unavailableState && m_configPending && m_applyAllowed && !m_calibrationPending) ||
                (unavailableState && m_attachAllowed && esp_timer_get_time() >= m_nextAttemptUs));
            // Explicit attach coalesces; it never bypasses the reconnect interval.
            if (doAttach) { config = m_desired; m_status.busy = true; }
            doCalibration = !doAttach && m_calibrationPending && !m_motionReserved;
            if (doCalibration) m_calibrationPending = false;
        }
        initial = false;
        if (doAttach) {
            if (m_device->isOpen() && getCurrentState() != IMUState::UNAVAILABLE &&
                !m_applied.requiresHardwareInit(config)) {
                applySoftwareConfiguration(config);
            } else {
                const auto ret = attach(config, task);
                if (ret != ESP_OK) unavailable(ret, IMUFaultReason::TRANSPORT);
            }
        } else if (doCalibration) {
            if (!m_calibrationAllowed || !m_device->isOpen()) {
                transition(m_device->isOpen() ? IMUState::VALIDATING : IMUState::UNAVAILABLE);
                m_bus.publish(IMU_CalibrationRequestRejected(IMU_CalibrationRequestRejected::Reason::OTHER, false));
            } else calibrate(task);
        }
        const auto state = getCurrentState();
        if (state == IMUState::OPERATIONAL || state == IMUState::VALIDATING) {
            const auto sample = m_estimator->getOrientation();
            const int64_t now = esp_timer_get_time();
            const int64_t maxAge = behavior.imu_max_sample_age_ms * 1000LL;
            // Read the FIFO before judging the previous snapshot: fresh packets
            // may already be waiting after a delayed worker wakeup.
            if (state == IMUState::VALIDATING && now >= m_validationDeadlineUs) {
                unavailable(ESP_ERR_TIMEOUT, IMUFaultReason::VALIDATION);
            } else {
                const auto result = m_fifo->processFIFO(sample.generation,
                    sample.sample_timestamp_us ? sample.sample_timestamp_us + maxAge : now + 5000);
                moreData = result.moreData;
                if (result.accepted) {
                    m_lastProgressUs = esp_timer_get_time();
                    // Acquisition is healthy even while acceleration prevents initialization.
                    m_validationDeadlineUs = m_lastProgressUs + 250000;
                }
                if (result.retried || (result.error != ESP_OK && result.reason == IMUFaultReason::TRANSPORT)) {
                    std::lock_guard<std::mutex> lock(m_mutex);
                    ++m_status.transportErrors;
                }
                if (result.outcome == FIFOOutcome::LOST_ALIGNMENT) {
                    invalidate(result.reason);
                    ESP_LOGW(TAG, "FIFO repair: reason=%u error=%s", static_cast<unsigned>(result.reason), esp_err_to_name(result.error));
                    if (m_resyncUsed) unavailable(result.error, result.reason);
                    else {
                        m_resyncUsed = true;
                        { std::lock_guard<std::mutex> lock(m_mutex); ++m_status.fifoResyncs; }
                        const auto ret = startStream(task);
                        if (ret != ESP_OK) unavailable(ret, result.reason);
                    }
                } else if (result.outcome == FIFOOutcome::HARDWARE_FAILURE) {
                    unavailable(result.error, result.reason);
                } else if ((result.outcome == FIFOOutcome::NO_DATA || result.outcome == FIFOOutcome::READ_FAILURE) &&
                    esp_timer_get_time() - m_lastProgressUs >= 250000) {
                    // The 20 ms deadline inhibits motion, not the bus. Only a
                    // sustained absence of acquisition progress needs reconnect.
                    unavailable(ESP_ERR_TIMEOUT, IMUFaultReason::STALE);
                } else if (result.outcome == FIFOOutcome::ACCEPTED && m_estimator->getOrientation().gyroContinuityLost) {
                    { std::lock_guard<std::mutex> lock(m_mutex); ++m_status.gyroClippingResets; }
                    // Clipping loses angular history, not FIFO framing. Revoke the old
                    // generation before revalidating, without touching MPU/FIFO/IRQ.
                    m_estimator->reset();
                    m_validationSamples = 0;
                    m_fullAttachValidating = false;
                    transition(IMUState::VALIDATING, ESP_ERR_INVALID_RESPONSE, IMUFaultReason::VALIDATION);
                } else if (result.outcome == FIFOOutcome::ACCEPTED && state == IMUState::VALIDATING) {
                    m_validationSamples += result.accepted;
                    const auto latest = m_estimator->getOrientation();
                    if (m_validationSamples >= 5 && esp_timer_get_time() - latest.sample_timestamp_us <= maxAge &&
                        m_estimator->setValidated()) {
                        if (m_fullAttachValidating) {
                            std::lock_guard<std::mutex> lock(m_mutex); ++m_status.reconnectSuccesses;
                        }
                        m_fullAttachValidating = false;
                        m_resyncUsed = false;
                        m_incident = false;
                        transition(IMUState::OPERATIONAL);
                        ESP_LOGD(TAG, "IMU stream validated");
                    }
                }
            }
        }
        const int64_t remainingUs = 5000 - (esp_timer_get_time() - iterationStartUs);
        const bool woke = task.wait(moreData ? 1 : remainingUs > 0 ? (remainingUs + 999) / 1000 : 1);
        if (!woke && !moreData && m_profile.interruptEnabled &&
            (getCurrentState() == IMUState::OPERATIONAL || getCurrentState() == IMUState::VALIDATING)) {
            std::lock_guard<std::mutex> lock(m_mutex); ++m_status.irqFallbacks;
        }
    }
    invalidate(IMUFaultReason::STOPPED);
    if (m_device->isOpen()) {
        const auto ret = m_driver->disableFIFO();
        if (ret != ESP_OK) ESP_LOGW(TAG, "IMU shutdown cleanup: %s", esp_err_to_name(ret));
    }
    while (m_hardware->disconnect() != ESP_OK)
        vTaskDelay(std::max<TickType_t>(1, pdMS_TO_TICKS(10)));
    transition(IMUState::UNAVAILABLE, ESP_OK, IMUFaultReason::STOPPED);
}

bool IMUService::reserveOta() {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_otaReserved) return true;
    if (m_motionReserved || m_status.busy || m_calibrationPending) return false;
    m_otaReserved = true;
    return true;
}
void IMUService::releaseOta() {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_otaReserved = false;
}
