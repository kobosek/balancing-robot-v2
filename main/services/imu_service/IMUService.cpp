// main/services/IMUService.cpp

#include "IMUService.hpp"
#include "IMUCalibration.hpp"
#include "mpu6050.hpp"
#include "OrientationEstimator.hpp"
#include "SYSTEM_StateChanged.hpp"
#include "IMU_RecoverySucceeded.hpp"
#include "IMU_RecoveryFailed.hpp"
#include "MOTION_UsingFallbackValues.hpp"
#include "CONFIG_ImuConfigUpdate.hpp" 
#include "IMUHealthMonitor.hpp"
#include "ConfigData.hpp"
#include "EventBus.hpp"
#include "IMU_GyroOffsetsUpdated.hpp"
#include "IMU_CalibrationRequestRejected.hpp"
#include "EventTypes.hpp"
#include "IMU_CommunicationError.hpp"
#include "CONFIG_FullConfigUpdate.hpp" 

#include "IMU_CalibrationRequest.hpp"
#include "IMU_CalibrationCompleted.hpp"
#include "BaseEvent.hpp"
#include "driver/gpio.h"
#include "esp_check.h"
#include <cmath>
#include <vector>
#include "freertos/task.h" 
#include <algorithm>      
#include "FIFOProcessor.hpp"
#include "FIFOTask.hpp"
#include "HealthMonitorTask.hpp"
#include "IMUHealthMonitor.hpp"

// Constructor Implementation
IMUService::IMUService(std::shared_ptr<OrientationEstimator> estimator,
                       const MPU6050Config& config,
                       const SystemBehaviorConfig& behaviorConfig,
                       EventBus& bus) :
    m_estimator(estimator),
    m_config(config),
    m_behaviorConfig(behaviorConfig),
    m_eventBus(bus),
    m_is_calibrating_flag(false),
    m_current_state(IMUState::INITIALIZED),
    m_system_state(SystemState::INIT),
    m_recovery_attempt_in_progress(false),
    m_recovery_start_time_ms(0),
    m_recovery_timeout_ms(5000), 
    m_accel_lsb_per_g(1.0f), 
    m_gyro_lsb_per_dps(1.0f),
    m_sample_period_s(1.0f / 100.0f), // Default 100Hz
    m_isr_handler_installed(false)
{

}

IMUService::~IMUService() {
    ESP_LOGI(TAG, "Deconstructing IMUService...");
    if (m_isr_handler_installed) { 
        if (m_config.int_pin >= 0 && m_config.int_pin < GPIO_NUM_MAX) {
            // Delegate interrupt unregistration to FIFOProcessor
            esp_err_t ret = m_fifoProcessor->unregisterInterrupt(m_config.int_pin);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "FIFOProcessor removed ISR handler for pin %d", (int)m_config.int_pin);
            } else {
                ESP_LOGE(TAG, "Error while FIFOProcessor was removing ISR handler for pin %d: %s", 
                         (int)m_config.int_pin, esp_err_to_name(ret));
            }
        }
        m_isr_handler_installed = false; 
    } else {
         ESP_LOGI(TAG, "ISR handler not removed (was not successfully installed or pin invalid).");
    }
    stopTasks();
    ESP_LOGI(TAG, "IMUService deconstructed.");
}


void IMUService::applyConfig(const MPU6050Config& newConfig) {
    bool needsEstimatorReinit = false;
    bool needsScalingFactorRecalc = false;
    bool hardwareConfigChanged = false;
    bool gyroOffsetsChanged = false;

    if (m_config.comp_filter_alpha != newConfig.comp_filter_alpha ||
        m_config.sample_rate_divisor != newConfig.sample_rate_divisor) { // sample_rate_divisor affects m_sample_period_s
        needsEstimatorReinit = true; // Triggers full estimator.init()
    }
    if (m_config.gyro_offset_x != newConfig.gyro_offset_x ||
        m_config.gyro_offset_y != newConfig.gyro_offset_y ||
        m_config.gyro_offset_z != newConfig.gyro_offset_z) {
        gyroOffsetsChanged = true; // Triggers estimator.updateGyroOffsets()
    }

    if (m_config.accel_range != newConfig.accel_range ||
        m_config.gyro_range != newConfig.gyro_range ||
        m_config.sample_rate_divisor != newConfig.sample_rate_divisor || // Also checked for needsEstimatorReinit
        m_config.dlpf_config != newConfig.dlpf_config) {
        needsScalingFactorRecalc = true;
        hardwareConfigChanged = true;
    }

    m_config = newConfig;
    ESP_LOGI(TAG, "Applied new IMU config. HW changed: %s, EstimatorReinit: %s, GyroOffsetsChanged: %s",
             hardwareConfigChanged ? "Yes" : "No", needsEstimatorReinit ? "Yes" : "No", gyroOffsetsChanged ? "Yes" : "No");

    if (needsScalingFactorRecalc || needsEstimatorReinit) { // calculateScalingFactors updates m_sample_period_s
        calculateScalingFactors();
    }

    // If full reinit is needed, it includes offset setting.
    // Otherwise, if only offsets changed, update them specifically.
    if (needsEstimatorReinit) {
        m_estimator->init(m_config.comp_filter_alpha, m_sample_period_s,
                         m_config.gyro_offset_x, m_config.gyro_offset_y, m_config.gyro_offset_z);
        ESP_LOGI(TAG, "Re-initialized OrientationEstimator due to config change.");
    } else if (gyroOffsetsChanged) {
        m_estimator->updateGyroOffsets(m_config.gyro_offset_x, m_config.gyro_offset_y, m_config.gyro_offset_z);
        ESP_LOGI(TAG, "Updated OrientationEstimator gyro offsets due to config change.");
    }

    if (m_current_state.load(std::memory_order_acquire) == IMUState::CALIBRATION && hardwareConfigChanged) {
        ESP_LOGI(TAG, "Hardware config parameters changed during CALIBRATION. Hardware will be reconfigured upon transitioning to OPERATIONAL.");
    }
}

void IMUService::applyConfig(const SystemBehaviorConfig& config) {
    m_healthMonitor->applyConfig(config);
}

void IMUService::handleConfigUpdate(const CONFIG_FullConfigUpdate& event) {
    ESP_LOGD(TAG, "Handling general config update event (CONFIG_FullConfigUpdate).");
    
    MPU6050Config oldImuConfig = m_config; 
    applyConfig(event.configData.imu); 
    applyConfig(event.configData.behavior); 

    bool criticalHardwareChanged = (oldImuConfig.accel_range != m_config.accel_range ||
                                   oldImuConfig.gyro_range != m_config.gyro_range ||
                                   oldImuConfig.dlpf_config != m_config.dlpf_config ||
                                   oldImuConfig.sample_rate_divisor != m_config.sample_rate_divisor ||
                                   oldImuConfig.i2c_port != m_config.i2c_port || 
                                   oldImuConfig.sda_pin != m_config.sda_pin ||
                                   oldImuConfig.scl_pin != m_config.scl_pin ||
                                   oldImuConfig.int_pin != m_config.int_pin);

    if (criticalHardwareChanged && m_current_state.load() != IMUState::RECOVERY) {
        ESP_LOGW(TAG, "Critical hardware parameters changed via CONFIG_FullConfigUpdate. Transitioning to RECOVERY to apply changes.");
        transitionToState(IMUState::RECOVERY);
    }
}

void IMUService::handleIMUConfigUpdate(const CONFIG_ImuConfigUpdate& event) {
    ESP_LOGI(TAG, "Handling granular IMU config update event (CONFIG_ImuConfigUpdate). Hardware init required: %s.",
             event.requiresHardwareInit ? "Yes" : "No");
    
    applyConfig(event.config); 
    
    if (event.requiresHardwareInit && m_current_state.load() != IMUState::RECOVERY) {
        ESP_LOGI(TAG, "Hardware parameters changed, transitioning to RECOVERY state to apply.");
        transitionToState(IMUState::RECOVERY);
    }
}


void IMUService::calculateScalingFactors() {
    // Using the baseline calculation method (no averaging) to get the current scaling factors:
    switch(m_config.accel_range) {
        case 0: m_accel_lsb_per_g = 16384.0f; break;
        case 1: m_accel_lsb_per_g = 8192.0f;  break;
        case 2: m_accel_lsb_per_g = 4096.0f;  break;
        case 3: m_accel_lsb_per_g = 2048.0f;  break;
        default: ESP_LOGW(TAG, "Unknown accel range %d, using default scale (8192 LSB/g)!", m_config.accel_range); m_accel_lsb_per_g = 8192.0f; break;
    }
    ESP_LOGD(TAG, "Accel scale set to: %.1f LSB/g for range %d", m_accel_lsb_per_g, m_config.accel_range);

    switch(m_config.gyro_range) {
        case 0: m_gyro_lsb_per_dps = 131.0f; break;
        case 1: m_gyro_lsb_per_dps = 65.5f;  break;
        case 2: m_gyro_lsb_per_dps = 32.8f;  break;
        case 3: m_gyro_lsb_per_dps = 16.4f;  break;
        default: ESP_LOGW(TAG, "Unknown gyro range %d, using default scale (65.5 LSB/dps)!", m_config.gyro_range); m_gyro_lsb_per_dps = 65.5f; break;
    }
    ESP_LOGD(TAG, "Gyro scale set to: %.1f LSB/dps for range %d", m_gyro_lsb_per_dps, m_config.gyro_range);

    bool dlpf_enabled = (m_config.dlpf_config >= 1 && m_config.dlpf_config <= 6);
    float gyro_output_rate_hz = dlpf_enabled ? 1000.0f : 8000.0f;
    if ((1.0f + m_config.sample_rate_divisor) <= 1e-6) { 
        ESP_LOGE(TAG, "Invalid sample rate divisor (%d), cannot calculate sample period! Defaulting to 1ms (1kHz).", m_config.sample_rate_divisor);
        m_sample_period_s = 0.001f; 
    } else {
        m_sample_period_s = 1.0f / (gyro_output_rate_hz / (1.0f + m_config.sample_rate_divisor));
    }
    ESP_LOGI(TAG, "Calculated Sample Period: %.6f s (Gyro Rate: %.0f Hz, Divisor: %d, DLPF Conf: %d)",
             m_sample_period_s, gyro_output_rate_hz, m_config.sample_rate_divisor, m_config.dlpf_config);
    
    // Update the FIFOProcessor with the new scaling factors
    m_fifoProcessor->setScalingFactors(m_accel_lsb_per_g, m_gyro_lsb_per_dps);

}


esp_err_t IMUService::configureSensorHardware() {
    ESP_LOGI(TAG, "Configuring MPU6050 hardware settings using current m_config...");
    esp_err_t ret;

    ret = m_driver->setPowerManagementReg(MPU6050PowerManagement::CLOCK_INTERNAL); 
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to set power management/clock source");
    vTaskDelay(pdMS_TO_TICKS(50)); 

    MPU6050AccelConfig accelRegValue;
    switch(m_config.accel_range) {
        case 0: accelRegValue = MPU6050AccelConfig::RANGE_2G; break;
        case 1: accelRegValue = MPU6050AccelConfig::RANGE_4G; break;
        case 2: accelRegValue = MPU6050AccelConfig::RANGE_8G; break;
        case 3: accelRegValue = MPU6050AccelConfig::RANGE_16G; break;
        default:
            ESP_LOGW(TAG, "Invalid accel_range %d in m_config, defaulting to 4G for hardware!", m_config.accel_range);
            accelRegValue = MPU6050AccelConfig::RANGE_4G; break;
    }

    MPU6050GyroConfig gyroRegValue;
    switch(m_config.gyro_range) {
        case 0: gyroRegValue = MPU6050GyroConfig::RANGE_250_DEG; break;
        case 1: gyroRegValue = MPU6050GyroConfig::RANGE_500_DEG; break;
        case 2: gyroRegValue = MPU6050GyroConfig::RANGE_1000_DEG; break;
        case 3: gyroRegValue = MPU6050GyroConfig::RANGE_2000_DEG; break;
        default:
             ESP_LOGW(TAG, "Invalid gyro_range %d in m_config, defaulting to 500DPS for hardware!", m_config.gyro_range);
             gyroRegValue = MPU6050GyroConfig::RANGE_500_DEG; break;
    }

    uint8_t dlpf_val = m_config.dlpf_config;
    if (dlpf_val > static_cast<uint8_t>(MPU6050DLPFConfig::DLPF_BW_5HZ_ACC_5HZ_GYRO)) { 
        ESP_LOGW(TAG, "Invalid dlpf_config %d, defaulting to 0 (260Hz)", dlpf_val);
        dlpf_val = 0;
    }
    MPU6050DLPFConfig dlpf = static_cast<MPU6050DLPFConfig>(dlpf_val);
    
    MPU6050SampleRateDiv rateDiv = static_cast<MPU6050SampleRateDiv>(m_config.sample_rate_divisor);

    ESP_LOGI(TAG, "Setting HW: AccelRange=%d, GyroRange=%d, DLPF=%d, SampRateDiv=%d", 
            static_cast<int>(accelRegValue), static_cast<int>(gyroRegValue), static_cast<int>(dlpf), static_cast<int>(rateDiv));

    ret = m_driver->setAccelRangeReg(accelRegValue); ESP_RETURN_ON_ERROR(ret, TAG, "Failed set Accel Range Reg");
    ret = m_driver->setGyroRangeReg(gyroRegValue);   ESP_RETURN_ON_ERROR(ret, TAG, "Failed set Gyro Range Reg");
    ret = m_driver->setDLPFConfigReg(dlpf);       ESP_RETURN_ON_ERROR(ret, TAG, "Failed set DLPF Reg");
    ret = m_driver->setSampleRateDivReg(rateDiv); ESP_RETURN_ON_ERROR(ret, TAG, "Failed set Sample Rate Div Reg");

    if (m_config.int_pin >= 0 && m_config.int_pin < GPIO_NUM_MAX) {
        MPU6050InterruptPinConfig intPinConfigVal = m_config.interrupt_active_high ?
            MPU6050InterruptPinConfig::ACTIVE_HIGH : MPU6050InterruptPinConfig::ACTIVE_LOW;
        ret = m_driver->configureInterruptPinReg(intPinConfigVal, MPU6050Interrupt::DATA_READY);
        ESP_RETURN_ON_ERROR(ret, TAG, "Failed setup MPU Interrupt Reg");
        ESP_LOGI(TAG,"MPU Interrupt Pin configured for DATA_READY.");
    } else {
        ESP_LOGW(TAG, "Interrupt pin (%d) not configured in MPU6050 hardware (disabling MPU INT).", m_config.int_pin);
         ret = m_driver->configureInterruptPinReg(MPU6050InterruptPinConfig::ACTIVE_HIGH, (MPU6050Interrupt)0); 
         ESP_RETURN_ON_ERROR(ret, TAG, "Failed to disable MPU Interrupt Reg");
    }

    ret = m_driver->resetFIFO(); 
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed reset MPU FIFO");
    vTaskDelay(pdMS_TO_TICKS(2)); 
    ret = m_driver->resetSignalPath(); 
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed reset MPU Signal Paths");
    vTaskDelay(pdMS_TO_TICKS(2));
    ret = m_driver->disableFIFO();
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to disable FIFO");


    ESP_LOGI(TAG, "MPU6050 hardware configuration complete.");
    return ESP_OK;
}

esp_err_t IMUService::init() {
    ESP_LOGI(TAG, "Initializing IMU Service");

    // Make sure m_config is valid
    if (m_config.accel_range == 0 || m_config.gyro_range == 0) {
        ESP_LOGE(TAG, "Invalid IMU config detected: accel_range=%d, gyro_range=%d",
                 m_config.accel_range, m_config.gyro_range);
        return ESP_ERR_INVALID_ARG;
    }

    // Set IMU state to INITIALIZED 
    m_current_state.store(IMUState::INITIALIZED, std::memory_order_relaxed);

    m_driver = std::make_unique<MPU6050Driver>(); 

    ESP_LOGI(TAG, "Initializing MPU6050 hardware");
    // Initialize the actual driver now
    esp_err_t ret = m_driver->init(m_config.i2c_port, m_config.sda_pin, m_config.scl_pin, m_config.device_address, m_config.i2c_freq_hz);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 init failed: %s", esp_err_to_name(ret));
        return ret;
    }    
    
    // Create owned components
    m_healthMonitor = std::make_unique<IMUHealthMonitor>(*m_driver, m_eventBus, m_behaviorConfig);
    
    // Create FIFOProcessor after health monitor
    m_fifoProcessor = std::make_unique<FIFOProcessor>(
        *m_driver,
        m_estimator,
        *m_healthMonitor,
        m_eventBus
    );

    m_fifoTask = std::make_unique<FIFOTask>(*m_fifoProcessor);

    m_healthMonitorTask = std::make_unique<HealthMonitorTask>(*m_healthMonitor);

    m_calibration = std::make_unique<IMUCalibration>(*m_driver);

    // Configure the hardware according to our config
    ret = configureSensorHardware();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 configuration failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "IMU Service initialized successfully");

    // Create the tasks (but don't start them yet)

    
    if (!m_fifoTask || !m_healthMonitorTask) {
        ESP_LOGE(TAG, "Failed to create IMU tasks");
        return ESP_ERR_NO_MEM;
    }

    // IMU initially transitions to OPERATIONAL state after init
    transitionToState(IMUState::OPERATIONAL);

    return ESP_OK;
} 

bool IMUService::startTasks() {
    ESP_LOGI(TAG, "Starting IMU tasks");
    
    // Start FIFO task with high priority
    if (!m_fifoTask->start(configMAX_PRIORITIES - 2, 0, 6144)) {
        ESP_LOGE(TAG, "Failed to start FIFO task!");
        return false;
    }
    
    // Start Health Monitor task with medium priority
    if (!m_healthMonitorTask->start(5, 1, 4096)) {
        ESP_LOGE(TAG, "Failed to start Health Monitor task!");
        // Stop FIFO task if it was started
        m_fifoTask->stop();
        return false;
    }
    
    ESP_LOGI(TAG, "All IMU tasks started successfully");
    return true;
}

void IMUService::stopTasks() {
    ESP_LOGI(TAG, "Stopping IMU tasks");
    if (m_fifoTask) m_fifoTask->stop();
    if (m_healthMonitorTask) m_healthMonitorTask->stop();
}

// EventHandler implementation
void IMUService::handleEvent(const BaseEvent& event) {
    // Central event handler that dispatches to specific handlers based on event type
    switch (event.type) {
        case EventType::CONFIG_FULL_UPDATE:
            handleConfigUpdate(static_cast<const CONFIG_FullConfigUpdate&>(event));
            break;
            
        case EventType::CONFIG_IMU_UPDATE:
            handleIMUConfigUpdate(static_cast<const CONFIG_ImuConfigUpdate&>(event));
            break;
            
        case EventType::IMU_COMMUNICATION_ERROR:
            handleImuCommunicationError(static_cast<const IMU_CommunicationError&>(event));
            break;
            
        case EventType::IMU_CALIBRATION_REQUEST:
            handleCalibrationRequest(static_cast<const IMU_CalibrationRequest&>(event));
            break;
            
        case EventType::SYSTEM_STATE_CHANGED:
            handleSystemStateChanged(static_cast<const SYSTEM_StateChanged&>(event));
            break;
            
        default:
            ESP_LOGV(TAG, "%s: Received unhandled event type %d", 
                     getHandlerName().c_str(), static_cast<int>(event.type));
            break;
    }
}

void IMUService::handleSystemStateChanged(const SYSTEM_StateChanged& event) {
    SystemState oldSystemState = m_system_state.exchange(event.newState);
    
    ESP_LOGI(TAG, "System state changed from: %d to: %d", static_cast<int>(oldSystemState), static_cast<int>(event.newState));
    
    if (m_current_state.load() == IMUState::RECOVERY && 
        (event.newState == SystemState::BALANCING || event.newState == SystemState::FALLEN)) {
        
        ESP_LOGW(TAG, "System state requires operational IMU (%d) while IMU is in RECOVERY.", 
                 static_cast<int>(event.newState));
        
        if (m_recovery_attempt_in_progress) {
            uint32_t current_time_ms = esp_timer_get_time() / 1000;
            if ((current_time_ms - m_recovery_start_time_ms > m_recovery_timeout_ms)) {
                ESP_LOGE(TAG, "Recovery timeout (%lu ms) exceeded during critical system state. Aborting current recovery attempt.", m_recovery_timeout_ms);
                
                m_recovery_attempt_in_progress = false; 
                m_eventBus.publish(IMU_RecoveryFailed(ESP_ERR_TIMEOUT)); 
                
                ESP_LOGW(TAG, "IMU remains in RECOVERY state after timeout. Manual intervention or new recovery command needed.");
            }
        }
    }
}

void IMUService::handleImuCommunicationError(const IMU_CommunicationError& event) {
    ESP_LOGE(TAG, "IMU Communication error detected (Code: %s). Transitioning to RECOVERY.", esp_err_to_name(event.errorCode));
    
    // Store current system state before recovery
    SystemState currentSystemState = m_system_state.load();
    
    // Transition to RECOVERY state if not already there
    if (m_current_state.load() != IMUState::RECOVERY) {
        transitionToState(IMUState::RECOVERY);
    }
    
    // Initiate recovery with minimal interruption if we're balancing
    bool minimal_interruption = (currentSystemState == SystemState::BALANCING);
    
    // Create recovery config
    IMURecoveryConfig config;
    config.minimal_interruption = minimal_interruption;
    config.max_attempts = 3; // Default value, could be configurable
    config.retry_delay_ms = 250; // Default value, could be configurable

    startRecoveryProcess(config);
}

const char* IMUService::stateToString(IMUState state) {
    switch(state) {
        case IMUState::INITIALIZED: return "INITIALIZED";
        case IMUState::OPERATIONAL: return "OPERATIONAL";
        case IMUState::CALIBRATION: return "CALIBRATION";
        case IMUState::RECOVERY:    return "RECOVERY";
        default: return "UNKNOWN_IMU_STATE";
    }
}

bool IMUService::isValidTransition(IMUState from, IMUState to) {
    if (from == to) {
        ESP_LOGD(TAG, "Attempted same-state transition to %s. Usually no-op.", stateToString(to));
        return false; 
    }
    
    switch(from) {
        case IMUState::INITIALIZED:
            return (to == IMUState::OPERATIONAL || to == IMUState::RECOVERY);
        case IMUState::OPERATIONAL:
            return (to == IMUState::CALIBRATION || to == IMUState::RECOVERY);
        case IMUState::CALIBRATION:
            return (to == IMUState::OPERATIONAL || to == IMUState::RECOVERY);
        case IMUState::RECOVERY:
            return (to == IMUState::OPERATIONAL || to == IMUState::INITIALIZED); 
        default:
            ESP_LOGE(TAG, "isValidTransition: Unknown 'from' state: %d", static_cast<int>(from));
            return false;
    }
}

void IMUService::transitionToState(IMUState newState) {
    IMUState oldState = m_current_state.load(std::memory_order_relaxed);
    
    if (oldState == newState) {
        ESP_LOGD(TAG, "Already in state %s. No transition needed.", stateToString(newState));
        return;
    }

    if (!isValidTransition(oldState, newState)) {
        ESP_LOGW(TAG, "Invalid state transition requested from %s to %s. Aborting.", stateToString(oldState), stateToString(newState));
        return;
    }
    ESP_LOGI(TAG, "Transitioning IMU state from %s to %s", stateToString(oldState), stateToString(newState));
    
    esp_err_t exit_ret = ESP_OK;
    switch(oldState) {
        case IMUState::INITIALIZED: exit_ret = exitInitializedState(); break;
        case IMUState::OPERATIONAL: exit_ret = exitOperationalState(); break;
        case IMUState::CALIBRATION: exit_ret = exitCalibrationState(); break;
        case IMUState::RECOVERY:    exit_ret = exitRecoveryState();    break;
    }
    if (exit_ret != ESP_OK) {
        ESP_LOGW(TAG, "Warning during exit from state %s: %s. Proceeding with transition.", stateToString(oldState), esp_err_to_name(exit_ret));
    }
    
    esp_err_t enter_ret = ESP_OK;
    switch(newState) {
        case IMUState::INITIALIZED: enter_ret = enterInitializedState(); break;
        case IMUState::OPERATIONAL: enter_ret = enterOperationalState(); break;
        case IMUState::CALIBRATION: enter_ret = enterCalibrationState(); break;
        case IMUState::RECOVERY:    enter_ret = enterRecoveryState();    break;
    }
    
    if (enter_ret != ESP_OK) {
        ESP_LOGE(TAG, "Error entering state %s: %s. Attempting to revert to previous state %s.", 
                 stateToString(newState), esp_err_to_name(enter_ret), stateToString(oldState));
        esp_err_t revert_enter_ret = ESP_OK;
        switch(oldState) { 
            case IMUState::INITIALIZED: revert_enter_ret = enterInitializedState(); break;
            case IMUState::OPERATIONAL: revert_enter_ret = enterOperationalState(); break;
            case IMUState::CALIBRATION: revert_enter_ret = enterCalibrationState(); break;
            case IMUState::RECOVERY:    revert_enter_ret = enterRecoveryState();    break;
        }
        if (revert_enter_ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to revert to state %s: %s. IMU state might be inconsistent!", stateToString(oldState), esp_err_to_name(revert_enter_ret));
            newState = IMUState::RECOVERY;
        }
    }
    
    m_current_state.store(newState, std::memory_order_release);
    m_healthMonitor->notifyIMUStateChange(newState);
    
    ESP_LOGI(TAG, "Successfully transitioned IMU to state %s", stateToString(newState));
}

esp_err_t IMUService::enterInitializedState() {
    ESP_LOGI(TAG, "Entering INITIALIZED state.");
    return m_fifoProcessor->disableFIFO();
}

esp_err_t IMUService::exitInitializedState() {
    ESP_LOGD(TAG, "Exiting INITIALIZED state.");
    return ESP_OK;
}

esp_err_t IMUService::enterOperationalState() {
    ESP_LOGI(TAG, "Entering OPERATIONAL state...");
    esp_err_t ret;

    // Set up ISR for in-task data processing with automatic ISR registration
    if (m_config.int_pin >= 0 && m_config.int_pin < GPIO_NUM_MAX) {
        // Use the overloaded method that also registers the ISR
        // Assume active high by default - most common for MPU6050
        bool active_high = true; 
        ret = m_fifoProcessor->enableFIFO(m_config.int_pin, active_high);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to enable FIFO and register ISR on enter OPERATIONAL: %s", esp_err_to_name(ret));
            return ret;
        }
        m_isr_handler_installed = true;
        ESP_LOGI(TAG, "FIFO enabled and ISR registered for pin %d", m_config.int_pin);
    } else {
        // Basic FIFO setup without ISR if pin configuration is invalid
        ret = m_fifoProcessor->resetAndReEnableFIFO();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to reset and enable FIFO on enter OPERATIONAL: %s", esp_err_to_name(ret));
            return ret;
        }
        ESP_LOGW(TAG, "INT pin configuration invalid: %d. Using polling mode.", m_config.int_pin);
    }
    
    return ESP_OK;
}

esp_err_t IMUService::exitOperationalState() {
    ESP_LOGI(TAG, "Exiting OPERATIONAL state");
    
    // Use the overloaded method that also unregisters the ISR if necessary
    esp_err_t ret = m_fifoProcessor->disableFIFO(m_isr_handler_installed);
    
    // Mark the ISR as uninstalled if it was installed
    if (m_isr_handler_installed) {
        m_isr_handler_installed = false;
        ESP_LOGI(TAG, "ISR handler marked as uninstalled during exit from OPERATIONAL state");
    }
    
    return ret;
}

esp_err_t IMUService::enterCalibrationState() {
    ESP_LOGI(TAG, "Entering CALIBRATION state");
    
    // Disable FIFO with ISR unregistration if needed
    esp_err_t ret = m_fifoProcessor->disableFIFO(m_isr_handler_installed);
    
    // Update ISR state
    if (m_isr_handler_installed) {
        m_isr_handler_installed = false;
        ESP_LOGI(TAG, "ISR handler marked as uninstalled during enter to CALIBRATION state");
    }
    
    return ret;
}

esp_err_t IMUService::exitCalibrationState() {
    ESP_LOGD(TAG, "Exiting CALIBRATION state.");
    m_is_calibrating_flag.store(false, std::memory_order_release);
    return ESP_OK;
}

esp_err_t IMUService::enterRecoveryState() {
    ESP_LOGI(TAG, "Entering RECOVERY state");
    
    // Disable FIFO with ISR unregistration if needed
    esp_err_t ret = m_fifoProcessor->disableFIFO(m_isr_handler_installed);
    
    // Update ISR state
    if (m_isr_handler_installed) {
        m_isr_handler_installed = false;
        ESP_LOGI(TAG, "ISR handler marked as uninstalled during enter to RECOVERY state");
    }
    
    return ret;
}

esp_err_t IMUService::exitRecoveryState() {
    ESP_LOGD(TAG, "Exiting RECOVERY state.");
    m_recovery_attempt_in_progress.store(false, std::memory_order_relaxed); 
    return ESP_OK;
}

void IMUService::handleCalibrationRequest(const IMU_CalibrationRequest& event) {
    ESP_LOGI(TAG, "Received IMU_CalibrationRequest event");
    
    // Reject calibration if we're in recovery state
    if (m_current_state.load() == IMUState::RECOVERY) {
        ESP_LOGW(TAG, "Cannot calibrate while in RECOVERY state. Rejecting request.");
        m_eventBus.publish(IMU_CalibrationRequestRejected(IMU_CalibrationRequestRejected::Reason::RECOVERY_IN_PROGRESS, true)); 
        return;
    }
    
    // Reject calibration if we're already calibrating
    if (m_is_calibrating_flag.load()) {
        ESP_LOGW(TAG, "Calibration already in progress. Rejecting request.");
        m_eventBus.publish(IMU_CalibrationRequestRejected(IMU_CalibrationRequestRejected::Reason::OTHER, true));   
        return;
    }

    transitionToState(IMUState::CALIBRATION);
    
    // Perform calibration
    esp_err_t result = performCalibration();
    
    // Return to operational state
    transitionToState(IMUState::OPERATIONAL);
    
    // Publish completion event
    m_eventBus.publish(IMU_CalibrationCompleted(result));
}

esp_err_t IMUService::performCalibration() {
    if (m_current_state.load() != IMUState::CALIBRATION) {
        ESP_LOGE(TAG, "Cannot perform calibration while not in CALIBRATION state");
        return ESP_ERR_INVALID_STATE;
    }
    
    m_is_calibrating_flag.store(true, std::memory_order_release);
    ESP_LOGI(TAG, "Starting gyroscope calibration with %d samples", m_config.calibration_samples);
    
    // Rate-limit the calibration by adding a small delay between samples
    auto rate_limited_progress_callback = [this](int progress, int total) {
        ESP_LOGD(TAG, "Calibration progress: %d/%d", progress, total);
        // Add a small delay between samples for all cases
        // This also covers delay needed after errors since IMUCalibration
        // reports progress for successful samples only
        vTaskDelay(pdMS_TO_TICKS(5));
    };
    
    // Perform calibration using our encapsulated calibration logic
    esp_err_t result = m_calibration->calibrate(m_config, rate_limited_progress_callback);
    
    if (result == ESP_OK) {
        ESP_LOGI(TAG, "Calibration completed successfully");
        
        // Update offsets in config
        m_config.gyro_offset_x = m_calibration->getGyroOffsetXDPS();
        m_config.gyro_offset_y = m_calibration->getGyroOffsetYDPS();
        m_config.gyro_offset_z = m_calibration->getGyroOffsetZDPS();
        
        // Publish the new offsets
        m_eventBus.publish(IMU_GyroOffsetsUpdated(
            m_calibration->getGyroOffsetXDPS(),
            m_calibration->getGyroOffsetYDPS(),
            m_calibration->getGyroOffsetZDPS()
        ));
    } else {
        ESP_LOGE(TAG, "Calibration failed: %s", esp_err_to_name(result));
    }
    
    m_is_calibrating_flag.store(false, std::memory_order_release);
    return result;
}

void IMUService::startRecoveryProcess(const IMURecoveryConfig& config) {
    ESP_LOGI(TAG, "Starting IMU recovery process. Config: MinimalInterrupt=%s, MaxAttempts=%d, Delay=%dms",
            config.minimal_interruption ? "true" : "false", config.max_attempts, config.retry_delay_ms);

    // Ensure we're in RECOVERY state before attempting recovery
    if (m_current_state.load() != IMUState::RECOVERY) {
        ESP_LOGW(TAG, "Recovery requested while not in RECOVERY state (current: %s). Transitioning to RECOVERY first.",
                stateToString(m_current_state.load()));
        transitionToState(IMUState::RECOVERY);
        if (m_current_state.load() != IMUState::RECOVERY) {
            ESP_LOGE(TAG, "Failed to transition to RECOVERY state. Cannot attempt recovery.");
            m_eventBus.publish(IMU_RecoveryFailed(ESP_ERR_INVALID_STATE)); 
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }

    // Set recovery in progress flags and timestamp
    m_recovery_attempt_in_progress = true;
    m_recovery_start_time_ms = esp_timer_get_time() / 1000;

    // Execute the actual recovery procedure
    esp_err_t result = attemptRecovery(config); 

    // Clear in-progress flag
    m_recovery_attempt_in_progress = false; 

    // Handle recovery result
    if (result == ESP_OK) {
        ESP_LOGI(TAG, "IMU recovery process succeeded. Transitioning to OPERATIONAL state.");
        m_eventBus.publish(IMU_RecoverySucceeded());
        transitionToState(IMUState::OPERATIONAL); 
    } else {
        ESP_LOGE(TAG, "IMU recovery process failed: %s. IMU remains in RECOVERY state.", esp_err_to_name(result));
        m_eventBus.publish(IMU_RecoveryFailed(result));
    }    
}

esp_err_t IMUService::attemptRecovery(const IMURecoveryConfig& config) {
    const int MAX_RETRIES = (config.max_attempts > 0) ? config.max_attempts : 3; 
    const int RETRY_DELAY_MS = (config.retry_delay_ms > 0 && config.retry_delay_ms < 5000) ? config.retry_delay_ms : 250;
    const bool MINIMAL_INTERRUPTION = config.minimal_interruption;

    ESP_LOGI(TAG, "Attempting IMU recovery (Max Retries=%d, Delay=%dms, Minimal Interrupt=%s)",
             MAX_RETRIES, RETRY_DELAY_MS, MINIMAL_INTERRUPTION ? "Yes" : "No");

    esp_err_t status = ESP_FAIL; 

    for (int attempt = 1; attempt <= MAX_RETRIES; ++attempt) {
        ESP_LOGI(TAG, "IMU Recovery Attempt %d/%d...", attempt, MAX_RETRIES);
        
        ESP_LOGI(TAG,"Recovery: Resetting sensor...");
        status = m_driver->resetSensor(); 
        if (status != ESP_OK) {
            ESP_LOGE(TAG, "Recovery Failed (Attempt %d): Sensor reset error (%s)", attempt, esp_err_to_name(status));
            if (attempt < MAX_RETRIES) vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
            continue; 
        }
        vTaskDelay(pdMS_TO_TICKS(50));

        ESP_LOGI(TAG,"Recovery: Reconfiguring sensor hardware...");
        status = configureSensorHardware(); 
        if (status != ESP_OK) {
            ESP_LOGE(TAG, "Recovery Failed (Attempt %d): Hardware reconfig error (%s)", attempt, esp_err_to_name(status));
            if (attempt < MAX_RETRIES) vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
            continue;
        }
        vTaskDelay(pdMS_TO_TICKS(50)); 

        ESP_LOGI(TAG,"Recovery: Verifying communication (WHO_AM_I)...");
        uint8_t who_am_i_val = 0;
        status = m_driver->getDeviceID(who_am_i_val);
        if (status != ESP_OK) {
            ESP_LOGE(TAG, "Recovery Failed (Attempt %d): WHO_AM_I read error (%s)", attempt, esp_err_to_name(status));
        } else if (who_am_i_val != IMUHealthMonitor::MPU6050_WHO_AM_I_VALUE) {
            ESP_LOGE(TAG, "Recovery Failed (Attempt %d): WHO_AM_I mismatch (Expected 0x%02X, Got 0x%02X)", 
                     attempt, IMUHealthMonitor::MPU6050_WHO_AM_I_VALUE, who_am_i_val);
            status = ESP_ERR_INVALID_RESPONSE; 
        }
        
        if (status != ESP_OK) {
            if (attempt < MAX_RETRIES) vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
            continue;
        }
        ESP_LOGI(TAG, "Recovery (Attempt %d): Communication verified (WHO_AM_I=0x%02X).", attempt, who_am_i_val);

        // Re-apply software gyro offsets to the estimator using current m_config
        // This ensures the estimator has the correct offsets after hardware reconfig.
        ESP_LOGI(TAG, "Recovery: Re-applying gyro offsets to OrientationEstimator from m_config.");
        m_estimator->updateGyroOffsets(m_config.gyro_offset_x, m_config.gyro_offset_y, m_config.gyro_offset_z);
        // Also ensure CalibrationService itself has the correct offsets loaded for its own logic
        m_calibration->setOffsets(m_config.gyro_offset_x, m_config.gyro_offset_y, m_config.gyro_offset_z);

        if (MINIMAL_INTERRUPTION) {
            ESP_LOGI(TAG, "Recovery (Minimal Interruption): Publishing MOTION_UsingFallbackValues(true).");
            m_eventBus.publish(MOTION_UsingFallbackValues(true));
        }

        ESP_LOGI(TAG, "IMU Recovery Attempt %d Succeeded!", attempt);
        m_healthMonitor->pet(); 
        return ESP_OK; 
    }

    ESP_LOGE(TAG, "IMU Recovery Failed after %d attempts (Last Error: %s).", MAX_RETRIES, esp_err_to_name(status));
    return status; 
}