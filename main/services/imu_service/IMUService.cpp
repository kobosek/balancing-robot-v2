// main/services/IMUService.cpp

#include "IMUService.hpp"
#include "mpu6050.hpp"
#include "OrientationEstimator.hpp"
#include "SYSTEM_StateChanged.hpp"
#include "IMU_RecoverySucceeded.hpp"
#include "IMU_RecoveryFailed.hpp"
#include "IMU_AttemptRecovery.hpp"
#include "MOTION_UsingFallbackValues.hpp"
#include "CONFIG_ImuConfigUpdate.hpp" 
#include "IMUCalibrationService.hpp"
#include "IMUHealthMonitor.hpp"
#include "ConfigData.hpp"
#include "EventBus.hpp"
#include "EventTypes.hpp"
#include "IMU_CommunicationError.hpp"
#include "CONFIG_FullConfigUpdate.hpp" 
#include "IMU_CalibrationStarted.hpp"
#include "IMU_CalibrationCompleted.hpp"
#include "IMU_StateChanged.hpp" 
#include "IMU_StateTransitionRequest.hpp"
#include "BaseEvent.hpp"
#include "driver/gpio.h"
#include "esp_check.h"
#include <cmath>
#include <vector>
#include "freertos/task.h" 
#include <algorithm>      

// Constructor Implementation
IMUService::IMUService(MPU6050Driver& driver,
                       IMUCalibrationService& calibrationService,
                       IMUHealthMonitor& healthMonitor,
                       OrientationEstimator& estimator,
                       const MPU6050Config& config, 
                       EventBus& bus) :
    m_driver(driver),
    m_calibrationService(calibrationService),
    m_healthMonitor(healthMonitor),
    m_estimator(estimator),
    m_config(config), 
    m_eventBus(bus),
    m_current_state(IMUState::INITIALIZED),
    m_system_state(SystemState::INIT),
    m_recovery_attempt_in_progress(false),
    m_recovery_start_time_ms(0),
    m_recovery_timeout_ms(5000), 
    m_accel_lsb_per_g(1.0f), 
    m_gyro_lsb_per_dps(1.0f), 
    m_sample_period_s(0.001f), 
    m_isr_data_counter(0),
    m_isr_handler_installed(false) 
{
    // Constructor body (if needed)
}

IMUService::~IMUService() {
    ESP_LOGI(TAG, "Deconstructing IMUService...");
    if (m_isr_handler_installed) { 
        if (m_config.int_pin >= 0 && m_config.int_pin < GPIO_NUM_MAX) {
            esp_err_t ret = gpio_isr_handler_remove(static_cast<gpio_num_t>(m_config.int_pin));
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Removed ISR handler for pin %d", (int)m_config.int_pin);
            } else {
                ESP_LOGE(TAG, "Error removing ISR handler for pin %d: %s", (int)m_config.int_pin, esp_err_to_name(ret));
            }
        }
        m_isr_handler_installed = false; 
    } else {
         ESP_LOGI(TAG, "ISR handler not removed (was not successfully installed or pin invalid).");
    }
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
        m_estimator.init(m_config.comp_filter_alpha, m_sample_period_s,
                         m_config.gyro_offset_x, m_config.gyro_offset_y, m_config.gyro_offset_z);
        ESP_LOGI(TAG, "Re-initialized OrientationEstimator due to config change.");
    } else if (gyroOffsetsChanged) {
        m_estimator.updateGyroOffsets(m_config.gyro_offset_x, m_config.gyro_offset_y, m_config.gyro_offset_z);
        ESP_LOGI(TAG, "Updated OrientationEstimator gyro offsets due to config change.");
    }
    
    // CalibrationService always gets the latest offsets from config directly
    // It's used for the calibration routine itself, not for feeding the estimator anymore.
    m_calibrationService.setOffsets(m_config.gyro_offset_x, m_config.gyro_offset_y, m_config.gyro_offset_z);
    
    if (m_current_state.load(std::memory_order_acquire) == IMUState::CALIBRATION && hardwareConfigChanged) {
        ESP_LOGI(TAG, "Hardware config parameters changed during CALIBRATION. Hardware will be reconfigured upon transitioning to OPERATIONAL.");
    }
}

void IMUService::handleConfigUpdate(const BaseEvent& event) {
    if (event.type != EventType::CONFIG_FULL_UPDATE) return;
    ESP_LOGD(TAG, "Handling general config update event (CONFIG_FullConfigUpdate).");
    const auto& configEvent = static_cast<const CONFIG_FullConfigUpdate&>(event);
    
    MPU6050Config oldImuConfig = m_config; 
    applyConfig(configEvent.configData.imu); 

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
}


esp_err_t IMUService::configureSensorHardware() {
    ESP_LOGI(TAG, "Configuring MPU6050 hardware settings using current m_config...");
    esp_err_t ret;

    ret = m_driver.setPowerManagementReg(MPU6050PowerManagement::CLOCK_INTERNAL); 
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

    ret = m_driver.setAccelRangeReg(accelRegValue); ESP_RETURN_ON_ERROR(ret, TAG, "Failed set Accel Range Reg");
    ret = m_driver.setGyroRangeReg(gyroRegValue);   ESP_RETURN_ON_ERROR(ret, TAG, "Failed set Gyro Range Reg");
    ret = m_driver.setDLPFConfigReg(dlpf);       ESP_RETURN_ON_ERROR(ret, TAG, "Failed set DLPF Reg");
    ret = m_driver.setSampleRateDivReg(rateDiv); ESP_RETURN_ON_ERROR(ret, TAG, "Failed set Sample Rate Div Reg");

    if (m_config.int_pin >= 0 && m_config.int_pin < GPIO_NUM_MAX) {
        MPU6050InterruptPinConfig intPinConfigVal = m_config.interrupt_active_high ?
            MPU6050InterruptPinConfig::ACTIVE_HIGH : MPU6050InterruptPinConfig::ACTIVE_LOW;
        ret = m_driver.configureInterruptPinReg(intPinConfigVal, MPU6050Interrupt::DATA_READY);
        ESP_RETURN_ON_ERROR(ret, TAG, "Failed setup MPU Interrupt Reg");
        ESP_LOGI(TAG,"MPU Interrupt Pin configured for DATA_READY.");
    } else {
        ESP_LOGW(TAG, "Interrupt pin (%d) not configured in MPU6050 hardware (disabling MPU INT).", m_config.int_pin);
         ret = m_driver.configureInterruptPinReg(MPU6050InterruptPinConfig::ACTIVE_HIGH, (MPU6050Interrupt)0); 
         ESP_RETURN_ON_ERROR(ret, TAG, "Failed to disable MPU Interrupt Reg");
    }

    ret = m_driver.configureFIFOReg(MPU6050UserControl::FIFO_RESET, (MPU6050FIFOEnable)0); 
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed reset MPU FIFO");
    vTaskDelay(pdMS_TO_TICKS(2)); 
    ret = m_driver.configureFIFOReg(MPU6050UserControl::SIG_COND_RESET, (MPU6050FIFOEnable)0); 
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed reset MPU Signal Paths");
    vTaskDelay(pdMS_TO_TICKS(2));
    ret = m_driver.writeRegister(MPU6050Register::USER_CTRL, 0); 
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to clear USER_CTRL for FIFO disable");
    ret = m_driver.writeRegister(MPU6050Register::FIFO_EN, 0);   
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to clear FIFO_EN for FIFO disable");


    ESP_LOGI(TAG, "MPU6050 hardware configuration complete.");
    return ESP_OK;
}

esp_err_t IMUService::init() {
    ESP_LOGI(TAG, "Initializing IMUService...");
    esp_err_t ret;
    m_isr_handler_installed = false; 

    // 1. Initialize Driver
    ret = m_driver.init(static_cast<i2c_port_t>(m_config.i2c_port), 
                        static_cast<gpio_num_t>(m_config.sda_pin), 
                        static_cast<gpio_num_t>(m_config.scl_pin), 
                        m_config.device_address, 
                        m_config.i2c_freq_hz);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG,"Failed init MPU6050 driver: %s. IMUService init failed.", esp_err_to_name(ret));
        return ret; 
    }
    ESP_LOGI(TAG,"MPU6050 Driver Initialized.");

    // 2. Calculate Scaling Factors (updates m_sample_period_s)
    calculateScalingFactors();

    // 3. Configure Sensor Hardware
    ret = configureSensorHardware(); 
    if (ret != ESP_OK) {
        ESP_LOGE(TAG,"Failed to configure MPU6050 hardware: %s. IMUService init failed.", esp_err_to_name(ret));
        return ret; 
    }
    ESP_LOGI(TAG,"MPU6050 Hardware Configured.");

    // 4. Initialize Calibration Service
    ret = m_calibrationService.init();
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to initialize Calibration Service");
    // Set offsets in CalibrationService (it uses them for its calibration routine logic)
    // These are also the initial offsets for the estimator.
    m_calibrationService.setOffsets(m_config.gyro_offset_x, m_config.gyro_offset_y, m_config.gyro_offset_z);
    ESP_LOGI(TAG,"Calibration Service Initialized and offsets applied.");

    // 5. Initialize Orientation Estimator
    // Pass initial offsets from config. These might be 0 if not calibrated yet, or loaded values.
    m_estimator.init(m_config.comp_filter_alpha, m_sample_period_s,
                     m_config.gyro_offset_x, m_config.gyro_offset_y, m_config.gyro_offset_z);
    ESP_LOGI(TAG,"Orientation Estimator Initialized.");

    // 6. Setup GPIO ISR
    if (m_config.int_pin >= 0 && m_config.int_pin < GPIO_NUM_MAX) {
        ESP_LOGI(TAG, "Configuring GPIO interrupt for pin %d...", m_config.int_pin);
        gpio_config_t io_conf = {};
        io_conf.pin_bit_mask = (1ULL << m_config.int_pin);
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE; 
        io_conf.pull_down_en = m_config.interrupt_active_high ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE;
        io_conf.intr_type = m_config.interrupt_active_high ? GPIO_INTR_POSEDGE : GPIO_INTR_NEGEDGE;
        ret = gpio_config(&io_conf);
        ESP_RETURN_ON_ERROR(ret, TAG, "Failed config INT GPIO %d", m_config.int_pin);

        ret = gpio_install_isr_service(ESP_INTR_FLAG_LOWMED); 
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
             ESP_LOGE(TAG, "Failed install ISR service: %s", esp_err_to_name(ret));
             return ret;
        } else if (ret == ESP_ERR_INVALID_STATE) {
             ESP_LOGW(TAG,"ISR service already installed. Proceeding.");
        }

        gpio_isr_handler_remove(static_cast<gpio_num_t>(m_config.int_pin)); 

        ret = gpio_isr_handler_add(static_cast<gpio_num_t>(m_config.int_pin), isrHandler, this);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed add ISR handler for pin %d: %s", m_config.int_pin, esp_err_to_name(ret));
            return ret; 
        }
        m_isr_handler_installed = true; 
        ESP_LOGI(TAG, "GPIO interrupt configured and handler added for INT pin %d.", m_config.int_pin);
    } else {
         ESP_LOGW(TAG, "GPIO Interrupt not configured (pin=%d is invalid or disabled).", m_config.int_pin);
    }

    m_isr_data_counter.store(0, std::memory_order_relaxed);
    m_recovery_attempt_in_progress.store(false, std::memory_order_relaxed);

    ESP_LOGI(TAG, "IMUService Core Initialized successfully.");
    transitionToState(IMUState::OPERATIONAL); 
    return ESP_OK;
}

void IMUService::subscribeToEvents(EventBus& bus) {
    ESP_LOGI(TAG, "Subscribing IMUService to events...");
    
    bus.subscribe(EventType::CONFIG_FULL_UPDATE, [this](const BaseEvent& event) {
        this->handleConfigUpdate(event);
    });
    bus.subscribe(EventType::CONFIG_IMU_UPDATE, [this](const BaseEvent& event) {
        this->handleIMUConfigUpdate(static_cast<const CONFIG_ImuConfigUpdate&>(event));
    });
    bus.subscribe(EventType::IMU_COMMUNICATION_ERROR, [this](const BaseEvent& event) {
        this->handleImuCommunicationError(static_cast<const IMU_CommunicationError&>(event));
    });
    bus.subscribe(EventType::IMU_CALIBRATION_STARTED, [this](const BaseEvent& event) {
        this->handleCalibrationStarted(static_cast<const IMU_CalibrationStarted&>(event));
    });
    bus.subscribe(EventType::IMU_CALIBRATION_COMPLETED, [this](const BaseEvent& event) {
        this->handleCalibrationComplete(static_cast<const IMU_CalibrationCompleted&>(event));
    });
    bus.subscribe(EventType::IMU_STATE_TRANSITION_REQUEST, [this](const BaseEvent& event) {
        const auto& req = static_cast<const IMU_StateTransitionRequest&>(event);
        ESP_LOGI(TAG, "Received state transition request to %s", stateToString(req.requestedState));
        this->transitionToState(req.requestedState);
    });
    bus.subscribe(EventType::SYSTEM_STATE_CHANGED, [this](const BaseEvent& event) {
        this->handleSystemStateChanged(event);
    });
    bus.subscribe(EventType::IMU_ATTEMPT_RECOVERY, [this](const BaseEvent& event) {
        this->handleAttemptImuRecovery(static_cast<const IMU_AttemptRecovery&>(event));
    });

    ESP_LOGI(TAG, "IMUService subscribed to: CONFIG_FULL_UPDATE, CONFIG_IMU_UPDATE, IMU_COMM_ERROR, CALIB_STARTED, CALIB_COMPLETE, IMU_STATE_TRANS_REQ, SYS_STATE_CHANGED, IMU_ATTEMPT_RECOVERY.");
}

// --- Core Data Pipeline ---
void IMUService::processDataPipeline() {
    if (m_current_state.load(std::memory_order_relaxed) != IMUState::OPERATIONAL) {
        return;
    }

    uint8_t isr_hint = m_isr_data_counter.exchange(0, std::memory_order_relaxed); 
    if (isr_hint > 0) {
         ESP_LOGV(TAG,"ISR hint received (%d data-ready events), checking FIFO count.", isr_hint);
    }

    uint16_t fifo_count = 0;
    esp_err_t ret = m_driver.readFifoCount(fifo_count);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read FIFO count: %s. Publishing error.", esp_err_to_name(ret));
        m_eventBus.publish(IMU_CommunicationError(ret)); 
        return; 
    }

    bool overflow_flag = false;
    ret = m_driver.isFIFOOverflow(overflow_flag); 
    if (ret != ESP_OK) {
         ESP_LOGW(TAG, "Failed to check FIFO overflow status from interrupt register: %s", esp_err_to_name(ret));
    }
    if (overflow_flag || fifo_count >= MAX_FIFO_BUFFER_SIZE) { 
         ESP_LOGW(TAG, "FIFO overflow detected (Flag: %d, Count: %u). Resetting FIFO.", overflow_flag, fifo_count);
         resetAndReEnableFIFO(); 
         return; 
    }

    if (fifo_count < FIFO_PACKET_SIZE) {
        return; 
    }

    uint16_t num_samples_available = fifo_count / FIFO_PACKET_SIZE;
    uint16_t num_samples_to_read = std::min(num_samples_available, MAX_SAMPLES_PER_PIPELINE_CALL);
    uint16_t bytes_to_read = num_samples_to_read * FIFO_PACKET_SIZE;

    if (bytes_to_read > MAX_FIFO_BUFFER_SIZE) { 
        ESP_LOGW(TAG, "Calculated bytes_to_read (%u) > MAX_FIFO_BUFFER_SIZE (%u). Clamping.",
                 bytes_to_read, MAX_FIFO_BUFFER_SIZE);
        bytes_to_read = MAX_FIFO_BUFFER_SIZE;
        num_samples_to_read = bytes_to_read / FIFO_PACKET_SIZE;
    }

    ESP_LOGV(TAG, "FIFO: %u bytes (%u avail). Reading: %u bytes (%u samples).",
             fifo_count, num_samples_available, bytes_to_read, num_samples_to_read);

    if (bytes_to_read == 0) {
        return;
    }
    ret = m_driver.readFifoBuffer(m_fifo_buffer, bytes_to_read);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read FIFO buffer: %s. Publishing error.", esp_err_to_name(ret));
        m_eventBus.publish(IMU_CommunicationError(ret));
        return; 
    }
    
    if (!validateFIFOData(m_fifo_buffer, bytes_to_read)) {
        ESP_LOGW(TAG, "FIFO data validation failed. Resetting and re-enabling FIFO.");
        resetAndReEnableFIFO();
        return; 
    }

    int processed_sample_count = 0;
    for (uint16_t i = 0; i < num_samples_to_read; ++i) {
        int offset = i * FIFO_PACKET_SIZE;
        if (offset + FIFO_PACKET_SIZE > bytes_to_read) {
            ESP_LOGE(TAG,"Buffer read boundary error (offset=%d, i=%u, bytes_read=%u). Stopping processing for this batch.", offset, i, bytes_to_read);
            break; 
        }

        int16_t ax_raw = (m_fifo_buffer[offset + 0] << 8) | m_fifo_buffer[offset + 1];
        int16_t ay_raw = (m_fifo_buffer[offset + 2] << 8) | m_fifo_buffer[offset + 3];
        int16_t az_raw = (m_fifo_buffer[offset + 4] << 8) | m_fifo_buffer[offset + 5];
        int16_t gx_raw = (m_fifo_buffer[offset + 6] << 8) | m_fifo_buffer[offset + 7];
        int16_t gy_raw = (m_fifo_buffer[offset + 8] << 8) | m_fifo_buffer[offset + 9];
        int16_t gz_raw = (m_fifo_buffer[offset + 10] << 8) | m_fifo_buffer[offset + 11];

        float ax_g = static_cast<float>(ax_raw) / m_accel_lsb_per_g;
        float ay_g = static_cast<float>(ay_raw) / m_accel_lsb_per_g;
        float az_g = static_cast<float>(az_raw) / m_accel_lsb_per_g;
        float gx_dps = static_cast<float>(gx_raw) / m_gyro_lsb_per_dps; // Scaled, but not offset-corrected here
        float gy_dps = static_cast<float>(gy_raw) / m_gyro_lsb_per_dps; // Scaled, but not offset-corrected here
        float gz_dps = static_cast<float>(gz_raw) / m_gyro_lsb_per_dps; // Scaled, but not offset-corrected here

        // Pass scaled (but not offset-corrected) gyro data to estimator. Estimator applies its configured offsets.
        m_estimator.processSample(ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps);
        processed_sample_count++;
    }

    if (processed_sample_count > 0) {
        ESP_LOGV(TAG, "Processed %d samples from FIFO.", processed_sample_count);
        m_healthMonitor.pet(); 
    } else if (bytes_to_read > 0) {
        ESP_LOGW(TAG, "Read %u bytes from FIFO but processed 0 samples. This might indicate an issue.", bytes_to_read);
    }
}

// --- Interrupt Handler ---
void IRAM_ATTR IMUService::isrHandler(void *arg) {
    IMUService *s = static_cast<IMUService *>(arg);
    if (s) {
        s->m_isr_data_counter.fetch_add(1, std::memory_order_relaxed);
    }
}

// --- Recovery Logic ---
void IMUService::handleSystemStateChanged(const BaseEvent& event) {
    if (event.type != EventType::SYSTEM_STATE_CHANGED) return;
    
    const auto& stateEvent = static_cast<const SYSTEM_StateChanged&>(event);
    SystemState oldSystemState = m_system_state.exchange(stateEvent.newState);
    
    ESP_LOGI(TAG, "System state changed from: %d to: %d", static_cast<int>(oldSystemState), static_cast<int>(stateEvent.newState));
    
    if (m_current_state.load() == IMUState::RECOVERY && 
        (stateEvent.newState == SystemState::BALANCING || stateEvent.newState == SystemState::FALLEN)) {
        
        ESP_LOGW(TAG, "System state requires operational IMU (%d) while IMU is in RECOVERY.", 
                 static_cast<int>(stateEvent.newState));
        
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
    
    if (m_current_state.load() != IMUState::RECOVERY) {
        transitionToState(IMUState::RECOVERY);
    }
}

// --- State Machine Helper Methods ---
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
            m_current_state.store(IMUState::RECOVERY, std::memory_order_acq_rel); 
            m_eventBus.publish(IMU_StateChanged(oldState, IMUState::RECOVERY)); 
        }
        return; 
    }
    
    m_current_state.store(newState, std::memory_order_acq_rel);
    
    m_healthMonitor.notifyIMUStateChange(newState);
    m_calibrationService.notifyIMUStateChange(newState);
    m_eventBus.publish(IMU_StateChanged(oldState, newState));
    
    ESP_LOGI(TAG, "Successfully transitioned IMU to state %s", stateToString(newState));
}

esp_err_t IMUService::enterInitializedState() {
    ESP_LOGI(TAG, "Entering INITIALIZED state.");
    return disableFIFO();
}

esp_err_t IMUService::exitInitializedState() {
    ESP_LOGD(TAG, "Exiting INITIALIZED state.");
    return ESP_OK;
}

esp_err_t IMUService::enterOperationalState() {
    ESP_LOGI(TAG, "Entering OPERATIONAL state.");
    esp_err_t config_ret = configureSensorHardware();
    if (config_ret != ESP_OK) {
        ESP_LOGE(TAG, "Hardware reconfiguration failed on entering OPERATIONAL state: %s", esp_err_to_name(config_ret));
        return config_ret; 
    }
    vTaskDelay(pdMS_TO_TICKS(10)); 
    
    ESP_LOGI(TAG, "Resetting and enabling FIFO for OPERATIONAL state.");
    return resetAndReEnableFIFO();
}


esp_err_t IMUService::exitOperationalState() {
    ESP_LOGI(TAG, "Exiting OPERATIONAL state. Disabling FIFO.");
    return disableFIFO(); 
}

esp_err_t IMUService::enterCalibrationState() {
    ESP_LOGI(TAG, "Entering CALIBRATION state. Disabling FIFO.");
    return disableFIFO();
}

esp_err_t IMUService::exitCalibrationState() {
    ESP_LOGD(TAG, "Exiting CALIBRATION state.");
    return ESP_OK;
}

esp_err_t IMUService::enterRecoveryState() {
    ESP_LOGI(TAG, "Entering RECOVERY state. Disabling FIFO.");
    m_recovery_attempt_in_progress.store(false, std::memory_order_relaxed); 
    return disableFIFO(); 
}

esp_err_t IMUService::exitRecoveryState() {
    ESP_LOGD(TAG, "Exiting RECOVERY state.");
    m_recovery_attempt_in_progress.store(false, std::memory_order_relaxed); 
    return ESP_OK;
}


esp_err_t IMUService::resetFIFO() {
    ESP_LOGD(TAG, "Resetting FIFO (USER_CTRL.FIFO_RESET=1)");
    esp_err_t ret = m_driver.configureFIFOReg(MPU6050UserControl::FIFO_RESET, (MPU6050FIFOEnable)0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set FIFO_RESET bit: %s", esp_err_to_name(ret));
    }
    vTaskDelay(pdMS_TO_TICKS(2)); 
    return ret;
}

esp_err_t IMUService::enableFIFO() {
    ESP_LOGD(TAG, "Enabling FIFO (USER_CTRL.FIFO_ENABLE=1, FIFO_EN=ACCEL|GYRO)");
    esp_err_t ret = m_driver.configureFIFOReg(MPU6050UserControl::FIFO_ENABLE, MPU6050FIFOEnable::GYRO_ACCEL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable FIFO: %s", esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t IMUService::disableFIFO() {
    ESP_LOGD(TAG, "Disabling FIFO (USER_CTRL.FIFO_ENABLE=0, FIFO_EN=0) and resetting.");
    esp_err_t ret;
    ret = m_driver.writeRegister(MPU6050Register::USER_CTRL, 0); 
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to clear USER_CTRL for FIFO disable: %s", esp_err_to_name(ret));
    }
    ret = m_driver.writeRegister(MPU6050Register::FIFO_EN, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write 0 to FIFO_EN register: %s", esp_err_to_name(ret));
    }
    esp_err_t reset_ret = resetFIFO();
    return (ret == ESP_OK) ? reset_ret : ret; 
}

esp_err_t IMUService::resetAndReEnableFIFO() {
    ESP_LOGI(TAG, "Performing FIFO Reset and Re-Enable sequence.");
    esp_err_t ret = resetFIFO();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset FIFO during re-enable sequence: %s. Publishing error.", esp_err_to_name(ret));
        m_eventBus.publish(IMU_CommunicationError(ret));
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(5)); 
    
    ret = enableFIFO();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to re-enable FIFO after reset: %s. Publishing error.", esp_err_to_name(ret));
        m_eventBus.publish(IMU_CommunicationError(ret));
        return ret;
    }
    ESP_LOGI(TAG, "FIFO reset and re-enabled successfully.");
    return ESP_OK;
}


bool IMUService::validateFIFOData(const uint8_t* data, size_t length) {
    if (data == nullptr) {
        ESP_LOGE(TAG, "FIFO data validation failed: null data pointer.");
        return false;
    }
    if (length == 0 || length % FIFO_PACKET_SIZE != 0) {
        ESP_LOGE(TAG, "FIFO data validation failed: invalid length %zu (not multiple of %zu).", 
                 length, FIFO_PACKET_SIZE);
        return false;
    }
    
    const int numSamples = length / FIFO_PACKET_SIZE;
    for (int i = 0; i < numSamples; i++) {
        const uint8_t* sample = data + (i * FIFO_PACKET_SIZE);
        int16_t accelX = (sample[0] << 8) | sample[1];
        if ((accelX == 0 || accelX == -1) &&
            (((sample[2] << 8) | sample[3]) == accelX) && 
            (((sample[4] << 8) | sample[5]) == accelX)    
           ) {
            int16_t gyroX = (sample[6] << 8) | sample[7];
            if ((gyroX == 0 || gyroX == -1) &&
                (((sample[8] << 8) | sample[9]) == gyroX) && 
                (((sample[10] << 8) | sample[11]) == gyroX)  
               ) {
                ESP_LOGW(TAG, "FIFO data validation: detected suspicious all-same pattern (0x%04X or 0x0000) in sample %d.", 
                        static_cast<uint16_t>(accelX), i);
                return false; 
            }
        }
    }
    return true; 
}

// Calibration event handlers
void IMUService::handleCalibrationStarted(const IMU_CalibrationStarted& event) {
    ESP_LOGI(TAG, "Handling IMU_CalibrationStarted event.");
    transitionToState(IMUState::CALIBRATION);
}

void IMUService::handleCalibrationComplete(const IMU_CalibrationCompleted& event) {
    ESP_LOGI(TAG, "Handling IMU_CalibrationComplete event (Status: %s).", esp_err_to_name(event.status));
    if (event.status == ESP_OK) {
        // Offsets are saved to config by ConfigurationService listening to IMU_GyroOffsetsUpdated.
        // IMUService will get the new offsets via CONFIG_ImuConfigUpdate -> applyConfig -> estimator.updateGyroOffsets.
        ESP_LOGI(TAG, "Calibration successful. Transitioning to OPERATIONAL.");
        transitionToState(IMUState::OPERATIONAL);
    } else {
        ESP_LOGE(TAG, "Calibration failed. Transitioning to RECOVERY to attempt sensor stabilization.");
        transitionToState(IMUState::RECOVERY);
    }
}

void IMUService::handleAttemptImuRecovery(const IMU_AttemptRecovery& event) {
    ESP_LOGI(TAG, "Handling IMU_AttemptRecovery event. Config: MinimalInterrupt=%s, MaxAttempts=%d, Delay=%dms",
             event.config.minimal_interruption ? "true" : "false", event.config.max_attempts, event.config.retry_delay_ms);
    
    if (m_current_state.load() != IMUState::RECOVERY) {
        ESP_LOGW(TAG, "IMU_AttemptRecovery received while not in RECOVERY state (current: %s). Transitioning to RECOVERY first.",
                 stateToString(m_current_state.load()));
        transitionToState(IMUState::RECOVERY);
        if (m_current_state.load() != IMUState::RECOVERY) {
            ESP_LOGE(TAG, "Failed to transition to RECOVERY state. Cannot attempt recovery.");
            m_eventBus.publish(IMU_RecoveryFailed(ESP_ERR_INVALID_STATE)); 
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
    
    m_recovery_attempt_in_progress = true;
    m_recovery_start_time_ms = esp_timer_get_time() / 1000;
    
    esp_err_t result = attemptRecovery(event.config); 
    
    m_recovery_attempt_in_progress = false; 
    
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
        status = m_driver.resetSensor(); 
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
        status = m_driver.readRegisters(MPU6050Register::WHO_AM_I, &who_am_i_val, 1);
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
        m_estimator.updateGyroOffsets(m_config.gyro_offset_x, m_config.gyro_offset_y, m_config.gyro_offset_z);
        // Also ensure CalibrationService itself has the correct offsets loaded for its own logic
        m_calibrationService.setOffsets(m_config.gyro_offset_x, m_config.gyro_offset_y, m_config.gyro_offset_z);

        if (MINIMAL_INTERRUPTION) {
            ESP_LOGI(TAG, "Recovery (Minimal Interruption): Publishing MOTION_UsingFallbackValues(true).");
            m_eventBus.publish(MOTION_UsingFallbackValues(true));
        }

        ESP_LOGI(TAG, "IMU Recovery Attempt %d Succeeded!", attempt);
        m_healthMonitor.pet(); 
        return ESP_OK; 
    }

    ESP_LOGE(TAG, "IMU Recovery Failed after %d attempts (Last Error: %s).", MAX_RETRIES, esp_err_to_name(status));
    return status; 
}