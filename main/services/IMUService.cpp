#include "IMUService.hpp"
#include "mpu6050.hpp" // Need MPU6050Driver definition
#include "OrientationEstimator.hpp"
#include "IMUCalibrationService.hpp" // Need ICalibrationService definition
#include "IMUHealthMonitor.hpp"      // Need IHealthMonitor definition
#include "ConfigData.hpp"         // Need MPU6050Config definition
#include "EventBus.hpp"
#include "EventTypes.hpp"
#include "ImuRecoveryEvents.hpp" // Need recovery event definitions
#include "IMU_CommunicationErrorEvent.hpp" // For subscription
#include "driver/gpio.h"
#include "esp_check.h"
#include <cmath>
#include <vector>
#include "freertos/task.h" // For vTaskDelay

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
    m_accel_lsb_per_g(1.0f), // Default, calculate in init
    m_gyro_lsb_per_dps(1.0f), // Default, calculate in init
    m_sample_period_s(0.001f), // Default, calculate in init
    m_isr_data_counter(0)
{}

IMUService::~IMUService() {
    ESP_LOGI(TAG, "Deconstructing IMUService...");
    // Remove ISR handler if it was installed
    if (m_config.int_pin >= 0 && m_config.int_pin < GPIO_NUM_MAX) {
        // TODO: Check if handler was actually installed before removing? Requires state tracking.
        gpio_isr_handler_remove(m_config.int_pin);
        ESP_LOGI(TAG, "Removed ISR handler for pin %d", (int)m_config.int_pin);
    } else {
         ESP_LOGI(TAG, "ISR handler not removed (pin %d was invalid).", (int)m_config.int_pin);
    }
    ESP_LOGI(TAG, "IMUService deconstructed.");
}

void IMUService::calculateScalingFactors() {
    // Calculate Accel Scale
    MPU6050AccelConfig accel_range_enum = static_cast<MPU6050AccelConfig>(m_config.accel_range);
    switch(accel_range_enum) {
        case MPU6050AccelConfig::RANGE_2G:  m_accel_lsb_per_g = 16384.0f; break;
        case MPU6050AccelConfig::RANGE_4G:  m_accel_lsb_per_g = 8192.0f;  break;
        case MPU6050AccelConfig::RANGE_8G:  m_accel_lsb_per_g = 4096.0f;  break;
        case MPU6050AccelConfig::RANGE_16G: m_accel_lsb_per_g = 2048.0f;  break;
        default: ESP_LOGW(TAG, "Unknown accel range %d, using default scale!", m_config.accel_range); m_accel_lsb_per_g = 8192.0f; break;
    }
    ESP_LOGI(TAG, "Accel scale set to: %.1f LSB/g", m_accel_lsb_per_g);

    // Calculate Gyro Scale (Matches logic in CalibrationService, keep consistent)
    MPU6050GyroConfig gyro_range_enum = static_cast<MPU6050GyroConfig>(m_config.gyro_range);
    switch(gyro_range_enum) {
        case MPU6050GyroConfig::RANGE_250_DEG:  m_gyro_lsb_per_dps = 131.0f; break;
        case MPU6050GyroConfig::RANGE_500_DEG:  m_gyro_lsb_per_dps = 65.5f;  break;
        case MPU6050GyroConfig::RANGE_1000_DEG: m_gyro_lsb_per_dps = 32.8f;  break;
        case MPU6050GyroConfig::RANGE_2000_DEG: m_gyro_lsb_per_dps = 16.4f;  break;
        default: ESP_LOGW(TAG, "Unknown gyro range %d, using default scale!", m_config.gyro_range); m_gyro_lsb_per_dps = 65.5f; break;
    }
    ESP_LOGI(TAG, "Gyro scale set to: %.1f LSB/dps", m_gyro_lsb_per_dps);

    // Calculate Sample Period
    // Gyro output rate is 1kHz if DLPF is enabled (1-6), 8kHz if disabled (0 or 7)
    bool dlpf_enabled = (m_config.dlpf_config >= 1 && m_config.dlpf_config <= 6);
    float gyro_output_rate_hz = dlpf_enabled ? 1000.0f : 8000.0f;
    m_sample_period_s = 1.0f / (gyro_output_rate_hz / (1.0f + m_config.sample_rate_divisor));
    ESP_LOGI(TAG, "Calculated Sample Period: %.6f s (Gyro Rate: %.0f Hz, Div: %d)",
             m_sample_period_s, gyro_output_rate_hz, m_config.sample_rate_divisor);
}


esp_err_t IMUService::configureSensorHardware() {
    ESP_LOGI(TAG, "Configuring MPU6050 hardware settings...");
    esp_err_t ret;

    // Wake up sensor and set clock source
    ret = m_driver.setPowerManagementReg(MPU6050PowerManagement::CLOCK_PLL_XGYRO);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to set power management/clock source");
    vTaskDelay(pdMS_TO_TICKS(50)); // Allow sensor to stabilize

    // Set Sensor Ranges, DLPF, Sample Rate
    MPU6050AccelConfig accelRange = static_cast<MPU6050AccelConfig>(m_config.accel_range);
    MPU6050GyroConfig gyroRange = static_cast<MPU6050GyroConfig>(m_config.gyro_range);
    MPU6050DLPFConfig dlpf = static_cast<MPU6050DLPFConfig>(m_config.dlpf_config);
    MPU6050SampleRateDiv rateDiv = static_cast<MPU6050SampleRateDiv>(m_config.sample_rate_divisor);

    ret = m_driver.setAccelRangeReg(accelRange); ESP_RETURN_ON_ERROR(ret, TAG, "Failed set Accel Range Reg");
    ret = m_driver.setGyroRangeReg(gyroRange);   ESP_RETURN_ON_ERROR(ret, TAG, "Failed set Gyro Range Reg");
    ret = m_driver.setDLPFConfigReg(dlpf);       ESP_RETURN_ON_ERROR(ret, TAG, "Failed set DLPF Reg");
    ret = m_driver.setSampleRateDivReg(rateDiv); ESP_RETURN_ON_ERROR(ret, TAG, "Failed set Sample Rate Div Reg");

    // Configure Interrupt Pin (if valid pin specified)
    if (m_config.int_pin >= 0 && m_config.int_pin < GPIO_NUM_MAX) {
        MPU6050InterruptPinConfig intPinConfig = m_config.interrupt_active_high ?
            MPU6050InterruptPinConfig::ACTIVE_HIGH : MPU6050InterruptPinConfig::ACTIVE_LOW;
        // Enable DATA_READY interrupt
        ret = m_driver.configureInterruptPinReg(intPinConfig, MPU6050Interrupt::DATA_READY);
        ESP_RETURN_ON_ERROR(ret, TAG, "Failed setup MPU Interrupt Reg");
        ESP_LOGI(TAG,"MPU Interrupt Pin configured via driver.");
    } else {
        ESP_LOGW(TAG, "Interrupt pin (%d) not configured in hardware.", m_config.int_pin);
        // Ensure interrupt is disabled in MPU6050 if pin not used
         ret = m_driver.configureInterruptPinReg(MPU6050InterruptPinConfig::ACTIVE_HIGH, (MPU6050Interrupt)0); // Disable all interrupts
         ESP_RETURN_ON_ERROR(ret, TAG, "Failed disable MPU Interrupt Reg");
    }

    // Configure FIFO (Enable FIFO, Reset, and enable Accel+Gyro data)
    ret = m_driver.configureFIFOReg(MPU6050UserControl::FIFO_RESET_ENABLE, MPU6050FIFOEnable::GYRO_ACCEL);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed setup MPU FIFO Reg");

    ESP_LOGI(TAG, "MPU6050 hardware configuration complete.");
    return ESP_OK;
}

esp_err_t IMUService::init() {
    ESP_LOGI(TAG, "Initializing IMUService...");
    esp_err_t ret;

    // 1. Initialize Driver (I2C Handles)
    // Note: Driver init doesn't configure the sensor yet
    ret = m_driver.init(m_config.i2c_port, m_config.sda_pin, m_config.scl_pin, m_config.device_address, m_config.i2c_freq_hz);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed init MPU6050 driver");

    // 2. Calculate Scaling Factors based on config
    calculateScalingFactors();

    // 3. Configure Sensor Hardware via Driver
    ret = configureSensorHardware();
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to configure MPU6050 hardware");

    // 4. Initialize Calibration Service (performs initial calibration)
    ret = m_calibrationService.init();
    // Log warning but continue if initial calibration fails
    if(ret != ESP_OK) { ESP_LOGW(TAG, "Initial calibration failed in Calibration Service (%s)", esp_err_to_name(ret)); }

    // 5. Initialize Orientation Estimator
    m_estimator.init(m_config.comp_filter_alpha, m_sample_period_s); // Pass params

    // 6. Setup GPIO ISR (if interrupt pin is valid)
    if (m_config.int_pin >= 0 && m_config.int_pin < GPIO_NUM_MAX) {
        gpio_config_t io_conf = {};
        io_conf.pin_bit_mask = (1ULL << m_config.int_pin);
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE; // Often needs pulldown if active high
        io_conf.intr_type = m_config.interrupt_active_high ? GPIO_INTR_POSEDGE : GPIO_INTR_NEGEDGE;
        ret = gpio_config(&io_conf);
        ESP_RETURN_ON_ERROR(ret, TAG, "Failed config INT GPIO %d", m_config.int_pin);

        // Install ISR service if not already installed (potential race condition if multiple services do this)
        // It's better if Application ensures ISR service is installed once globally.
        // Assuming it's handled elsewhere or accepting potential ESP_ERR_INVALID_STATE.
        ret = gpio_install_isr_service(ESP_INTR_FLAG_LOWMED);
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
             ESP_RETURN_ON_ERROR(ret, TAG, "Failed install ISR service");
        } else if (ret == ESP_ERR_INVALID_STATE) {
             ESP_LOGW(TAG,"ISR service already installed.");
             ret = ESP_OK; // Treat as OK if already installed
        }

        ret = gpio_isr_handler_add(m_config.int_pin, isrHandler, this);
        if (ret == ESP_ERR_INVALID_STATE) {
             ESP_LOGW(TAG, "ISR handler already added for pin %d.", m_config.int_pin);
             ret = ESP_OK; // Treat as OK if already added (e.g., during recovery re-init)
        }
        ESP_RETURN_ON_ERROR(ret, TAG, "Failed add ISR handler for pin %d", m_config.int_pin);
        ESP_LOGI(TAG, "GPIO interrupt configured for INT pin %d", m_config.int_pin);
    } else {
         ESP_LOGW(TAG, "GPIO Interrupt not configured (pin=%d)", m_config.int_pin);
    }

    // 8. Reset state variables
    m_isr_data_counter.store(0, std::memory_order_relaxed);
    m_recovery_in_progress.store(false, std::memory_order_relaxed);
    m_imu_recovery_attempts = 0;

    ESP_LOGI(TAG, "IMUService Initialized successfully.");
    return ESP_OK;
}

// <<< ADDED: Event subscription logic >>>
void IMUService::subscribeToEvents(EventBus& bus) {
    // Subscribe to communication errors to trigger recovery
    bus.subscribe(EventType::IMU_COMMUNICATION_ERROR, [this](const BaseEvent& ev){
        this->handleImuCommunicationError(static_cast<const IMU_CommunicationErrorEvent&>(ev));
    });
    ESP_LOGI(TAG, "Subscribed to IMU_COMMUNICATION_ERROR events.");
    // Potentially subscribe to ATTEMPT_IMU_RECOVERY_COMMAND if needed, but handled by error event now
}


// --- Core Data Pipeline ---
void IMUService::processDataPipeline() {
    // Check if calibration or recovery is happening
    if (m_calibrationService.isCalibrating() || m_recovery_in_progress.load(std::memory_order_relaxed)) {
        vTaskDelay(pdMS_TO_TICKS(5)); // Small delay
        return;
    }

    // --- Check FIFO Count ---
    uint8_t isr_hint = m_isr_data_counter.load(std::memory_order_relaxed);
    if (isr_hint > 0) {
         m_isr_data_counter.store(0, std::memory_order_relaxed); // Reset hint
         ESP_LOGV(TAG,"ISR hint received (%d), checking FIFO count.", isr_hint);
    }

    uint16_t fifo_count = 0;
    esp_err_t ret = m_driver.readFifoCount(fifo_count);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read FIFO count: %s. Triggering Health Check.", esp_err_to_name(ret));
        m_healthMonitor.checkHealth(); // Force health check on read failure
        return;
    }

    // --- Check for Overflow ---
    bool overflow = false;
    ret = m_driver.isFIFOOverflow(overflow); // Check interrupt status register
    if (ret != ESP_OK) {
         ESP_LOGW(TAG, "Failed to check FIFO overflow status: %s", esp_err_to_name(ret));
    }
    if (overflow || fifo_count >= MAX_FIFO_BUFFER_SIZE) {
         ESP_LOGW(TAG, "FIFO overflow detected (Flag: %d, Count: %d). Resetting FIFO.", overflow, fifo_count);
         ret = m_driver.configureFIFOReg(MPU6050UserControl::FIFO_RESET, MPU6050FIFOEnable::GYRO_ACCEL); // Reset
         if(ret == ESP_OK) vTaskDelay(pdMS_TO_TICKS(1)); // Short delay after reset
         ret = m_driver.configureFIFOReg(MPU6050UserControl::FIFO_ENABLE, MPU6050FIFOEnable::GYRO_ACCEL); // Re-enable
         if (ret != ESP_OK) ESP_LOGE(TAG,"Failed re-enable FIFO after overflow reset");
         return; // Skip processing this cycle
    }

    // --- Read FIFO Buffer ---
    if (fifo_count < FIFO_PACKET_SIZE) {
        return; // Not enough for a full sample
    }

    uint16_t num_samples_to_read = fifo_count / FIFO_PACKET_SIZE;
    uint16_t bytes_to_read = num_samples_to_read * FIFO_PACKET_SIZE;
    bytes_to_read = std::min(bytes_to_read, (uint16_t)MAX_FIFO_BUFFER_SIZE); // Clamp to buffer size

    ret = m_driver.readFifoBuffer(m_fifo_buffer, bytes_to_read);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read FIFO buffer: %s. Triggering Health Check.", esp_err_to_name(ret));
        m_healthMonitor.checkHealth(); // Force health check
        return;
    }

    // --- Process Samples ---
    int processed_sample_count = 0;
    for (uint16_t i = 0; i < num_samples_to_read; ++i) {
        int offset = i * FIFO_PACKET_SIZE;
        if (offset + FIFO_PACKET_SIZE > bytes_to_read) {
            ESP_LOGW(TAG,"Buffer read error or calculation mismatch, stopping processing.");
            break; // Avoid reading past buffer
        }

        // Parse Raw Data (Big Endian)
        int16_t ax_raw = (m_fifo_buffer[offset + 0] << 8) | m_fifo_buffer[offset + 1];
        int16_t ay_raw = (m_fifo_buffer[offset + 2] << 8) | m_fifo_buffer[offset + 3];
        int16_t az_raw = (m_fifo_buffer[offset + 4] << 8) | m_fifo_buffer[offset + 5];
        int16_t gx_raw = (m_fifo_buffer[offset + 6] << 8) | m_fifo_buffer[offset + 7];
        int16_t gy_raw = (m_fifo_buffer[offset + 8] << 8) | m_fifo_buffer[offset + 9];
        int16_t gz_raw = (m_fifo_buffer[offset + 10] << 8) | m_fifo_buffer[offset + 11];

        // Scale to Physical Units (g's and dps)
        float ax_g = static_cast<float>(ax_raw) / m_accel_lsb_per_g;
        float ay_g = static_cast<float>(ay_raw) / m_accel_lsb_per_g;
        float az_g = static_cast<float>(az_raw) / m_accel_lsb_per_g;
        float gx_dps = static_cast<float>(gx_raw) / m_gyro_lsb_per_dps;
        float gy_dps = static_cast<float>(gy_raw) / m_gyro_lsb_per_dps;
        float gz_dps = static_cast<float>(gz_raw) / m_gyro_lsb_per_dps;

        // Apply Offsets (from Calibration Service)
        float gx_dps_offset = gx_dps - m_calibrationService.getGyroOffsetXDPS();
        float gy_dps_offset = gy_dps - m_calibrationService.getGyroOffsetYDPS();
        float gz_dps_offset = gz_dps - m_calibrationService.getGyroOffsetZDPS();

        // Pass Processed Data to Estimator
        m_estimator.processSample(ax_g, ay_g, az_g, gx_dps_offset, gy_dps_offset, gz_dps_offset);
        processed_sample_count++;
    }

    // Pet the watchdog if we processed at least one sample
    if (processed_sample_count > 0) {
        ESP_LOGV(TAG, "Processed %d samples from FIFO.", processed_sample_count);
        m_healthMonitor.pet();
    } else {
         ESP_LOGW(TAG, "Read %d bytes from FIFO but processed 0 samples.", bytes_to_read);
    }
}


// --- Interrupt Handler ---
void IRAM_ATTR IMUService::isrHandler(void *arg) {
    IMUService *s = static_cast<IMUService *>(arg);
    if (s) {
        // Just increment counter, processing logic is in the task
        s->m_isr_data_counter.fetch_add(1, std::memory_order_relaxed);
    }
}

// --- Recovery Logic ---
void IMUService::handleImuCommunicationError(const IMU_CommunicationErrorEvent& event) {
    // Prevent concurrent recovery attempts
    bool expected = false;
    if (!m_recovery_in_progress.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
        ESP_LOGW(TAG, "Recovery already in progress, ignoring communication error event.");
        return;
    }

    ESP_LOGE(TAG, "IMU Communication Error Event received (Code: %s). Starting recovery process.", esp_err_to_name(event.errorCode));
    m_imu_recovery_attempts = 0; // Reset attempts for this recovery cycle
    attemptRecovery(); // Start the first attempt

    // Recovery flag will be reset by attemptRecovery on success/failure/retry logic
}


esp_err_t IMUService::attemptRecovery() {
    m_imu_recovery_attempts++;
    ESP_LOGI(TAG, "Attempting IMU recovery (Attempt %d/%d)...", m_imu_recovery_attempts, MAX_IMU_RECOVERY_ATTEMPTS);

    esp_err_t status = ESP_OK; // Start with OK status

    // --- Step 1: Reset Sensor ---
    if (status == ESP_OK) {
        ESP_LOGI(TAG,"Recovery Step 1: Resetting sensor...");
        status = m_driver.resetSensor();
        if (status != ESP_OK) {
            ESP_LOGE(TAG, "Recovery Step 1 Failed: Sensor reset error (%s)", esp_err_to_name(status));
        } else {
            vTaskDelay(pdMS_TO_TICKS(100)); // Delay after successful reset
        }
    }

    // --- Step 2: Reconfigure Sensor Hardware ---
    if (status == ESP_OK) {
        ESP_LOGI(TAG,"Recovery Step 2: Reconfiguring sensor hardware...");
        status = configureSensorHardware();
        if (status != ESP_OK) {
            ESP_LOGE(TAG, "Recovery Step 2 Failed: Hardware reconfiguration error (%s)", esp_err_to_name(status));
        } else {
            vTaskDelay(pdMS_TO_TICKS(50)); // Delay after successful config
        }
    }

    // --- Step 3: Verify Communication ---
    if (status == ESP_OK) {
        ESP_LOGI(TAG,"Recovery Step 3: Verifying communication (WHO_AM_I)...");
        uint8_t who = 0;
        esp_err_t verify_ret = m_driver.readRegisters(MPU6050Register::WHO_AM_I, &who, 1);
        if (verify_ret != ESP_OK || who != IMUHealthMonitor::MPU6050_WHO_AM_I_VALUE) {
            ESP_LOGE(TAG, "Recovery Step 3 Failed: Communication verification error (ret=%s, WHO_AM_I=0x%02X)",
                     esp_err_to_name(verify_ret), who);
            status = (verify_ret == ESP_OK) ? ESP_FAIL : verify_ret; // Assign failure status
        } else {
            ESP_LOGI(TAG, "Recovery Step 3 Success: Communication verified.");
        }
    }

    // --- Step 4: Optional Recalibrate ---
    // Skip recalibration for now, as decided previously.
    if (status == ESP_OK) {
        ESP_LOGI(TAG, "Recovery Step 4: Skipping recalibration, using existing offsets.");
    }

    // --- Final Outcome ---
    if (status == ESP_OK) {
        // Success
        ESP_LOGI(TAG, "IMU Recovery Attempt %d Succeeded!", m_imu_recovery_attempts);
        m_eventBus.publish(ImuRecoverySucceededEvent());
        m_recovery_in_progress.store(false, std::memory_order_release); // Reset flag on success
        m_healthMonitor.pet(); // Pet watchdog immediately
        return ESP_OK;
    } else {
        // Failure
        ESP_LOGE(TAG, "IMU Recovery Attempt %d Failed (Error: %s).", m_imu_recovery_attempts, esp_err_to_name(status));
        if (m_imu_recovery_attempts >= MAX_IMU_RECOVERY_ATTEMPTS) {
            ESP_LOGE(TAG, "Maximum IMU recovery attempts reached. Publishing final failure.");
            m_eventBus.publish(ImuRecoveryFailedEvent(status)); // Publish final failure with specific error
            m_recovery_in_progress.store(false, std::memory_order_release); // Reset flag
            // StateManager will handle transition to FATAL_ERROR if needed based on the event
        } else {
            ESP_LOGI(TAG, "Will allow health monitor to trigger next recovery attempt.");
            // Reset the flag so the next error event can trigger another attempt
            m_recovery_in_progress.store(false, std::memory_order_release);
        }
        return status; // Return the specific error code that caused the failure
    }
}