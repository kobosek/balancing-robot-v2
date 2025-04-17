// main/services/IMUService.cpp

#include "IMUService.hpp"
#include "mpu6050.hpp"
#include "OrientationEstimator.hpp"
#include "IMUCalibrationService.hpp"
#include "IMUHealthMonitor.hpp"
#include "ConfigData.hpp"
#include "EventBus.hpp"
#include "EventTypes.hpp"
#include "ImuRecoveryEvents.hpp"
#include "IMU_CommunicationErrorEvent.hpp"
#include "ConfigUpdatedEvent.hpp" // Need event definition
#include "BaseEvent.hpp"
#include "driver/gpio.h"
#include "esp_check.h"
#include <cmath>
#include <vector>
#include "freertos/task.h" // For vTaskDelay
#include <algorithm>      // For std::min

// Constructor Implementation
IMUService::IMUService(MPU6050Driver& driver,
                       IMUCalibrationService& calibrationService,
                       IMUHealthMonitor& healthMonitor,
                       OrientationEstimator& estimator,
                       const MPU6050Config& config, // Take initial config
                       EventBus& bus) :
    m_driver(driver),
    m_calibrationService(calibrationService),
    m_healthMonitor(healthMonitor),
    m_estimator(estimator),
    m_config(config), // Store initial config copy
    m_eventBus(bus),
    m_accel_lsb_per_g(1.0f), // Default, calculate in init
    m_gyro_lsb_per_dps(1.0f), // Default, calculate in init
    m_sample_period_s(0.001f), // Default, calculate in init
    m_isr_data_counter(0),
    m_recovery_in_progress(false), // Initialize recovery flag
    m_isr_handler_installed(false) // Initialize ISR installed flag
{
    // Constructor body (if needed)
}

IMUService::~IMUService() {
    ESP_LOGI(TAG, "Deconstructing IMUService...");
    if (m_isr_handler_installed) { // <-- Check flag
        if (m_config.int_pin >= 0 && m_config.int_pin < GPIO_NUM_MAX) {
            esp_err_t ret = gpio_isr_handler_remove(m_config.int_pin);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Removed ISR handler for pin %d", (int)m_config.int_pin);
            } else {
                // Log error, but don't crash. Maybe the service wasn't running or ISR was already removed elsewhere.
                ESP_LOGE(TAG, "Error removing ISR handler for pin %d: %s", (int)m_config.int_pin, esp_err_to_name(ret));
            }
        }
        m_isr_handler_installed = false; // Mark as removed attempt regardless of success
    } else {
         ESP_LOGI(TAG, "ISR handler not removed (was not successfully installed or pin invalid).");
    }
    // Note: We don't uninstall the ISR *service* here, as it might be shared.
    // This should ideally be handled at the Application level if it's certain no other component uses it.
    ESP_LOGI(TAG, "IMUService deconstructed.");
}


// Apply config values from a struct
void IMUService::applyConfig(const MPU6050Config& newConfig) {
    // <<< DOC COMMENT >>>
    // NOTE on Runtime Configuration Updates:
    // - Software parameters like the complementary filter alpha (`comp_filter_alpha`)
    //   and software gyro offsets (`gyro_offset_x/y/z`) are applied live.
    // - Changes to hardware settings (e.g., `accel_range`, `gyro_range`, `dlpf_config`,
    //   `sample_rate_divisor`, `i2c_*`, pins) are NOT applied live by this function.
    //   Applying these would require stopping data acquisition, reconfiguring hardware,
    //   and potentially resetting the FIFO. Currently, such changes require either
    //   a system restart or rely on the IMU recovery mechanism (if a communication
    //   error occurs) to re-apply the hardware configuration.
    // <<< END DOC COMMENT >>>

    bool needsEstimatorReinit = false;
    bool needsHardwareReconfig = false; // Not doing full reconfig on update yet
    bool needsScalingFactorRecalc = false;

    // Check which parameters changed
    if (m_config.comp_filter_alpha != newConfig.comp_filter_alpha ||
        m_config.sample_rate_divisor != newConfig.sample_rate_divisor) { // Sample rate affects estimator period
        needsEstimatorReinit = true;
    }
    if (m_config.accel_range != newConfig.accel_range ||
        m_config.gyro_range != newConfig.gyro_range ||
        m_config.sample_rate_divisor != newConfig.sample_rate_divisor || // Check if sample rate divisor changed
        m_config.dlpf_config != newConfig.dlpf_config) {
        needsScalingFactorRecalc = true;
    }

    // Update the stored config copy
    m_config = newConfig;
    ESP_LOGI(TAG, "Applied new IMU config.");

    // Recalculate/Reinitialize based on changes
    if (needsScalingFactorRecalc || needsEstimatorReinit) { // Recalc factors if needed for estimator period
        calculateScalingFactors();
    }
    if (needsEstimatorReinit) {
        m_estimator.init(m_config.comp_filter_alpha, m_sample_period_s);
        ESP_LOGI(TAG, "Re-initialized OrientationEstimator due to config change.");
    }

    // Apply gyro offsets from the new config to the calibration service
    m_calibrationService.setOffsets(m_config.gyro_offset_x, m_config.gyro_offset_y, m_config.gyro_offset_z);

    // Note: Hardware reconfiguration (ranges, DLPF etc.) during runtime is complex
    // It would require stopping data acquisition, reconfiguring, potentially resetting FIFO.
    // For now, assume these require a restart or are handled by recovery.
}

// Handle config update event
void IMUService::handleConfigUpdate(const BaseEvent& event) {
    if (event.type != EventType::CONFIG_UPDATED) return;
    ESP_LOGD(TAG, "Handling config update event.");
    const auto& configEvent = static_cast<const ConfigUpdatedEvent&>(event);
    applyConfig(configEvent.configData.imu); // Apply the IMU part of the config
}


void IMUService::calculateScalingFactors() {
    // Calculate Accel Scale (using m_config)
    switch(m_config.accel_range) {
        case 0: m_accel_lsb_per_g = 16384.0f; break;
        case 1: m_accel_lsb_per_g = 8192.0f;  break;
        case 2: m_accel_lsb_per_g = 4096.0f;  break;
        case 3: m_accel_lsb_per_g = 2048.0f;  break;
        default: ESP_LOGW(TAG, "Unknown accel range %d, using default scale!", m_config.accel_range); m_accel_lsb_per_g = 8192.0f; break;
    }
    ESP_LOGD(TAG, "Accel scale set to: %.1f LSB/g", m_accel_lsb_per_g); // Use DEBUG level for potentially frequent logs

    // Calculate Gyro Scale (using m_config)
    switch(m_config.gyro_range) {
        case 0: m_gyro_lsb_per_dps = 131.0f; break;
        case 1: m_gyro_lsb_per_dps = 65.5f;  break;
        case 2: m_gyro_lsb_per_dps = 32.8f;  break;
        case 3: m_gyro_lsb_per_dps = 16.4f;  break;
        default: ESP_LOGW(TAG, "Unknown gyro range %d, using default scale!", m_config.gyro_range); m_gyro_lsb_per_dps = 65.5f; break;
    }
    ESP_LOGD(TAG, "Gyro scale set to: %.1f LSB/dps", m_gyro_lsb_per_dps); // Use DEBUG level

    // Calculate Sample Period (using m_config)
    bool dlpf_enabled = (m_config.dlpf_config >= 1 && m_config.dlpf_config <= 6);
    float gyro_output_rate_hz = dlpf_enabled ? 1000.0f : 8000.0f;
    if ((1.0f + m_config.sample_rate_divisor) <= 0) { // Avoid division by zero
        ESP_LOGE(TAG, "Invalid sample rate divisor (%d), cannot calculate sample period!", m_config.sample_rate_divisor);
        m_sample_period_s = 0.001f; // Default to 1ms
    } else {
        m_sample_period_s = 1.0f / (gyro_output_rate_hz / (1.0f + m_config.sample_rate_divisor));
    }
    ESP_LOGI(TAG, "Calculated Sample Period: %.6f s (Gyro Rate: %.0f Hz, Div: %d)",
             m_sample_period_s, gyro_output_rate_hz, m_config.sample_rate_divisor);
}


esp_err_t IMUService::configureSensorHardware() {
    ESP_LOGI(TAG, "Configuring MPU6050 hardware settings...");
    esp_err_t ret;

    // Wake up sensor and set clock source
    ret = m_driver.setPowerManagementReg(MPU6050PowerManagement::CLOCK_INTERNAL);
    // --- START FIX for ESP_ERR_INVALID_STATE ---
    if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "I2C driver state invalid during PWR_MGMT write. Potential init issue.");
        // Attempt a small delay and retry once, maybe the bus needs time?
        vTaskDelay(pdMS_TO_TICKS(5));
        ret = m_driver.setPowerManagementReg(MPU6050PowerManagement::CLOCK_INTERNAL);
        if (ret != ESP_OK) {
             ESP_LOGE(TAG, "Retry setting PWR_MGMT failed: %s", esp_err_to_name(ret));
             ESP_RETURN_ON_ERROR(ret, TAG, "Failed to set power management/clock source after retry");
        } else {
             ESP_LOGW(TAG, "Setting PWR_MGMT succeeded after retry.");
        }
    } else {
         ESP_RETURN_ON_ERROR(ret, TAG, "Failed to set power management/clock source");
    }
    // --- END FIX ---
    vTaskDelay(pdMS_TO_TICKS(50)); // Allow sensor to stabilize

    MPU6050AccelConfig accelRegValue;
    switch(m_config.accel_range) {
        case 0: accelRegValue = MPU6050AccelConfig::RANGE_2G; break;
        case 1: accelRegValue = MPU6050AccelConfig::RANGE_4G; break;
        case 2: accelRegValue = MPU6050AccelConfig::RANGE_8G; break;
        case 3: accelRegValue = MPU6050AccelConfig::RANGE_16G; break;
        default:
            ESP_LOGW(TAG, "Invalid accel_range %d, defaulting to 4G for hardware config!", m_config.accel_range);
            accelRegValue = MPU6050AccelConfig::RANGE_4G; break;
    }

    MPU6050GyroConfig gyroRegValue;
    switch(m_config.gyro_range) {
        case 0: gyroRegValue = MPU6050GyroConfig::RANGE_250_DEG; break;
        case 1: gyroRegValue = MPU6050GyroConfig::RANGE_500_DEG; break;
        case 2: gyroRegValue = MPU6050GyroConfig::RANGE_1000_DEG; break;
        case 3: gyroRegValue = MPU6050GyroConfig::RANGE_2000_DEG; break;
        default:
             ESP_LOGW(TAG, "Invalid gyro_range %d, defaulting to 500DPS for hardware config!", m_config.gyro_range);
             gyroRegValue = MPU6050GyroConfig::RANGE_500_DEG; break;
    }

    MPU6050DLPFConfig dlpf = static_cast<MPU6050DLPFConfig>(m_config.dlpf_config);
    MPU6050SampleRateDiv rateDiv = static_cast<MPU6050SampleRateDiv>(m_config.sample_rate_divisor);

    ret = m_driver.setAccelRangeReg(accelRegValue); ESP_RETURN_ON_ERROR(ret, TAG, "Failed set Accel Range Reg");
    ret = m_driver.setGyroRangeReg(gyroRegValue);   ESP_RETURN_ON_ERROR(ret, TAG, "Failed set Gyro Range Reg");
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
    // Do Reset first, then Enable
    ret = m_driver.configureFIFOReg(MPU6050UserControl::FIFO_RESET, (MPU6050FIFOEnable)0); // Reset FIFO (FIFO_EN doesn't matter here)
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed reset MPU FIFO");
    vTaskDelay(pdMS_TO_TICKS(1)); // Short delay after reset
    ret = m_driver.configureFIFOReg(MPU6050UserControl::FIFO_ENABLE, MPU6050FIFOEnable::GYRO_ACCEL); // Enable FIFO and set data sources
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed enable MPU FIFO Reg");

    ESP_LOGI(TAG, "MPU6050 hardware configuration complete.");
    return ESP_OK;
}

esp_err_t IMUService::init() {
    ESP_LOGI(TAG, "Initializing IMUService...");
    esp_err_t ret;
    m_isr_handler_installed = false; // Ensure flag starts false

    // 1. Initialize Driver (I2C Handles)
    ret = m_driver.init(m_config.i2c_port, m_config.sda_pin, m_config.scl_pin, m_config.device_address, m_config.i2c_freq_hz);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG,"Failed init MPU6050 driver: %s", esp_err_to_name(ret));
        return ret; // Exit early if driver init fails
    }
    ESP_LOGI(TAG,"MPU6050 Driver Initialized.");


    // 2. Calculate Scaling Factors based on initial config
    calculateScalingFactors();

    // 3. Configure Sensor Hardware via Driver using initial config
    ret = configureSensorHardware();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG,"Failed to configure MPU6050 hardware: %s", esp_err_to_name(ret));
        // Note: Driver's destructor will handle I2C cleanup if we return here
        return ret; // Exit early if hardware config fails
    }
     ESP_LOGI(TAG,"MPU6050 Hardware Configured.");

    // 4. Initialize Calibration Service (Does NOT calibrate initially anymore)
    ret = m_calibrationService.init();
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to initialize Calibration Service");

    // 4b. Apply persistent gyro offsets from initial config to calibration service
    m_calibrationService.setOffsets(m_config.gyro_offset_x, m_config.gyro_offset_y, m_config.gyro_offset_z);

    // 5. Initialize Orientation Estimator using initial config
    m_estimator.init(m_config.comp_filter_alpha, m_sample_period_s);
     ESP_LOGI(TAG,"Orientation Estimator Initialized.");

    // 6. Setup GPIO ISR (if interrupt pin is valid)
    if (m_config.int_pin >= 0 && m_config.int_pin < GPIO_NUM_MAX) {
        ESP_LOGI(TAG, "Configuring GPIO interrupt for pin %d...", m_config.int_pin);
        gpio_config_t io_conf = {};
        io_conf.pin_bit_mask = (1ULL << m_config.int_pin);
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        // Use pull-down if interrupt is active-high, no pull if active-low (assuming external pull-up)
        io_conf.pull_down_en = m_config.interrupt_active_high ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE;
        io_conf.intr_type = m_config.interrupt_active_high ? GPIO_INTR_POSEDGE : GPIO_INTR_NEGEDGE;
        ret = gpio_config(&io_conf);
        if (ret != ESP_OK) { ESP_LOGE(TAG, "Failed config INT GPIO %d: %s", m_config.int_pin, esp_err_to_name(ret)); return ret;}


        // Install ISR service if not already installed
        // Use ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LOWMED or similar suitable flags
        ret = gpio_install_isr_service(ESP_INTR_FLAG_LOWMED);
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
             ESP_LOGE(TAG, "Failed install ISR service: %s", esp_err_to_name(ret));
             return ret; // Critical failure if ISR service cannot be installed
        } else if (ret == ESP_ERR_INVALID_STATE) {
             ESP_LOGW(TAG,"ISR service already installed.");
             // Continue, assuming it's usable
        }

        // Remove existing handler first, just in case (e.g., re-init after recovery)
        gpio_isr_handler_remove(m_config.int_pin); // Ignore error if not found

        ret = gpio_isr_handler_add(m_config.int_pin, isrHandler, this);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed add ISR handler for pin %d: %s", m_config.int_pin, esp_err_to_name(ret));
            return ret; // Critical failure if handler cannot be added
        }

        m_isr_handler_installed = true; // Set flag ONLY on success
        ESP_LOGI(TAG, "GPIO interrupt configured and handler added for INT pin %d", m_config.int_pin);
    } else {
         ESP_LOGW(TAG, "GPIO Interrupt not configured (pin=%d)", m_config.int_pin);
    }

    // 8. Reset state variables
    m_isr_data_counter.store(0, std::memory_order_relaxed);
    m_recovery_in_progress.store(false, std::memory_order_relaxed);
    // m_imu_recovery_attempts removed

    ESP_LOGI(TAG, "IMUService Initialized successfully.");
    return ESP_OK;
}

// Subscribe to events
void IMUService::subscribeToEvents(EventBus& bus) {
    // Subscribe to attempt recovery commands from StateManager
    bus.subscribe(EventType::ATTEMPT_IMU_RECOVERY_COMMAND, [this](const BaseEvent& ev){
        if (!m_recovery_in_progress.load(std::memory_order_relaxed)) {
             bool expected = false;
             // Attempt to set the flag to true atomically
             if (m_recovery_in_progress.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
                  ESP_LOGI(TAG, "Received ATTEMPT_IMU_RECOVERY_COMMAND. Starting recovery attempt.");
                  // It's generally better to trigger complex actions like recovery from a task context
                  // rather than directly within the event handler/callback.
                  // However, since recovery involves delays and I2C ops, doing it directly here
                  // might block the event bus publisher if not handled carefully.
                  // For now, calling directly, assuming recovery itself doesn't deadlock.
                  attemptRecovery(); // Start the attempt (which now has internal retries)
             } else {
                  ESP_LOGW(TAG, "Received ATTEMPT_IMU_RECOVERY_COMMAND, but recovery flag was already set (compare_exchange failed).");
             }
        } else {
             ESP_LOGW(TAG, "Received ATTEMPT_IMU_RECOVERY_COMMAND, but recovery is already in progress (flag check).");
        }
    });
    // Subscribe to config updates
    bus.subscribe(EventType::CONFIG_UPDATED, [this](const BaseEvent& ev){
        this->handleConfigUpdate(ev);
    });
    ESP_LOGI(TAG, "Subscribed to ATTEMPT_IMU_RECOVERY_COMMAND and CONFIG_UPDATED events.");
}


// --- Core Data Pipeline ---
void IMUService::processDataPipeline() {
    // Check if calibration or recovery is happening
    if (m_calibrationService.isCalibrating() || m_recovery_in_progress.load(std::memory_order_relaxed)) {
        // Do minimal work while calibrating/recovering
        // Avoid flooding logs, maybe check less often?
        // vTaskDelay(pdMS_TO_TICKS(5));
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
        ESP_LOGE(TAG, "Failed to read FIFO count: %s. Publishing error event.", esp_err_to_name(ret));
        m_eventBus.publish(IMU_CommunicationErrorEvent(ret));
        return; // Stop processing this cycle
    }

    // --- Check for Overflow ---
    bool overflow = false;
    ret = m_driver.isFIFOOverflow(overflow); // Check interrupt status register
    if (ret != ESP_OK) {
         ESP_LOGW(TAG, "Failed to check FIFO overflow status: %s", esp_err_to_name(ret));
         // Continue processing data if possible, but log the warning
    }
    if (overflow || fifo_count >= MAX_FIFO_BUFFER_SIZE) { // Check against our buffer size too
         ESP_LOGW(TAG, "FIFO overflow detected (Flag: %d, Count: %d). Resetting FIFO.", overflow, fifo_count);
         ret = m_driver.configureFIFOReg(MPU6050UserControl::FIFO_RESET, (MPU6050FIFOEnable)0); // Reset
         if(ret == ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(1)); // Short delay after reset
            ret = m_driver.configureFIFOReg(MPU6050UserControl::FIFO_ENABLE, MPU6050FIFOEnable::GYRO_ACCEL); // Re-enable
            if (ret != ESP_OK) {
                ESP_LOGE(TAG,"Failed re-enable FIFO after overflow reset. Publishing error.");
                m_eventBus.publish(IMU_CommunicationErrorEvent(ret)); // Publish error if re-enable fails
            }
         } else {
             ESP_LOGE(TAG,"Failed reset FIFO after overflow. Publishing error.");
             m_eventBus.publish(IMU_CommunicationErrorEvent(ret)); // Publish error if reset fails
         }
         return; // Skip processing this cycle
    }

    // --- Check if enough data for at least one sample ---
    if (fifo_count < FIFO_PACKET_SIZE) {
        // ESP_LOGV(TAG, "FIFO count (%d) less than packet size (%d). Skipping read.", fifo_count, FIFO_PACKET_SIZE);
        return; // Not enough data yet
    }

    // --- Determine number of samples to read (respecting limit) ---
    uint16_t num_samples_available = fifo_count / FIFO_PACKET_SIZE;
    // <<< Limit the number of samples processed per call >>>
    uint16_t num_samples_to_read = std::min(num_samples_available, MAX_SAMPLES_PER_PIPELINE_CALL);
    uint16_t bytes_to_read = num_samples_to_read * FIFO_PACKET_SIZE;

    // --- Clamp read size to our buffer size (safety check) ---
    if (bytes_to_read > MAX_FIFO_BUFFER_SIZE) {
        ESP_LOGW(TAG, "Calculated bytes_to_read (%d) exceeds MAX_FIFO_BUFFER_SIZE (%d). Clamping.",
                 bytes_to_read, MAX_FIFO_BUFFER_SIZE);
        bytes_to_read = MAX_FIFO_BUFFER_SIZE;
        // Recalculate num_samples_to_read based on clamped bytes
        num_samples_to_read = bytes_to_read / FIFO_PACKET_SIZE;
    }

    ESP_LOGV(TAG, "FIFO Count: %d bytes (%d samples avail). Reading %d bytes (%d samples).",
             fifo_count, num_samples_available, bytes_to_read, num_samples_to_read);


    // --- Read FIFO Buffer ---
    if (bytes_to_read == 0) {
        // This case should only happen if fifo_count was < FIFO_PACKET_SIZE initially
        ESP_LOGV(TAG, "No bytes to read from FIFO this cycle.");
        return;
    }
    ret = m_driver.readFifoBuffer(m_fifo_buffer, bytes_to_read);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read FIFO buffer: %s. Publishing error event.", esp_err_to_name(ret));
        m_eventBus.publish(IMU_CommunicationErrorEvent(ret));
        return; // Stop processing
    }

    // --- Process Samples ---
    int processed_sample_count = 0;
    for (uint16_t i = 0; i < num_samples_to_read; ++i) {
        int offset = i * FIFO_PACKET_SIZE;
        if (offset + FIFO_PACKET_SIZE > bytes_to_read) {
            ESP_LOGW(TAG,"Buffer read error or calculation mismatch (offset=%d, i=%d, bytes_read=%d). Stopping processing.", offset, i, bytes_to_read);
            break; // Avoid reading past buffer
        }

        // Parse Raw Data (Big Endian)
        int16_t ax_raw = (int16_t)((m_fifo_buffer[offset + 0] << 8) | m_fifo_buffer[offset + 1]);
        int16_t ay_raw = (int16_t)((m_fifo_buffer[offset + 2] << 8) | m_fifo_buffer[offset + 3]);
        int16_t az_raw = (int16_t)((m_fifo_buffer[offset + 4] << 8) | m_fifo_buffer[offset + 5]);
        int16_t gx_raw = (int16_t)((m_fifo_buffer[offset + 6] << 8) | m_fifo_buffer[offset + 7]);
        int16_t gy_raw = (int16_t)((m_fifo_buffer[offset + 8] << 8) | m_fifo_buffer[offset + 9]);
        int16_t gz_raw = (int16_t)((m_fifo_buffer[offset + 10] << 8) | m_fifo_buffer[offset + 11]);

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

    // --- Pet the watchdog if we processed at least one sample ---
    if (processed_sample_count > 0) {
        ESP_LOGV(TAG, "Processed %d samples from FIFO.", processed_sample_count);
        m_healthMonitor.pet();
    } else if (bytes_to_read > 0) {
        // This case might happen if num_samples_to_read was 0 due to clamping,
        // or if the loop broke early.
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
    // This handler is now primarily for logging, recovery is triggered by ATTEMPT_IMU_RECOVERY_COMMAND
    // StateManager decides when to send that command based on this event and overall state.
    ESP_LOGE(TAG, "IMU Communication Error Event received (Code: %s). StateManager should initiate recovery if needed.", esp_err_to_name(event.errorCode));

    // We might still want to set the recovery_in_progress flag here if StateManager immediately
    // sends the recovery command upon receiving this error event. Let's assume StateManager does.
    // Setting the flag prevents concurrent recovery attempts triggered by rapid-fire errors.
    bool expected = false;
    if (!m_recovery_in_progress.load(std::memory_order_relaxed)) {
        // Attempt to set the flag to true only if it's currently false
        if (m_recovery_in_progress.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
            ESP_LOGI(TAG, "Marked recovery as in progress due to communication error. Waiting for ATTEMPT_IMU_RECOVERY_COMMAND.");
            // Note: If StateManager doesn't send the command, the flag remains true until explicitly cleared.
            // The flag should be reset by attemptRecovery() upon completion (success or final failure).
        } else {
             // This case means another thread set the flag between the initial check and the compare_exchange. Unlikely but possible.
             ESP_LOGW(TAG,"Communication error received, but recovery flag was concurrently set by another process.");
        }
    } else {
        ESP_LOGW(TAG, "Communication error received, but recovery was already marked as in progress.");
    }
}


esp_err_t IMUService::attemptRecovery() {
    // This function is called when m_recovery_in_progress is true, typically triggered by StateManager.
    // It attempts recovery steps and retries immediately a few times on failure.

    // <<< NEW: Define local constants for immediate retries >>>
    const int MAX_IMMEDIATE_RETRIES = 2; // Try immediate recovery this many times
    const int RETRY_DELAY_MS = 200;     // Delay between immediate retries

    esp_err_t status = ESP_FAIL; // Start assuming failure

    for (int attempt = 1; attempt <= MAX_IMMEDIATE_RETRIES; ++attempt) {
        ESP_LOGI(TAG, "Attempting IMU recovery (Immediate Attempt %d/%d)...", attempt, MAX_IMMEDIATE_RETRIES);
        status = ESP_OK; // Reset status for this attempt

        // --- Step 1: Reset Sensor ---
        if (status == ESP_OK) {
            ESP_LOGI(TAG,"Recovery Step 1: Resetting sensor...");
            status = m_driver.resetSensor();
            if (status != ESP_OK) { ESP_LOGE(TAG, "Recovery Step 1 Failed: Sensor reset error (%s)", esp_err_to_name(status)); }
            else { vTaskDelay(pdMS_TO_TICKS(100)); } // Delay only on success
        }

        // --- Step 2: Reconfigure Sensor Hardware ---
        if (status == ESP_OK) {
            ESP_LOGI(TAG,"Recovery Step 2: Reconfiguring sensor hardware...");
            status = configureSensorHardware();
            if (status != ESP_OK) { ESP_LOGE(TAG, "Recovery Step 2 Failed: Hardware reconfiguration error (%s)", esp_err_to_name(status)); }
            else { vTaskDelay(pdMS_TO_TICKS(50)); } // Delay only on success
        }

        // --- Step 3: Verify Communication ---
        if (status == ESP_OK) {
            ESP_LOGI(TAG,"Recovery Step 3: Verifying communication (WHO_AM_I)...");
            uint8_t who = 0;
            esp_err_t verify_ret = m_driver.readRegisters(MPU6050Register::WHO_AM_I, &who, 1);
            if (verify_ret != ESP_OK || who != IMUHealthMonitor::MPU6050_WHO_AM_I_VALUE) {
                ESP_LOGE(TAG, "Recovery Step 3 Failed: Comm verification error (ret=%s, WHO_AM_I=0x%02X)",
                         esp_err_to_name(verify_ret), who);
                status = (verify_ret == ESP_OK) ? ESP_FAIL : verify_ret; // Assign failure status
            } else {
                ESP_LOGI(TAG, "Recovery Step 3 Success: Communication verified.");
            }
        }

        // --- Step 4: Re-apply Software Offsets --- (Only if all hardware steps OK)
        if (status == ESP_OK) {
             ESP_LOGI(TAG, "Recovery Step 4: Re-applying software gyro offsets.");
             m_calibrationService.setOffsets(m_config.gyro_offset_x, m_config.gyro_offset_y, m_config.gyro_offset_z);
        }

        // --- Check Attempt Outcome ---
        if (status == ESP_OK) {
            ESP_LOGI(TAG, "IMU Immediate Recovery Attempt %d Succeeded!", attempt);
            break; // Exit the retry loop on success
        } else {
            ESP_LOGW(TAG, "IMU Immediate Recovery Attempt %d Failed (Error: %s).", attempt, esp_err_to_name(status));
            if (attempt < MAX_IMMEDIATE_RETRIES) {
                ESP_LOGI(TAG, "Delaying %dms before next immediate retry...", RETRY_DELAY_MS);
                vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
            }
        }
    } // End of immediate retry loop

    // --- Final Outcome ---
    if (status == ESP_OK) {
        // Success
        ESP_LOGI(TAG, "IMU Recovery Succeeded!");
        m_eventBus.publish(ImuRecoverySucceededEvent());
        m_recovery_in_progress.store(false, std::memory_order_release); // Reset flag on success
        m_healthMonitor.pet(); // Pet watchdog immediately
        return ESP_OK;
    } else {
        // Failure after immediate retries
        ESP_LOGE(TAG, "IMU Recovery Failed after %d immediate attempts (Last Error: %s). StateManager will handle overall retry logic.",
                 MAX_IMMEDIATE_RETRIES, esp_err_to_name(status));
        // Publish the failure event - StateManager will decide if max *overall* attempts are reached
        m_eventBus.publish(ImuRecoveryFailedEvent(status));
        m_recovery_in_progress.store(false, std::memory_order_release); // Reset flag to allow StateManager to trigger next cycle if needed
        return status; // Return the specific error code that caused the failure
    }
}