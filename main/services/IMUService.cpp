#include "IMUService.hpp"
#include "driver/gpio.h"
#include "esp_check.h"
#include <cmath>
#include <atomic>
#include <vector>
#include <numeric>
#include <algorithm>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "mpu6050.hpp"
#include "OrientationEstimator.hpp"
#include "OrientationDataEvent.hpp"

#include "EventBus.hpp"
#include "BaseEvent.hpp"
#include "StartCalibrationRequestEvent.hpp"
#include "CalibrationStartedEvent.hpp"
#include "CalibrationCompleteEvent.hpp"
#include "EventTypes.hpp"
#include "IMU_CommunicationErrorEvent.hpp"
#include "ImuRecoveryEvents.hpp"

#include <cstring> // For memcmp

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

const float G_ACCEL = 9.80665f;

// Define MPU6050 WHO_AM_I value
#define MPU6050_WHO_AM_I_VALUE 0x68

// Constants for FIFO data structure
constexpr size_t FIFO_PACKET_SIZE = 12; // 6 bytes accel + 6 bytes gyro
constexpr size_t ACCEL_X_OFFSET = 0;
constexpr size_t ACCEL_Y_OFFSET = 2;
constexpr size_t ACCEL_Z_OFFSET = 4;
constexpr size_t GYRO_X_OFFSET = 6;
constexpr size_t GYRO_Y_OFFSET = 8;
constexpr size_t GYRO_Z_OFFSET = 10;

// Constructor
IMUService::IMUService(const MPU6050Config& config, EventBus& bus, OrientationEstimator& estimator) :
    m_config(config),
    m_eventBus(bus),
    m_estimator(estimator),
    m_sensor(), // Use default constructor
    m_calibration_mutex(nullptr),
    m_is_calibrating(false),
    m_gyro_offset_dps{0.0f, 0.0f, 0.0f}, // Initialize array explicitly
    m_isr_data_counter(0), // Initialize atomic counter
    m_last_successful_read_timestamp_us(0),
    m_last_proactive_check_time_us(0),
    m_watchdog_enabled(true), // Initialize non-atomic bool
    m_watchdog_reset_flag(false),
    m_consecutive_i2c_failures(0),
    m_no_data_counter(0),
    m_last_disconnect_time_us(0),
    m_sensor_disconnected(false),
    m_last_data({{0, 0, 0}, {0, 0, 0}}), // Explicitly initialize struct
    m_unchanged_data_count(0),
    m_calib_gx_samples(), // Use default vector constructor
    m_calib_gy_samples(),
    m_calib_gz_samples()
{
    // Reserve space in vectors based on config
    m_calib_gx_samples.reserve(m_config.calibration_samples);
    m_calib_gy_samples.reserve(m_config.calibration_samples);
    m_calib_gz_samples.reserve(m_config.calibration_samples);
}

IMUService::~IMUService() {
    ESP_LOGI(TAG, "Deconstructing IMUService...");
    if (m_calibration_mutex) { vSemaphoreDelete(m_calibration_mutex); m_calibration_mutex = nullptr; }
    if (m_config.int_pin >= 0 && m_config.int_pin < GPIO_NUM_MAX) {
         // Ideally check if handler was installed before removing
         gpio_isr_handler_remove(m_config.int_pin);
    }
    ESP_LOGI(TAG, "IMUService deconstructed.");
}

esp_err_t IMUService::init() {
    ESP_LOGI(TAG, "Initializing IMUService...");
    m_calibration_mutex = xSemaphoreCreateMutex();
    ESP_RETURN_ON_FALSE(m_calibration_mutex != NULL, ESP_FAIL, TAG, "Failed create calibration mutex");

    esp_err_t ret = m_sensor.init(m_config.i2c_port, m_config.sda_pin, m_config.scl_pin, m_config.device_address, m_config.i2c_freq_hz);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed init MPU6050 driver");

    // Sensor configuration
    MPU6050AccelConfig accelRange = mapAccelConfig(m_config.accel_range);
    MPU6050GyroConfig gyroRange = mapGyroConfig(m_config.gyro_range);
    MPU6050DLPFConfig dlpf = static_cast<MPU6050DLPFConfig>(m_config.dlpf_config);
    MPU6050SampleRateDiv rateDiv = static_cast<MPU6050SampleRateDiv>(m_config.sample_rate_divisor);
    ESP_RETURN_ON_ERROR(m_sensor.setAccelRange(accelRange), TAG, "Failed set Accel Range");
    ESP_RETURN_ON_ERROR(m_sensor.setGyroRange(gyroRange), TAG, "Failed set Gyro Range");
    ESP_RETURN_ON_ERROR(m_sensor.setDLPFConfig(dlpf), TAG, "Failed set DLPF");
    ESP_RETURN_ON_ERROR(m_sensor.setSampleRate(rateDiv), TAG, "Failed set Sample Rate");

    // Initial calibration
    ret = performCalibration();
    CalibrationCompleteEvent initial_calib_event(ret);
    m_eventBus.publish(initial_calib_event);
    ESP_RETURN_ON_ERROR(ret, TAG, "Initial Gyro calibration failed");

    // Interrupt setup
    if (m_config.int_pin >= 0 && m_config.int_pin < GPIO_NUM_MAX) {
        gpio_config_t io_conf = {};
        io_conf.pin_bit_mask = (1ULL << m_config.int_pin); io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE; io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.intr_type = m_config.interrupt_active_high ? GPIO_INTR_POSEDGE : GPIO_INTR_NEGEDGE;
        ret = gpio_config(&io_conf); ESP_RETURN_ON_ERROR(ret, TAG, "Failed config INT GPIO %d", m_config.int_pin);
        static bool isr_service_installed = false;
        if (!isr_service_installed) { ret = gpio_install_isr_service(ESP_INTR_FLAG_LOWMED); if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) { ESP_RETURN_ON_ERROR(ret, TAG, "Failed install ISR service"); } isr_service_installed = (ret == ESP_OK || ret == ESP_ERR_INVALID_STATE); }
        ret = gpio_isr_handler_add(m_config.int_pin, isrHandler, this); if (ret == ESP_ERR_INVALID_STATE) { ESP_LOGW(TAG, "ISR handler already added for pin %d.", m_config.int_pin); ret = ESP_OK; }
        ESP_RETURN_ON_ERROR(ret, TAG, "Failed add ISR handler for pin %d", m_config.int_pin); ESP_LOGI(TAG, "GPIO interrupt configured for INT pin %d", m_config.int_pin);
        MPU6050InterruptPinConfig intPinConfig = m_config.interrupt_active_high ? MPU6050InterruptPinConfig::ACTIVE_HIGH : MPU6050InterruptPinConfig::ACTIVE_LOW;
        ESP_RETURN_ON_ERROR(m_sensor.setupInterrupt(intPinConfig, MPU6050Interrupt::DATA_READY), TAG, "Failed setup MPU Interrupt");
    } else { ESP_LOGW(TAG, "Interrupt pin not configured (pin=%d)", m_config.int_pin); }

    // FIFO setup
    ESP_RETURN_ON_ERROR(m_sensor.setupFIFO(MPU6050UserControl::FIFO_RESET_ENABLE, MPU6050FIFOEnable::GYRO_ACCEL), TAG, "Failed setup MPU FIFO");

    // Initialize watchdog timestamps
    m_last_successful_read_timestamp_us.store(esp_timer_get_time(), std::memory_order_release);
    m_last_proactive_check_time_us.store(m_last_successful_read_timestamp_us.load(std::memory_order_relaxed), std::memory_order_release);

    // Subscribe to Calibration Request
    m_eventBus.subscribe(EventType::START_CALIBRATION_REQUEST, [this](const BaseEvent& ev) { this->triggerCalibration(); }); ESP_LOGI(TAG, "Subscribed to START_CALIBRATION_REQUEST events.");
    m_eventBus.subscribe(EventType::ATTEMPT_IMU_RECOVERY_COMMAND, [this](const BaseEvent& ev) { this->handleAttemptRecoveryCommand(static_cast<const AttemptImuRecoveryCommand&>(ev)); }); ESP_LOGI(TAG, "Subscribed to ATTEMPT_IMU_RECOVERY_COMMAND events.");
    ESP_LOGI(TAG, "IMUService Initialized successfully.");
    return ESP_OK;
}

esp_err_t IMUService::triggerCalibration() {
    ESP_LOGI(TAG, "Calibration triggered.");
    if (xSemaphoreTake(m_calibration_mutex, pdMS_TO_TICKS(100)) != pdTRUE) { ESP_LOGW(TAG, "Calibration busy"); CalibrationCompleteEvent e(ESP_ERR_TIMEOUT); m_eventBus.publish(e); return ESP_ERR_TIMEOUT; }
    esp_err_t ret = performCalibration();
    CalibrationCompleteEvent e(ret); m_eventBus.publish(e); ESP_LOGI(TAG,"Published CalibComplete (Status: %s)", esp_err_to_name(ret));
    xSemaphoreGive(m_calibration_mutex);
    return ret;
}

esp_err_t IMUService::performCalibration() {
    if (m_is_calibrating) { return ESP_ERR_INVALID_STATE; }
    
    // Set calibration flag BEFORE any FIFO operations
    m_is_calibrating = true;
    
    // Signal system calibration state change
    CalibrationStartedEvent calib_start_event;
    m_eventBus.publish(calib_start_event);
    
    ESP_LOGI(TAG, "Starting Gyro calibration (X, Y, Z) for %d samples...", m_config.calibration_samples);
    
    // Stop FIFO operation during calibration
    esp_err_t ret = m_sensor.setupFIFO(MPU6050UserControl::FIFO_RESET, MPU6050FIFOEnable::GYRO_ACCEL);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to reset FIFO before calibration: %s", esp_err_to_name(ret));
        // Continue with calibration anyway
    }
    
    float old_offset_x_dps = m_gyro_offset_dps[0]; 
    float old_offset_y_dps = m_gyro_offset_dps[1]; 
    float old_offset_z_dps = m_gyro_offset_dps[2];
    double gx_sum = 0, gy_sum = 0, gz_sum = 0; 
    int successful_samples = 0; 
    int attempts = 0; 
    const int max_attempts = m_config.calibration_samples + 200;
    m_calib_gx_samples.clear(); 
    m_calib_gy_samples.clear(); 
    m_calib_gz_samples.clear();

    while (successful_samples < m_config.calibration_samples && attempts < max_attempts) {
        attempts++; 
        float ax, ay, az, gx_dps, gy_dps, gz_dps;
        
        // Use direct register reads for calibration, not FIFO
        esp_err_t ret_a = m_sensor.getAcceleration(ax, ay, az); 
        esp_err_t ret_g = m_sensor.getRotation(gx_dps, gy_dps, gz_dps);
        
        if (ret_a != ESP_OK || ret_g != ESP_OK) { 
            ESP_LOGE(TAG, "Read fail calib attempt %d", attempts); 
            vTaskDelay(pdMS_TO_TICKS(10)); 
            continue; 
        }
        
        gx_sum += gx_dps; 
        gy_sum += gy_dps; 
        gz_sum += gz_dps;
        m_calib_gx_samples.push_back(gx_dps); 
        m_calib_gy_samples.push_back(gy_dps); 
        m_calib_gz_samples.push_back(gz_dps); 
        successful_samples++;
        
        if ((successful_samples % (m_config.calibration_samples / 5) == 0) && successful_samples > 0) { 
            ESP_LOGI(TAG, "Calib progress: %d/%d", successful_samples, m_config.calibration_samples); 
        } 
        
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    esp_err_t final_status = ESP_OK;
    if (successful_samples < m_config.calibration_samples / 2) {
        ESP_LOGE(TAG, "Insufficient calib samples (%d/%d). Restoring previous offsets.", successful_samples, m_config.calibration_samples);
        m_gyro_offset_dps[0] = old_offset_x_dps; 
        m_gyro_offset_dps[1] = old_offset_y_dps; 
        m_gyro_offset_dps[2] = old_offset_z_dps; 
        final_status = ESP_FAIL;
    } else {
        float offset_gx_dps = gx_sum / successful_samples; 
        float offset_gy_dps = gy_sum / successful_samples; 
        float offset_gz_dps = gz_sum / successful_samples;
        m_gyro_offset_dps[0] = offset_gx_dps; 
        m_gyro_offset_dps[1] = offset_gy_dps; 
        m_gyro_offset_dps[2] = offset_gz_dps; // Store DPS
        
        auto calculate_stdev = [&](const std::vector<float>& samples, double mean) { 
            if (samples.empty()) return 0.0; 
            double sq_sum = std::inner_product(samples.begin(), samples.end(), samples.begin(), 0.0); 
            return std::sqrt(std::max(0.0, sq_sum / samples.size() - mean * mean)); 
        };
        
        double stdev_gx = calculate_stdev(m_calib_gx_samples, offset_gx_dps); 
        double stdev_gy = calculate_stdev(m_calib_gy_samples, offset_gy_dps); 
        double stdev_gz = calculate_stdev(m_calib_gz_samples, offset_gz_dps);
        
        ESP_LOGI(TAG, "Gyro Calib Complete (%d samples). Offsets(dps): X:%.4f Y:%.4f Z:%.4f | Stdev: X:%.4f Y:%.4f Z:%.4f", 
                successful_samples, offset_gx_dps, offset_gy_dps, offset_gz_dps, stdev_gx, stdev_gy, stdev_gz);
        
        const float OFFSET_WARN_THRESHOLD = 2.0; 
        const float STDEV_WARN_THRESHOLD = 1.0;
        
        if (std::abs(offset_gx_dps) > OFFSET_WARN_THRESHOLD || 
            std::abs(offset_gy_dps) > OFFSET_WARN_THRESHOLD || 
            std::abs(offset_gz_dps) > OFFSET_WARN_THRESHOLD || 
            stdev_gx > STDEV_WARN_THRESHOLD || 
            stdev_gy > STDEV_WARN_THRESHOLD || 
            stdev_gz > STDEV_WARN_THRESHOLD) { 
            ESP_LOGW(TAG, "Warning: Gyro offset/stdev high."); 
        }
        
        if (final_status != ESP_FAIL) final_status = ESP_OK;
    }
    
    // Restart FIFO after calibration is complete
    ret = m_sensor.setupFIFO(MPU6050UserControl::FIFO_RESET_ENABLE, MPU6050FIFOEnable::GYRO_ACCEL);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to restart FIFO after calibration: %s", esp_err_to_name(ret));
        // Consider if this should affect final status
    }
    
    // Reset ISR counter to prevent processing stale data
    m_isr_data_counter.store(0, std::memory_order_relaxed);
    
    // Clear calibration flag AFTER all operations
    m_is_calibrating = false; 
    
    ESP_LOGI(TAG, "Calibration finished: %s", esp_err_to_name(final_status)); 
    
    // Signal completion
    CalibrationCompleteEvent cal_event(final_status);
    m_eventBus.publish(cal_event);
    
    // If this was part of recovery, signal success
    ImuRecoverySucceededEvent successEvent;
    m_eventBus.publish(successEvent);
    
    return final_status;
}

// Static ISR handler
void IRAM_ATTR IMUService::isrHandler(void *arg) {
    IMUService *s = static_cast<IMUService *>(arg);
    if (s) {
        s->m_isr_data_counter.fetch_add(1, std::memory_order_relaxed);
    }
}

// Member function called by static handler (not used currently)
void IMUService::handleInterrupt() {}

// Process FIFO data - Exposed for IMUFifoTask
void IMUService::processFifoData() {
    // Configure scales based on sensor settings
    float accel_scale = 0.0f;
    float gyro_scale = 0.0f;
    
    switch (m_config.accel_range) {
        case 0: accel_scale = 16384.0f; break; // ±2g
        case 1: accel_scale = 8192.0f; break;  // ±4g
        case 2: accel_scale = 4096.0f; break;  // ±8g
        case 3: accel_scale = 2048.0f; break;  // ±16g
        default: accel_scale = 8192.0f; ESP_LOGW(TAG, "Unknown accel range %d, using ±4g", m_config.accel_range);
    }
    
    switch (m_config.gyro_range) {
        case 0: gyro_scale = 131.0f; break;  // ±250°/s
        case 1: gyro_scale = 65.5f; break;   // ±500°/s
        case 2: gyro_scale = 32.8f; break;   // ±1000°/s
        case 3: gyro_scale = 16.4f; break;   // ±2000°/s
        default: gyro_scale = 65.5f; ESP_LOGW(TAG, "Unknown gyro range %d, using ±500°/s", m_config.gyro_range);
    }
    
    // Bytes per sample (accel XYZ + gyro XYZ = 6 values * 2 bytes each)
    const uint8_t bytes_per_sample = 12;
    
    // Buffer for FIFO data (max 1024 bytes in MPU6050 FIFO)
    uint8_t fifo_buffer[1024];
    
    // Current offsets (updated during operation)
    float current_offset_y_dps = m_gyro_offset_dps[1];
    float current_offset_z_dps = m_gyro_offset_dps[2];
    
    // If calibrating, wait and skip processing
    if (m_is_calibrating) {
        vTaskDelay(pdMS_TO_TICKS(10));
        return;
    }
    
    // Check if we have enough data in the FIFO (wait for ISR counter)
    uint8_t isr_count = m_isr_data_counter.load(std::memory_order_relaxed);
    
    // CRITICAL CHANGE: Don't get stuck waiting for interrupts if they aren't firing
    // If after 50ms we don't have enough interrupts, try reading FIFO anyway
    static TickType_t last_fifo_read_time = 0;
    TickType_t current_time = xTaskGetTickCount();
    bool timeout_check = (current_time - last_fifo_read_time) > pdMS_TO_TICKS(50);
    
    if (isr_count < 5 && !timeout_check) { // Reduced from 10 to 5 and added timeout check
        return;
    }
    
    last_fifo_read_time = current_time;  // Update timestamp for timeout checking
    
    // Reset counter
    m_isr_data_counter.store(0, std::memory_order_relaxed);
    
    // Read FIFO Count
    uint8_t count_buf[2]; uint16_t fifo_count = 0;
    esp_err_t ret = m_sensor.readRegisters(MPU6050Register::FIFO_COUNT_H, count_buf, 2);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed read FIFO count: %s.", esp_err_to_name(ret));
        // Atomically increment and check
        uint8_t failures = m_consecutive_i2c_failures.fetch_add(1, std::memory_order_relaxed) + 1;
        if (failures >= I2C_FAILURE_THRESHOLD) {
            ESP_LOGE(TAG, "IMU failure threshold (%d) reached due to FIFO count read error. Publishing event.", failures);
            IMU_CommunicationErrorEvent error_event(ret); 
            m_eventBus.publish(error_event);
            m_consecutive_i2c_failures.store(0, std::memory_order_relaxed); // Reset after publishing
        }
        return; 
    } else {
        m_consecutive_i2c_failures.store(0, std::memory_order_relaxed);
    }
    fifo_count = (count_buf[0] << 8) | count_buf[1];

    // Handle Overflow
    if (fifo_count >= 1024 || m_sensor.isFIFOOverflow() == ESP_OK) {
        ESP_LOGW(TAG, "FIFO overflow detected (count=%d or flag). Resetting.", fifo_count);
        
        // First, disable FIFO to prevent new data during reset
        m_sensor.setupFIFO(MPU6050UserControl::FIFO_RESET, MPU6050FIFOEnable::GYRO_ACCEL);
        
        // Small delay to ensure reset completes
        vTaskDelay(pdMS_TO_TICKS(1));
        
        // Re-enable FIFO
        m_sensor.setupFIFO(MPU6050UserControl::FIFO_RESET_ENABLE, MPU6050FIFOEnable::GYRO_ACCEL);
        
        // Reset counters
        m_isr_data_counter.store(0, std::memory_order_relaxed);
        
        // Skip this iteration to allow FIFO to collect new data
        return;
    }

    // Check if enough data for full samples and Read Data
    if (fifo_count >= bytes_per_sample) {
        // Calculate bytes_to_read ensuring full samples
        uint16_t bytes_to_read = (fifo_count / bytes_per_sample) * bytes_per_sample;
        bytes_to_read = std::min((uint16_t)sizeof(fifo_buffer), bytes_to_read);

        // Read Data from HW FIFO
        ret = m_sensor.readRegisters(MPU6050Register::FIFO_R_W, fifo_buffer, bytes_to_read);
        if (ret != ESP_OK) {
             ESP_LOGE(TAG, "FIFO read data failed: %s.", esp_err_to_name(ret));
             // Atomically increment and check
             uint8_t failures = m_consecutive_i2c_failures.fetch_add(1, std::memory_order_relaxed) + 1;
             if (failures >= I2C_FAILURE_THRESHOLD) {
                 ESP_LOGE(TAG, "IMU failure threshold (%d) reached due to FIFO data read error. Publishing event.", failures);
                 IMU_CommunicationErrorEvent error_event(ret); 
                 m_eventBus.publish(error_event);
                 m_consecutive_i2c_failures.store(0, std::memory_order_relaxed); // Reset after publishing
             }
             return;
        } else {
             m_consecutive_i2c_failures.store(0, std::memory_order_relaxed);
        }

        // Sanity check: Verify FIFO data structure
        bool fifo_structure_valid = true;
        if (bytes_to_read >= bytes_per_sample) {
            // Check first sample for reasonable values
            for (int i = 0; i < bytes_per_sample; i += 2) {
                int16_t value = (fifo_buffer[i] << 8) | fifo_buffer[i+1];
                
                // Raw values should generally be in a reasonable range
                // This is a basic check to catch completely corrupted data
                if (abs(value) > 32000) {
                    fifo_structure_valid = false;
                    ESP_LOGW(TAG, "FIFO data structure validation failed at offset %d: value %d out of normal range", 
                            i, value);
                    break;
                }
            }
            
            if (!fifo_structure_valid) {
                ESP_LOGW(TAG, "FIFO data appears corrupted, resetting FIFO");
                m_sensor.setupFIFO(MPU6050UserControl::FIFO_RESET_ENABLE, MPU6050FIFOEnable::GYRO_ACCEL);
                vTaskDelay(pdMS_TO_TICKS(1)); // Small delay after reset
                return;
            }
        }

        // Process first sample for data consistency check
        if (bytes_to_read >= bytes_per_sample) {
            // Extract first sample for consistency check
            IMUData sample_data;
            int16_t ax = (fifo_buffer[ACCEL_X_OFFSET] << 8) | fifo_buffer[ACCEL_X_OFFSET + 1];
            int16_t ay = (fifo_buffer[ACCEL_Y_OFFSET] << 8) | fifo_buffer[ACCEL_Y_OFFSET + 1];
            int16_t az = (fifo_buffer[ACCEL_Z_OFFSET] << 8) | fifo_buffer[ACCEL_Z_OFFSET + 1];
            int16_t gx = (fifo_buffer[GYRO_X_OFFSET] << 8) | fifo_buffer[GYRO_X_OFFSET + 1];
            int16_t gy = (fifo_buffer[GYRO_Y_OFFSET] << 8) | fifo_buffer[GYRO_Y_OFFSET + 1];
            int16_t gz = (fifo_buffer[GYRO_Z_OFFSET] << 8) | fifo_buffer[GYRO_Z_OFFSET + 1];
            
            // Convert to physical units
            sample_data.accel.x = ax / accel_scale;
            sample_data.accel.y = ay / accel_scale;
            sample_data.accel.z = az / accel_scale;
            sample_data.gyro.x = gx / gyro_scale;
            sample_data.gyro.y = gy / gyro_scale;
            sample_data.gyro.z = gz / gyro_scale;
            
            // Check data consistency
            if (!isDataConsistent(sample_data)) {
                ESP_LOGW(TAG, "Inconsistent IMU data detected, skipping processing");
                return;
            }
        }

        // Process Data - Pass DPS offsets directly
        m_estimator.processFifoBatch(fifo_buffer, bytes_to_read, accel_scale, gyro_scale,
                                     current_offset_y_dps, // Already in DPS
                                     current_offset_z_dps); // Already in DPS
        
        // Pet the watchdog to indicate successful data processing
        petWatchdog();
    }
}

bool IMUService::isDataConsistent(const IMUData& data) {
    // Check for NaN values in accelerometer data
    if (std::isnan(data.accel.x) || std::isnan(data.accel.y) || std::isnan(data.accel.z)) {
        ESP_LOGW(TAG, "NaN values in accelerometer data");
        return false;
    }
    
    // Check for NaN values in gyroscope data
    if (std::isnan(data.gyro.x) || std::isnan(data.gyro.y) || std::isnan(data.gyro.z)) {
        ESP_LOGW(TAG, "NaN values in gyroscope data");
        return false;
    }
    
    // Check for infinite values in accelerometer data
    if (std::isinf(data.accel.x) || std::isinf(data.accel.y) || std::isinf(data.accel.z)) {
        ESP_LOGW(TAG, "Infinite values in accelerometer data");
        return false;
    }
    
    // Check for infinite values in gyroscope data
    if (std::isinf(data.gyro.x) || std::isinf(data.gyro.y) || std::isinf(data.gyro.z)) {
        ESP_LOGW(TAG, "Infinite values in gyroscope data");
        return false;
    }
    
    // Check accelerometer values against maximum limits
    if (std::abs(data.accel.x) > MAX_ACCEL_G || 
        std::abs(data.accel.y) > MAX_ACCEL_G || 
        std::abs(data.accel.z) > MAX_ACCEL_G) {
        ESP_LOGW(TAG, "Accelerometer values exceed maximum limits: x=%.2f, y=%.2f, z=%.2f", 
                 data.accel.x, data.accel.y, data.accel.z);
        return false;
    }
    
    // Check gyroscope values against maximum limits
    if (std::abs(data.gyro.x) > MAX_GYRO_RATE_DPS || 
        std::abs(data.gyro.y) > MAX_GYRO_RATE_DPS || 
        std::abs(data.gyro.z) > MAX_GYRO_RATE_DPS) {
        ESP_LOGW(TAG, "Gyroscope values exceed maximum limits: x=%.2f, y=%.2f, z=%.2f", 
                 data.gyro.x, data.gyro.y, data.gyro.z);
        return false;
    }
    
    // If we've gone multiple cycles with identical data, that's suspicious
    bool identical_data = (
        data.accel.x == m_last_data.accel.x &&
        data.accel.y == m_last_data.accel.y &&
        data.accel.z == m_last_data.accel.z &&
        data.gyro.x == m_last_data.gyro.x &&
        data.gyro.y == m_last_data.gyro.y &&
        data.gyro.z == m_last_data.gyro.z
    );
    
    if (identical_data) {
        m_unchanged_data_count++;
        if (m_unchanged_data_count >= STUCK_DATA_THRESHOLD) {
            ESP_LOGW(TAG, "Detected stuck IMU data - same values for %d consecutive reads", 
                     m_unchanged_data_count);
            m_unchanged_data_count = 0; // Reset counter
            return false;
        }
    } else {
        m_unchanged_data_count = 0;
        // Store current data for next comparison
        m_last_data = data;
    }
    
    return true;
}

// Watchdog petting - Exposed for tasks
void IMUService::petWatchdog() {
    m_last_successful_read_timestamp_us.store(esp_timer_get_time(), std::memory_order_release);
}

// Check IMU health - Exposed for IMUWatchdogTask
void IMUService::checkImuHealth() {
    // Skip if watchdog is disabled
    if (!m_watchdog_enabled) {
        return;
    }
    
    // Get current time
    int64_t current_time = esp_timer_get_time();
    
    // Check if we need to do a proactive health check
    int64_t last_proactive_check = m_last_proactive_check_time_us.load(std::memory_order_acquire);
    bool do_proactive_check = (current_time - last_proactive_check) > PROACTIVE_CHECK_INTERVAL_US;
    
    // Check if we've had a successful read recently
    int64_t last_successful_read = m_last_successful_read_timestamp_us.load(std::memory_order_acquire);
    bool data_timeout = (current_time - last_successful_read) > IMU_DATA_TIMEOUT_US;
    
    // If we're not in a timeout situation and it's not time for a proactive check, just return
    if (!data_timeout && !do_proactive_check) {
        return;
    }
    
    // Update proactive check timestamp if this is a proactive check
    if (do_proactive_check) {
        m_last_proactive_check_time_us.store(current_time, std::memory_order_release);
        ESP_LOGI(TAG, "Performing proactive IMU health check");
    }
    
    // If we have a data timeout, log it
    if (data_timeout) {
        ESP_LOGW(TAG, "IMU data timeout detected! Last successful read: %.2f seconds ago", 
                 (current_time - last_successful_read) / 1000000.0f);
    }
    
    // Check if we can communicate with the sensor at all
    uint8_t who_am_i = 0;
    esp_err_t ret = m_sensor.readRegisters(MPU6050Register::WHO_AM_I, &who_am_i, 1);
    
    if (ret != ESP_OK) {
        // Communication error
        ESP_LOGE(TAG, "IMU communication error during health check: %s", esp_err_to_name(ret));
        
        // Increment failure counter
        uint8_t failures = m_consecutive_i2c_failures.fetch_add(1, std::memory_order_relaxed) + 1;
        
        // Check if we've reached the threshold for reporting
        if (failures >= I2C_FAILURE_THRESHOLD) {
            ESP_LOGE(TAG, "IMU failure threshold (%d) reached. Publishing event.", failures);
            
            // Publish error event
            IMU_CommunicationErrorEvent error_event(ret);
            m_eventBus.publish(error_event);
            
            // Reset counter after publishing
            m_consecutive_i2c_failures.store(0, std::memory_order_relaxed);
            
            // Set disconnected flag
            m_sensor_disconnected = true;
            m_last_disconnect_time_us = current_time;
        }
        
        // Reset no-data counter since this is a communication error
        m_no_data_counter.store(0, std::memory_order_relaxed);
        
        return;
    }
    
    // Reset I2C failure counter since we successfully communicated
    m_consecutive_i2c_failures.store(0, std::memory_order_relaxed);
    
    // Check WHO_AM_I value
    if (who_am_i != MPU6050_WHO_AM_I_VALUE) {
        ESP_LOGE(TAG, "IMU WHO_AM_I check failed! Expected 0x%02X, got 0x%02X", 
                 MPU6050_WHO_AM_I_VALUE, who_am_i);
        
        // This is a serious error - the device is responding but with wrong identity
        // Publish error event immediately
        IMU_CommunicationErrorEvent error_event(ESP_ERR_INVALID_RESPONSE);
        m_eventBus.publish(error_event);
        
        // Set disconnected flag
        m_sensor_disconnected = true;
        m_last_disconnect_time_us = current_time;
        
        return;
    }
    
    // If we were previously disconnected, log reconnection
    if (m_sensor_disconnected) {
        ESP_LOGI(TAG, "IMU reconnected after %.2f seconds", 
                 (current_time - m_last_disconnect_time_us) / 1000000.0f);
        m_sensor_disconnected = false;
    }
    
    // If we have a data timeout but can communicate with the sensor,
    // it means the sensor is responsive but not providing data
    if (data_timeout) {
        // Increment no-data counter
        uint8_t no_data_count = m_no_data_counter.fetch_add(1, std::memory_order_relaxed) + 1;
        
        // Check if we've reached the threshold for reporting
        if (no_data_count >= NO_DATA_FAILURE_THRESHOLD) {
            ESP_LOGE(TAG, "IMU no-data threshold (%d) reached. Attempting recovery.", no_data_count);
            
            // Reset counter
            m_no_data_counter.store(0, std::memory_order_relaxed);
            
            // Set watchdog reset flag
            m_watchdog_reset_flag.store(true, std::memory_order_release);
            
            // Attempt recovery
            attemptRecovery();
        }
    } else {
        // Reset no-data counter if we're not in a timeout situation
        m_no_data_counter.store(0, std::memory_order_relaxed);
    }
}

esp_err_t IMUService::resetSensor() {
    ESP_LOGI(TAG, "Attempting IMU sensor reset...");
    esp_err_t ret = m_sensor.writeRegister(MPU6050Register::PWR_MGMT_1, 0x80); // Set RESET bit
    if (ret == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(100));
        ret = m_sensor.writeRegister(MPU6050Register::PWR_MGMT_1, 0x00); // Wake up
        if (ret == ESP_OK) {
             // Set clock source explicitly after reset
             ret = m_sensor.writeRegister(MPU6050Register::PWR_MGMT_1, static_cast<uint8_t>(MPU6050PowerManagement::CLOCK_PLL_XGYRO));
        }
    }
    ESP_LOGI(TAG, "Sensor reset attempt finished: %s", esp_err_to_name(ret));
    return ret;
}

esp_err_t IMUService::reinitializeSensor() {
    ESP_LOGI(TAG, "Reinitializing IMU sensor configuration...");
    esp_err_t ret = ESP_OK;
    MPU6050AccelConfig ar = mapAccelConfig(m_config.accel_range); MPU6050GyroConfig gr = mapGyroConfig(m_config.gyro_range); MPU6050DLPFConfig dlpf = static_cast<MPU6050DLPFConfig>(m_config.dlpf_config); MPU6050SampleRateDiv sr = static_cast<MPU6050SampleRateDiv>(m_config.sample_rate_divisor);
    ret = m_sensor.setAccelRange(ar); if (ret != ESP_OK) { ESP_LOGE(TAG, "Reinit Accel failed: %s", esp_err_to_name(ret)); return ret; }
    ret = m_sensor.setGyroRange(gr); if (ret != ESP_OK) { ESP_LOGE(TAG, "Reinit Gyro failed: %s", esp_err_to_name(ret)); return ret; }
    ret = m_sensor.setDLPFConfig(dlpf); if (ret != ESP_OK) { ESP_LOGE(TAG, "Reinit DLPF failed: %s", esp_err_to_name(ret)); return ret; }
    ret = m_sensor.setSampleRate(sr); if (ret != ESP_OK) { ESP_LOGE(TAG, "Reinit Sample Rate failed: %s", esp_err_to_name(ret)); return ret; }
    if (m_config.int_pin >= 0 && m_config.int_pin < GPIO_NUM_MAX) { MPU6050InterruptPinConfig ipc = m_config.interrupt_active_high ? MPU6050InterruptPinConfig::ACTIVE_HIGH : MPU6050InterruptPinConfig::ACTIVE_LOW; ret = m_sensor.setupInterrupt(ipc, MPU6050Interrupt::DATA_READY); if (ret != ESP_OK) { ESP_LOGE(TAG, "Reinit Interrupt failed: %s", esp_err_to_name(ret)); return ret; } }
    ret = m_sensor.setupFIFO(MPU6050UserControl::FIFO_RESET_ENABLE, MPU6050FIFOEnable::GYRO_ACCEL); if (ret != ESP_OK) { ESP_LOGE(TAG, "Reinit FIFO failed: %s", esp_err_to_name(ret)); return ret; }
    m_isr_data_counter.store(0, std::memory_order_relaxed); m_consecutive_i2c_failures = 0;
    ESP_LOGI(TAG, "Sensor reinitialization finished: %s", esp_err_to_name(ret));
    return ret;
}

esp_err_t IMUService::verifyCommunication() {
    ESP_LOGI(TAG, "Verifying IMU communication...");
    uint8_t who = 0;
    esp_err_t ret = m_sensor.readRegisters(MPU6050Register::WHO_AM_I, &who, 1); // Read only 1 byte
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Communication verified (WHO_AM_I=0x%02X).", who);
        // Check against expected MPU6050 default or configured address (0x68 is common)
        if (who != m_config.device_address && who != 0x68) {
             ESP_LOGW(TAG, "WHO_AM_I value (0x%02X) unexpected (expected 0x%02X or 0x68).", who, m_config.device_address);
             // ret = ESP_FAIL; // Optionally fail if unexpected ID
        }
    } else {
        ESP_LOGE(TAG, "Communication verification failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

// Example event handler (can be replaced by lambda in init)
void IMUService::handleStartCalibrationRequest(const BaseEvent& event) {
    ESP_LOGW(TAG, "Handling StartCalibrationRequest via method (should use lambda).");
    triggerCalibration();
}

void IMUService::handleAttemptRecoveryCommand(const AttemptImuRecoveryCommand& event) {
    ESP_LOGI(TAG, "Received IMU recovery command, attempting recovery");
    attemptRecovery();
}

void IMUService::attemptRecovery() {
    ESP_LOGI(TAG, "Starting IMU recovery process");
    
    // Disable watchdog during recovery
    m_watchdog_enabled = false;
    
    // First try to reset the sensor
    esp_err_t ret = resetSensor();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "IMU reset failed: %s", esp_err_to_name(ret));
        // Publish failure event with error code
        ImuRecoveryFailedEvent failEvent(ret);
        m_eventBus.publish(failEvent);
        
        // Re-enable watchdog before returning
        petWatchdog(); // Reset the watchdog timer
        m_watchdog_enabled = true;
        return;
    }
    
    // Then try to reinitialize
    ret = reinitializeSensor();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "IMU reinitialization failed: %s", esp_err_to_name(ret));
        // Publish failure event with error code
        ImuRecoveryFailedEvent failEvent(ret);
        m_eventBus.publish(failEvent);
        
        // Re-enable watchdog before returning
        petWatchdog(); // Reset the watchdog timer
        m_watchdog_enabled = true;
        return;
    }
    
    // Verify communication is working
    ret = verifyCommunication();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "IMU communication verification failed: %s", esp_err_to_name(ret));
        // Publish failure event with error code
        ImuRecoveryFailedEvent failEvent(ret);
        m_eventBus.publish(failEvent);
        
        // Re-enable watchdog before returning
        petWatchdog(); // Reset the watchdog timer
        m_watchdog_enabled = true;
        return;
    }
    
    // Skip calibration for faster recovery
    // Only use existing calibration values
    ESP_LOGI(TAG, "Using existing calibration values for faster recovery");
    
    // Signal recovery success
    ImuRecoverySucceededEvent successEvent;
    m_eventBus.publish(successEvent);
    
    // Re-enable watchdog after recovery
    petWatchdog(); // Reset the watchdog timer
    m_watchdog_enabled = true;
    
    ESP_LOGI(TAG, "IMU recovery completed successfully without recalibration");
}

MPU6050AccelConfig IMUService::mapAccelConfig(int config_value) const {
    switch(config_value) {
        case 0: return MPU6050AccelConfig::RANGE_2G;
        case 1: return MPU6050AccelConfig::RANGE_4G;
        case 2: return MPU6050AccelConfig::RANGE_8G;
        case 3: return MPU6050AccelConfig::RANGE_16G;
        default:
            ESP_LOGW(TAG, "Invalid accel range value %d, using default ±4g", config_value);
            return MPU6050AccelConfig::RANGE_4G;
    }
}

MPU6050GyroConfig IMUService::mapGyroConfig(int config_value) const {
    switch(config_value) {
        case 0: return MPU6050GyroConfig::RANGE_250_DEG;
        case 1: return MPU6050GyroConfig::RANGE_500_DEG;
        case 2: return MPU6050GyroConfig::RANGE_1000_DEG;
        case 3: return MPU6050GyroConfig::RANGE_2000_DEG;
        default:
            ESP_LOGW(TAG, "Invalid gyro range value %d, using default ±500°/s", config_value);
            return MPU6050GyroConfig::RANGE_500_DEG;
    }
}