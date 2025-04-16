#include "IMUCalibrationService.hpp"
#include "mpu6050.hpp" // Need MPU6050Driver definition
#include "ConfigData.hpp" // Need MPU6050Config definition
#include "EventBus.hpp"
#include "EventTypes.hpp"
#include "CalibrationStartedEvent.hpp"
#include "CalibrationCompleteEvent.hpp"
#include "BaseEvent.hpp" // <<< ADDED Include needed
#include "StartCalibrationRequestEvent.hpp" // For subscription
#include "esp_check.h"
#include "freertos/task.h" // For vTaskDelay
#include <numeric> // For std::accumulate, std::inner_product
#include <cmath>   // For std::sqrt, std::abs, std::max
#include <algorithm> // For std::max


IMUCalibrationService::IMUCalibrationService(MPU6050Driver& driver, const MPU6050Config& imuConfig, EventBus& bus) :
    m_driver(driver),
    m_config(imuConfig),
    m_eventBus(bus),
    m_mutex(nullptr),
    m_is_calibrating_flag(false),
    m_gyro_offset_dps{0.0f, 0.0f, 0.0f},
    m_gyro_lsb_per_dps(1.0f) // Default, will be calculated
{
    // Reserve space based on config
    m_calib_gx_samples.reserve(m_config.calibration_samples);
    m_calib_gy_samples.reserve(m_config.calibration_samples);
    m_calib_gz_samples.reserve(m_config.calibration_samples);
}

IMUCalibrationService::~IMUCalibrationService() {
    if (m_mutex) {
        vSemaphoreDelete(m_mutex);
        m_mutex = nullptr;
    }
}

float IMUCalibrationService::calculateGyroScaleFactor() const {
     // Get the appropriate scale factor based on the configured range
    MPU6050GyroConfig range_enum = static_cast<MPU6050GyroConfig>(m_config.gyro_range);
    float scale = 1.0f; // Default
    switch(range_enum) {
        case MPU6050GyroConfig::RANGE_250_DEG:  scale = 131.0f; break;
        case MPU6050GyroConfig::RANGE_500_DEG:  scale = 65.5f;  break;
        case MPU6050GyroConfig::RANGE_1000_DEG: scale = 32.8f;  break;
        case MPU6050GyroConfig::RANGE_2000_DEG: scale = 16.4f;  break;
        default: ESP_LOGW(TAG, "Unknown gyro range enum %d in config, using default scale!", (int)range_enum); scale = 65.5f; break;
    }
    ESP_LOGD(TAG, "Gyro scale factor calculated: %.1f LSB/DPS for range %d", scale, (int)range_enum);
    return scale;
}


esp_err_t IMUCalibrationService::init() {
    ESP_LOGI(TAG, "Initializing IMUCalibrationService...");
    m_mutex = xSemaphoreCreateMutex();
    ESP_RETURN_ON_FALSE(m_mutex != NULL, ESP_FAIL, TAG, "Failed to create calibration mutex");

    // Calculate scale factor based on initial config
    m_gyro_lsb_per_dps = calculateGyroScaleFactor();

    // <<< REMOVED: Subscription moved >>>

    // Perform an initial calibration
    ESP_LOGI(TAG, "Performing initial gyro calibration...");
    esp_err_t calib_ret = calibrate();
    if (calib_ret != ESP_OK) {
        ESP_LOGE(TAG, "Initial gyro calibration failed (%s). Offsets may be zero or inaccurate.", esp_err_to_name(calib_ret));
        // Continue initialization even if calibration fails initially
    } else {
        ESP_LOGI(TAG, "Initial gyro calibration successful.");
    }

    ESP_LOGI(TAG, "IMUCalibrationService Initialized.");
    return ESP_OK;
}

// <<< ADDED: Event subscription logic >>>
void IMUCalibrationService::subscribeToEvents(EventBus& bus) {
    // Subscribe to external calibration requests
    bus.subscribe(EventType::START_CALIBRATION_REQUEST, [this](const BaseEvent& ev) {
        ESP_LOGD(TAG, "Received START_CALIBRATION_REQUEST, triggering calibration.");
        this->calibrate(); // Call member function directly
    });
    ESP_LOGI(TAG, "Subscribed to START_CALIBRATION_REQUEST events.");
}

esp_err_t IMUCalibrationService::calibrate() {
    if (m_is_calibrating_flag) {
        ESP_LOGW(TAG, "Calibration already in progress.");
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(m_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Could not acquire calibration mutex (timeout).");
        // Publish failure event? Maybe not here, let caller handle immediate failure.
        return ESP_ERR_TIMEOUT;
    }

    m_is_calibrating_flag = true;
    ESP_LOGI(TAG, "Starting Gyro calibration for %d samples...", m_config.calibration_samples);
    m_eventBus.publish(CalibrationStartedEvent());

    // --- Store Old Offsets & Clear Accumulators/Vectors ---
    float old_offset_x_dps = m_gyro_offset_dps[0];
    float old_offset_y_dps = m_gyro_offset_dps[1];
    float old_offset_z_dps = m_gyro_offset_dps[2];
    double gx_sum_dps = 0, gy_sum_dps = 0, gz_sum_dps = 0;
    int successful_samples = 0;
    int attempts = 0;
    const int max_attempts = m_config.calibration_samples + 200; // Allow for some read failures
    m_calib_gx_samples.clear();
    m_calib_gy_samples.clear();
    m_calib_gz_samples.clear();

    // Re-calculate scale factor in case config changed (though unlikely without restart)
    float current_scale = calculateGyroScaleFactor();
    if (current_scale <= 0) {
         ESP_LOGE(TAG, "Invalid gyro scale factor (%.1f), cannot calibrate.", current_scale);
         m_is_calibrating_flag = false;
         xSemaphoreGive(m_mutex);
         m_eventBus.publish(CalibrationCompleteEvent(ESP_FAIL));
         return ESP_FAIL;
    }

    // --- Calibration Loop ---
    while (successful_samples < m_config.calibration_samples && attempts < max_attempts) {
        attempts++;
        int16_t raw_gx, raw_gy, raw_gz;

        // Use the driver to read RAW gyro data
        esp_err_t read_ret = m_driver.readRawGyroXYZ(raw_gx, raw_gy, raw_gz);

        if (read_ret != ESP_OK) {
            ESP_LOGE(TAG, "Gyro read fail during calib attempt %d: %s", attempts, esp_err_to_name(read_ret));
            vTaskDelay(pdMS_TO_TICKS(10)); // Small delay before retrying read
            continue;
        }

        // Convert RAW LSB to DPS using the current scale factor
        float gx_dps = static_cast<float>(raw_gx) / current_scale;
        float gy_dps = static_cast<float>(raw_gy) / current_scale;
        float gz_dps = static_cast<float>(raw_gz) / current_scale;

        // Accumulate DPS values
        gx_sum_dps += gx_dps;
        gy_sum_dps += gy_dps;
        gz_sum_dps += gz_dps;
        m_calib_gx_samples.push_back(gx_dps);
        m_calib_gy_samples.push_back(gy_dps);
        m_calib_gz_samples.push_back(gz_dps);
        successful_samples++;

        // Log progress periodically
        if ((successful_samples % (m_config.calibration_samples / 5) == 0) && successful_samples > 0) {
            ESP_LOGD(TAG, "Calibration progress: %d/%d", successful_samples, m_config.calibration_samples);
        }

        vTaskDelay(pdMS_TO_TICKS(5)); // Delay between samples
    }

    // --- Process Results ---
    esp_err_t final_status = ESP_FAIL; // Default to failure
    if (successful_samples >= m_config.calibration_samples / 2) { // Require at least half the samples
        float offset_gx_dps = gx_sum_dps / successful_samples;
        float offset_gy_dps = gy_sum_dps / successful_samples;
        float offset_gz_dps = gz_sum_dps / successful_samples;
        m_gyro_offset_dps[0] = offset_gx_dps;
        m_gyro_offset_dps[1] = offset_gy_dps;
        m_gyro_offset_dps[2] = offset_gz_dps;

        // Calculate Standard Deviation (Optional but useful)
        auto calculate_stdev = [&](const std::vector<float>& samples, double mean) {
            if (samples.empty()) return 0.0;
            double sq_sum = std::inner_product(samples.begin(), samples.end(), samples.begin(), 0.0);
            double variance = std::max(0.0, sq_sum / samples.size() - mean * mean); // Ensure non-negative variance
            return std::sqrt(variance);
        };
        double stdev_gx = calculate_stdev(m_calib_gx_samples, offset_gx_dps);
        double stdev_gy = calculate_stdev(m_calib_gy_samples, offset_gy_dps);
        double stdev_gz = calculate_stdev(m_calib_gz_samples, offset_gz_dps);

        ESP_LOGI(TAG, "Gyro Calib Complete (%d samples). Offsets(dps): X:%.4f Y:%.4f Z:%.4f | Stdev: X:%.4f Y:%.4f Z:%.4f",
                 successful_samples, offset_gx_dps, offset_gy_dps, offset_gz_dps, stdev_gx, stdev_gy, stdev_gz);

        // Warning thresholds (example)
        const float OFFSET_WARN_THRESHOLD = 2.0; // dps
        const float STDEV_WARN_THRESHOLD = 1.0;  // dps
        if (std::abs(offset_gx_dps) > OFFSET_WARN_THRESHOLD || std::abs(offset_gy_dps) > OFFSET_WARN_THRESHOLD || std::abs(offset_gz_dps) > OFFSET_WARN_THRESHOLD ||
            stdev_gx > STDEV_WARN_THRESHOLD || stdev_gy > STDEV_WARN_THRESHOLD || stdev_gz > STDEV_WARN_THRESHOLD) {
            ESP_LOGW(TAG, "Warning: Gyro offset or standard deviation seems high. Ensure robot was stationary.");
        }
        final_status = ESP_OK; // Calibration succeeded
    } else {
        ESP_LOGE(TAG, "Insufficient successful samples (%d/%d) for calibration. Restoring previous offsets.",
                 successful_samples, m_config.calibration_samples);
        m_gyro_offset_dps[0] = old_offset_x_dps;
        m_gyro_offset_dps[1] = old_offset_y_dps;
        m_gyro_offset_dps[2] = old_offset_z_dps;
        final_status = ESP_FAIL; // Calibration failed
    }

    // --- Cleanup and Finalize ---
    m_is_calibrating_flag = false;
    xSemaphoreGive(m_mutex);
    ESP_LOGI(TAG, "Calibration finished: %s", esp_err_to_name(final_status));

    // Publish completion event
    m_eventBus.publish(CalibrationCompleteEvent(final_status));
    return final_status;
}

// --- Getters (Thread-safe via mutex if needed, but reading floats might be atomic enough) ---
// For simplicity, not using mutex here, assuming reads are infrequent enough relative to writes (only during calib)
// Add mutex lock/unlock around reads if strict consistency is required during calibration.
float IMUCalibrationService::getGyroOffsetXDPS() const { return m_gyro_offset_dps[0]; }
float IMUCalibrationService::getGyroOffsetYDPS() const { return m_gyro_offset_dps[1]; }
float IMUCalibrationService::getGyroOffsetZDPS() const { return m_gyro_offset_dps[2]; }

bool IMUCalibrationService::isCalibrating() const {
    // Reading a volatile bool should be safe without mutex
    return m_is_calibrating_flag;
}

// Event handler (can be replaced by lambda in init)
void IMUCalibrationService::handleStartCalibrationRequest(const BaseEvent& event) {
    ESP_LOGD(TAG, "Handling StartCalibrationRequest via method."); // Use DEBUG level
    calibrate();
}