#include "IMUCalibrationService.hpp"
#include "mpu6050.hpp" // Need MPU6050Driver definition
#include "ConfigData.hpp" // Need MPU6050Config definition
#include "EventBus.hpp"
#include "EventTypes.hpp"
#include "IMU_CalibrationStarted.hpp"
#include "IMU_CalibrationCompleted.hpp"
#include "BaseEvent.hpp"
// #include "IMU_CalibrationRequest.hpp"; // No longer subscribing here, task handles it
#include "IMU_GyroOffsetsUpdated.hpp" // For publishing offset updates
#include "IMUService.hpp" // For IMUState enum
#include "esp_check.h"
#include "freertos/task.h" // For vTaskDelay
#include <numeric> // For std::accumulate, std::inner_product
#include <cmath>   // For std::sqrt, std::abs, std::max
#include <algorithm> // For std::max
// #include "IMU_StateTransitionRequest.hpp"; // Requesting state transition removed
#include "IMU_StateChanged.hpp"

IMUCalibrationService::IMUCalibrationService(MPU6050Driver& driver, const MPU6050Config& imuConfig, EventBus& bus) :
    m_driver(driver),
    m_config(imuConfig),
    m_eventBus(bus),
    m_mutex(nullptr),
    m_is_calibrating_flag(false),
    m_current_imu_state(IMUState::INITIALIZED), // Initialize with default state
    m_gyro_offset_dps{0.0f, 0.0f, 0.0f}, // Initialize offsets
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
    float scale = 1.0f; // Default
    switch(m_config.gyro_range) {
        case 0: scale = 131.0f; break;
        case 1: scale = 65.5f;  break;
        case 2: scale = 32.8f;  break;
        case 3: scale = 16.4f;  break;
        default: ESP_LOGW(TAG, "Unknown gyro range %d in config, using default scale!", m_config.gyro_range); scale = 65.5f; break;
    }
    ESP_LOGD(TAG, "Gyro scale factor calculated: %.1f LSB/DPS for range %d", scale, m_config.gyro_range);
    return scale;
}


esp_err_t IMUCalibrationService::init() {
    ESP_LOGI(TAG, "Initializing IMUCalibrationService...");
    m_mutex = xSemaphoreCreateMutex();
    ESP_RETURN_ON_FALSE(m_mutex != NULL, ESP_FAIL, TAG, "Failed to create calibration mutex");

    // Calculate scale factor based on initial config
    m_gyro_lsb_per_dps = calculateGyroScaleFactor();
    // Do *not* calibrate here - IMUService will load offsets from config first
    // and calibration is triggered by an event.
    ESP_LOGI(TAG, "IMUCalibrationService Initialized.");
    return ESP_OK;
}

void IMUCalibrationService::subscribeToEvents(EventBus& bus) {
    // IMUCalibrationTask subscribes to IMU_CALIBRATION_REQUEST.
    // This service only needs to know about IMU state changes.
    bus.subscribe(EventType::IMU_STATE_CHANGED, [this](const BaseEvent& ev){
        const auto& stateEvent = static_cast<const IMU_StateChanged&>(ev);
        this->notifyIMUStateChange(stateEvent.newState);
    });
    
    ESP_LOGI(TAG, "Subscribed to IMU_STATE_CHANGED events.");
}

void IMUCalibrationService::notifyIMUStateChange(IMUState newState) {
    IMUState oldState = m_current_imu_state.exchange(newState);
    ESP_LOGD(TAG, "IMU state changed from %s to %s", IMUService::stateToString(oldState), IMUService::stateToString(newState)); // Use shared stringify
    
    // Adapt behavior based on new state
    if (newState == IMUState::CALIBRATION) {
        if (!m_is_calibrating_flag) {
            ESP_LOGI(TAG, "IMU entered CALIBRATION state but calibration routine not actively running yet.");
        }
    } else if (oldState == IMUState::CALIBRATION && m_is_calibrating_flag) {
        // If we were in the process of m_is_calibrating_flag = true and state changed away from CALIBRATION
        // (e.g., due to an error causing transition to RECOVERY), then we should stop.
        ESP_LOGW(TAG, "Calibration routine was active but IMU state changed away from CALIBRATION to %s. Aborting calibration.", IMUService::stateToString(newState));
        m_is_calibrating_flag = false; // This flag helps the calibrate() loop to terminate.
    }
}

esp_err_t IMUCalibrationService::calibrate() {
    if (m_is_calibrating_flag) {
        ESP_LOGW(TAG, "Calibration already in progress.");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Ensure we are in CALIBRATION state, otherwise it's an invalid call to calibrate()
    // The IMUService should have transitioned to CALIBRATION state upon IMU_CalibrationStarted event.
    if (m_current_imu_state.load(std::memory_order_acquire) != IMUState::CALIBRATION) {
        ESP_LOGE(TAG, "Attempted to calibrate while not in CALIBRATION state (current: %s). Aborting.", 
                 IMUService::stateToString(m_current_imu_state.load()));
        // Publish completion with failure, as something is wrong with state coordination
        m_eventBus.publish(IMU_CalibrationCompleted(ESP_ERR_INVALID_STATE));
        return ESP_ERR_INVALID_STATE;
    }


    if (xSemaphoreTake(m_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Could not acquire calibration mutex (timeout).");
        // Publish failure event? Maybe not here, let caller handle immediate failure.
        return ESP_ERR_TIMEOUT;
    }

    m_is_calibrating_flag = true;
    ESP_LOGI(TAG, "Starting Gyro calibration for %d samples...", m_config.calibration_samples);
    m_eventBus.publish(IMU_CalibrationStarted()); // IMUService will use this to transition to CALIBRATION state

    double gx_sum_dps = 0, gy_sum_dps = 0, gz_sum_dps = 0;
    int successful_samples = 0;
    int attempts = 0;
    const int max_attempts = m_config.calibration_samples + 200; // Allow for some read failures
    m_calib_gx_samples.clear();
    m_calib_gy_samples.clear();
    m_calib_gz_samples.clear();

    float current_scale = calculateGyroScaleFactor();
    if (current_scale <= 0) {
         ESP_LOGE(TAG, "Invalid gyro scale factor (%.1f), cannot calibrate.", current_scale);
         m_is_calibrating_flag = false;
         xSemaphoreGive(m_mutex);
         m_eventBus.publish(IMU_CalibrationCompleted(ESP_FAIL));
         return ESP_FAIL;
    }

    // --- Calibration Loop ---
    while (successful_samples < m_config.calibration_samples && attempts < max_attempts && m_is_calibrating_flag) {
        // m_is_calibrating_flag check allows external abort via state change
        attempts++;
        int16_t raw_gx, raw_gy, raw_gz;

        esp_err_t read_ret = m_driver.readRawGyroXYZ(raw_gx, raw_gy, raw_gz);

        if (read_ret != ESP_OK) {
            ESP_LOGE(TAG, "Gyro read fail during calib attempt %d: %s", attempts, esp_err_to_name(read_ret));
            vTaskDelay(pdMS_TO_TICKS(10)); 
            continue;
        }

        float gx_dps = static_cast<float>(raw_gx) / current_scale;
        float gy_dps = static_cast<float>(raw_gy) / current_scale;
        float gz_dps = static_cast<float>(raw_gz) / current_scale;

        gx_sum_dps += gx_dps;
        gy_sum_dps += gy_dps;
        gz_sum_dps += gz_dps;
        m_calib_gx_samples.push_back(gx_dps);
        m_calib_gy_samples.push_back(gy_dps);
        m_calib_gz_samples.push_back(gz_dps);
        successful_samples++;

        if ((successful_samples % (m_config.calibration_samples / 5) == 0) && successful_samples > 0) {
            ESP_LOGD(TAG, "Calibration progress: %d/%d", successful_samples, m_config.calibration_samples);
        }

        vTaskDelay(pdMS_TO_TICKS(5)); 
    }

    // --- Process Results ---
    esp_err_t final_status = ESP_FAIL; 
    if (!m_is_calibrating_flag) { // Check if aborted
        ESP_LOGW(TAG, "Calibration was aborted externally.");
        final_status = ESP_ERR_INVALID_STATE; // Or a custom error code for aborted
    } else if (successful_samples >= m_config.calibration_samples / 2) { 
        float offset_gx_dps = gx_sum_dps / successful_samples;
        float offset_gy_dps = gy_sum_dps / successful_samples;
        float offset_gz_dps = gz_sum_dps / successful_samples;

        m_gyro_offset_dps[0] = offset_gx_dps;
        m_gyro_offset_dps[1] = offset_gy_dps;
        m_gyro_offset_dps[2] = offset_gz_dps;

        auto calculate_stdev = [&](const std::vector<float>& samples, double mean) {
            if (samples.empty()) return 0.0;
            double sq_sum = std::inner_product(samples.begin(), samples.end(), samples.begin(), 0.0);
            double variance = std::max(0.0, sq_sum / samples.size() - mean * mean); 
            return std::sqrt(variance);
        };
        double stdev_gx = calculate_stdev(m_calib_gx_samples, offset_gx_dps);
        double stdev_gy = calculate_stdev(m_calib_gy_samples, offset_gy_dps);
        double stdev_gz = calculate_stdev(m_calib_gz_samples, offset_gz_dps);

        ESP_LOGI(TAG, "Gyro Calib Complete (%d samples). Offsets(dps): X:%.4f Y:%.4f Z:%.4f | Stdev: X:%.4f Y:%.4f Z:%.4f",
                 successful_samples, offset_gx_dps, offset_gy_dps, offset_gz_dps, stdev_gx, stdev_gy, stdev_gz);

        const float OFFSET_WARN_THRESHOLD = 2.0; 
        const float STDEV_WARN_THRESHOLD = 1.0;  
        if (std::abs(offset_gx_dps) > OFFSET_WARN_THRESHOLD || std::abs(offset_gy_dps) > OFFSET_WARN_THRESHOLD || std::abs(offset_gz_dps) > OFFSET_WARN_THRESHOLD ||
            stdev_gx > STDEV_WARN_THRESHOLD || stdev_gy > STDEV_WARN_THRESHOLD || stdev_gz > STDEV_WARN_THRESHOLD) {
            ESP_LOGW(TAG, "Warning: Gyro offset or standard deviation seems high. Ensure robot was stationary.");
        }
        final_status = ESP_OK; 

        IMU_GyroOffsetsUpdated event(offset_gx_dps, offset_gy_dps, offset_gz_dps);
        m_eventBus.publish(event);
        ESP_LOGI(TAG, "Published gyro offsets updated event.");

    } else {
        ESP_LOGE(TAG, "Insufficient successful samples (%d/%d) for calibration. Offsets not updated.",
                 successful_samples, m_config.calibration_samples);
        final_status = ESP_FAIL; 
    }

    m_is_calibrating_flag = false;
    xSemaphoreGive(m_mutex);
    ESP_LOGI(TAG, "Calibration finished: %s", esp_err_to_name(final_status));

    m_eventBus.publish(IMU_CalibrationCompleted(final_status));
    return final_status;
}

float IMUCalibrationService::getGyroOffsetXDPS() const { return m_gyro_offset_dps[0]; }
float IMUCalibrationService::getGyroOffsetYDPS() const { return m_gyro_offset_dps[1]; }
float IMUCalibrationService::getGyroOffsetZDPS() const { return m_gyro_offset_dps[2]; }

bool IMUCalibrationService::isCalibrating() const {
    return m_is_calibrating_flag;
}

void IMUCalibrationService::setOffsets(float x_offset_dps, float y_offset_dps, float z_offset_dps) {
    if (xSemaphoreTake(m_mutex, portMAX_DELAY) == pdTRUE) {
        m_gyro_offset_dps[0] = x_offset_dps;
        m_gyro_offset_dps[1] = y_offset_dps;
        m_gyro_offset_dps[2] = z_offset_dps;
        xSemaphoreGive(m_mutex);
        ESP_LOGI(TAG, "Gyro offsets set from config: X=%.4f Y=%.4f Z=%.4f DPS",
             x_offset_dps, y_offset_dps, z_offset_dps);
    } else {
        ESP_LOGE(TAG, "Failed to acquire mutex to set gyro offsets.");
    }
}