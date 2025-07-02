#include "IMUCalibration.hpp"
#include "mpu6050.hpp"
#include <numeric>
#include <cmath>
#include <algorithm>

IMUCalibration::IMUCalibration(MPU6050Driver& driver) : 
    m_driver(driver)
{
    // Reserve reasonable space for samples
    m_calib_gx_samples.reserve(100);
    m_calib_gy_samples.reserve(100);
    m_calib_gz_samples.reserve(100);
}

float IMUCalibration::calculateGyroScaleFactor(const MPU6050Config& config) {
    float scale = 1.0f; // Default
    switch(config.gyro_range) {
        case 0: scale = 131.0f; break;
        case 1: scale = 65.5f;  break;
        case 2: scale = 32.8f;  break;
        case 3: scale = 16.4f;  break;
        default: ESP_LOGW(TAG, "Unknown gyro range %d in config, using default scale!", config.gyro_range); scale = 65.5f; break;
    }
    ESP_LOGD(TAG, "Gyro scale factor calculated: %.1f LSB/DPS for range %d", scale, config.gyro_range);
    return scale;
}

void IMUCalibration::setOffsets(float x_offset_dps, float y_offset_dps, float z_offset_dps) {
    m_gyro_offset_dps[0] = x_offset_dps;
    m_gyro_offset_dps[1] = y_offset_dps;
    m_gyro_offset_dps[2] = z_offset_dps;
    ESP_LOGI(TAG, "Gyro offsets set to: X:%.4f Y:%.4f Z:%.4f dps", 
             x_offset_dps, y_offset_dps, z_offset_dps);
}

esp_err_t IMUCalibration::calibrate(const MPU6050Config& config, 
                                  std::function<void(int, int)> progressCallback) {
    ESP_LOGI(TAG, "Starting Gyro calibration for %d samples...", config.calibration_samples);
    
    double gx_sum_dps = 0, gy_sum_dps = 0, gz_sum_dps = 0;
    int successful_samples = 0;
    int attempts = 0;
    const int max_attempts = config.calibration_samples + 200; // Allow for some read failures
    
    m_calib_gx_samples.clear();
    m_calib_gy_samples.clear();
    m_calib_gz_samples.clear();

    float current_scale = calculateGyroScaleFactor(config);
    if (current_scale <= 0) {
         ESP_LOGE(TAG, "Invalid gyro scale factor (%.1f), cannot calibrate.", current_scale);
         return ESP_FAIL;
    }

    // --- Calibration Loop ---
    while (successful_samples < config.calibration_samples && attempts < max_attempts) {
        attempts++;
        int16_t raw_gx, raw_gy, raw_gz;

        esp_err_t read_ret = m_driver.readRawGyroXYZ(raw_gx, raw_gy, raw_gz);

        if (read_ret != ESP_OK) {
            ESP_LOGE(TAG, "Gyro read fail during calib attempt %d: %s", attempts, esp_err_to_name(read_ret));
            // Error handling delay now managed by the caller
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

        if (progressCallback && ((successful_samples % (config.calibration_samples / 5) == 0) && successful_samples > 0)) {
            progressCallback(successful_samples, config.calibration_samples);
        }

        // No delay here - parent context can control the sampling rate
    }

    // --- Process Results ---
    if (successful_samples >= config.calibration_samples / 2) { 
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

        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Not enough successful samples collected (%d/%d).", successful_samples, config.calibration_samples);
        return ESP_FAIL;
    }
}
