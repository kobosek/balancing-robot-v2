 // main/OrientationEstimator.cpp
#include "OrientationEstimator.hpp"     // Relative path within module's include dir
#include <cmath>
#include <vector>                       // If needing temp storage
#include "esp_log.h"                    // Moved from header
#include "ConfigData.hpp"               // Needed for MPU6050Config in constructor, include explicitly

OrientationEstimator::OrientationEstimator(const MPU6050Config& imuConfig) :
    m_alpha(imuConfig.comp_filter_alpha) // Store alpha from config
{
    reset();
    ESP_LOGI(TAG, "Orientation Estimator initialized with alpha=%.3f, Filter dt=%.4fs", m_alpha, SENSOR_SAMPLE_PERIOD_S);
}

void OrientationEstimator::reset() {
    std::lock_guard<std::mutex> lock(m_stateMutex); // Protect state variables
    m_pitch_deg = 0.0f;
    // m_pitch_rate_dps = 0.0f;
    ESP_LOGI(TAG, "Orientation Estimator state reset.");
}

// Process a batch of raw FIFO data (Accel + Gyro, 12 bytes/sample)
void OrientationEstimator::processFifoBatch(const uint8_t* fifoBuffer, uint16_t fifoCount,
                                            float accelScale_lsb_g, float gyroScale_lsb_dps,
                                            float gyroOffsetY_dps)
{
    const int bytes_per_sample = 12; // 6 accel + 6 gyro

    // Calculate number of full samples available based on bytes read
    uint16_t num_samples = fifoCount / bytes_per_sample;

    if (num_samples == 0) {
        ESP_LOGV(TAG, "Received %d bytes, not enough for a full sample.", fifoCount);
        return; // Not enough data for even one full sample
    }
    // Log if partial sample data was received (fifoCount not multiple of bytes_per_sample)
    if (fifoCount % bytes_per_sample != 0) {
        ESP_LOGW(TAG, "Received %d bytes, which is not a multiple of sample size (%d). Processing %d full samples.", fifoCount, bytes_per_sample, num_samples);
    } else {
        ESP_LOGV(TAG, "Processing %d samples from FIFO batch (%d bytes)", num_samples, fifoCount);
    }

    float current_pitch_deg;
    {
        std::lock_guard<std::mutex> lock(m_stateMutex);
        current_pitch_deg = m_pitch_deg;
    }

    // Loop through each sample in the batch
    for (uint16_t i = 0; i < num_samples; ++i) {
        int offset = i * bytes_per_sample;

        // --- Parse Raw Data (Big Endian) ---
        int16_t ax_raw = (fifoBuffer[offset + 0] << 8) | fifoBuffer[offset + 1];
        int16_t ay_raw = (fifoBuffer[offset + 2] << 8) | fifoBuffer[offset + 3];
        int16_t az_raw = (fifoBuffer[offset + 4] << 8) | fifoBuffer[offset + 5];
        //int16_t gx_raw = (fifoBuffer[offset + 6] << 8) | fifoBuffer[offset + 7]; // Roll rate raw
        int16_t gy_raw = (fifoBuffer[offset + 8] << 8) | fifoBuffer[offset + 9]; // Pitch rate raw
        // int16_t gz_raw = (fifoBuffer[offset + 10] << 8) | fifoBuffer[offset + 11]; // Yaw rate raw

        // --- Convert to Physical Units (g's and dps) ---
        float ax_g = static_cast<float>(ax_raw) / accelScale_lsb_g;
        float ay_g = static_cast<float>(ay_raw) / accelScale_lsb_g;
        float az_g = static_cast<float>(az_raw) / accelScale_lsb_g;
        // Gyro in Degrees Per Second (dps), applying offset
        //float gx_dps = (static_cast<float>(gx_raw) / gyroScale_lsb_dps); 
        float gy_dps = (static_cast<float>(gy_raw) / gyroScale_lsb_dps) - gyroOffsetY_dps; // Apply Y offset here
        // float gz_dps = (static_cast<float>(gz_raw) / gyroScale_lsb_dps); // Apply offset if needed

        float accel_pitch_deg = current_pitch_deg;
        float ay_sq = ay_g * ay_g;
        float az_sq = az_g * az_g;

        // Pitch calculation using atan2(-Ax, sqrt(Ay^2 + Az^2))
        float yz_mag_sq = ay_sq + az_sq;
        if (yz_mag_sq > 1e-6) {
           accel_pitch_deg = std::atan2(-ax_g, std::sqrt(yz_mag_sq)) * RAD_TO_DEG;
        }

        // Integrate gyro rate (dps * sample_period_s) -> angle change (degrees)
        current_pitch_deg = m_alpha * (current_pitch_deg + gy_dps * SENSOR_SAMPLE_PERIOD_S) + (1.0f - m_alpha) * accel_pitch_deg;
    }
    // Store the final updated values (thread-safe)
    {
        std::lock_guard<std::mutex> lock(m_stateMutex);
        m_pitch_deg = current_pitch_deg;
    }

    ESP_LOGV(TAG, "Batch processed %d samples. Final Filt P:%.2f",
                 num_samples, current_pitch_deg);
}

float OrientationEstimator::getPitchDeg() const {
    std::lock_guard<std::mutex> lock(m_stateMutex);
    return m_pitch_deg;
}