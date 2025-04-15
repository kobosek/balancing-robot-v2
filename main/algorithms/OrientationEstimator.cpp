#include "OrientationEstimator.hpp"
#include <cmath>
#include <vector>
#include "esp_log.h"
#include "ConfigData.hpp" // Needed for MPU6050Config in constructor

OrientationEstimator::OrientationEstimator(const MPU6050Config& imuConfig) :
    m_alpha(imuConfig.comp_filter_alpha) // Store alpha from config
{
    reset();
    ESP_LOGI(TAG, "Orientation Estimator initialized with alpha=%.3f, Filter dt=%.4fs", m_alpha, SENSOR_SAMPLE_PERIOD_S);
}

void OrientationEstimator::reset() {
    std::lock_guard<std::mutex> lock(m_stateMutex); // Protect state variables
    m_pitch_deg = 0.0f;
    m_yaw_rate_dps = 0.0f; // <<< Reset Yaw Rate
    ESP_LOGI(TAG, "Orientation Estimator state reset.");
}

// <<< MODIFIED: Added gyroOffsetZ_dps parameter >>>
void OrientationEstimator::processFifoBatch(const uint8_t* fifoBuffer, uint16_t fifoCount,
                                            float accelScale_lsb_g, float gyroScale_lsb_dps,
                                            float gyroOffsetY_dps, float gyroOffsetZ_dps)
{
    // FIFO structure offsets (when GYRO_ACCEL is enabled)
    static constexpr int ACCEL_X_OFFSET = 0;
    static constexpr int ACCEL_Y_OFFSET = 2;
    static constexpr int ACCEL_Z_OFFSET = 4;
    static constexpr int GYRO_X_OFFSET = 6;
    static constexpr int GYRO_Y_OFFSET = 8;
    static constexpr int GYRO_Z_OFFSET = 10;
    
    const int bytes_per_sample = 12; // 6 accel + 6 gyro
    uint16_t num_samples = fifoCount / bytes_per_sample;

    if (num_samples == 0) {
        ESP_LOGV(TAG, "Received %d bytes, not enough for a full sample.", fifoCount);
        return;
    }
    if (fifoCount % bytes_per_sample != 0) {
        ESP_LOGW(TAG, "Received %d bytes, not multiple of sample size (%d). Processing %d samples.", fifoCount, bytes_per_sample, num_samples);
    } else {
        ESP_LOGV(TAG, "Processing %d samples from FIFO batch (%d bytes)", num_samples, fifoCount);
    }

    // Fetch current state outside the loop (reduces lock contention)
    float current_pitch_deg;
    float latest_yaw_rate_dps = 0; // Will store the rate from the *last* sample in the batch
    {
        std::lock_guard<std::mutex> lock(m_stateMutex);
        current_pitch_deg = m_pitch_deg;
        // Don't need to read m_yaw_rate_dps here as it's just overwritten
    }

    // Loop through each sample in the batch
    for (uint16_t i = 0; i < num_samples; ++i) {
        int offset = i * bytes_per_sample;

        // --- Parse Raw Data (Big Endian) ---
        int16_t ax_raw = (fifoBuffer[offset + ACCEL_X_OFFSET] << 8) | fifoBuffer[offset + ACCEL_X_OFFSET + 1];
        int16_t ay_raw = (fifoBuffer[offset + ACCEL_Y_OFFSET] << 8) | fifoBuffer[offset + ACCEL_Y_OFFSET + 1];
        int16_t az_raw = (fifoBuffer[offset + ACCEL_Z_OFFSET] << 8) | fifoBuffer[offset + ACCEL_Z_OFFSET + 1];
        int16_t gx_raw = (fifoBuffer[offset + GYRO_X_OFFSET] << 8) | fifoBuffer[offset + GYRO_X_OFFSET + 1]; // Roll rate raw
        int16_t gy_raw = (fifoBuffer[offset + GYRO_Y_OFFSET] << 8) | fifoBuffer[offset + GYRO_Y_OFFSET + 1]; // Pitch rate raw
        int16_t gz_raw = (fifoBuffer[offset + GYRO_Z_OFFSET] << 8) | fifoBuffer[offset + GYRO_Z_OFFSET + 1]; // Yaw rate raw

        // --- Convert to Physical Units (g's and dps) ---
        float ax_g = static_cast<float>(ax_raw) / accelScale_lsb_g;
        float ay_g = static_cast<float>(ay_raw) / accelScale_lsb_g;
        float az_g = static_cast<float>(az_raw) / accelScale_lsb_g;
        // Gyro in Degrees Per Second (dps), applying offsets
        float gx_dps = static_cast<float>(gx_raw) / gyroScale_lsb_dps; // No offset for X currently
        float gy_dps = (static_cast<float>(gy_raw) / gyroScale_lsb_dps) - gyroOffsetY_dps; // Apply Y offset
        float gz_dps = (static_cast<float>(gz_raw) / gyroScale_lsb_dps) - gyroOffsetZ_dps; // Apply Z offset

        // --- Complementary Filter for Pitch ---
        float accel_pitch_deg = current_pitch_deg; // Default if calculation fails
        float yz_mag_sq = (ay_g * ay_g) + (az_g * az_g);
        if (yz_mag_sq > 1e-6) { // Avoid division by zero / sqrt of negative
           accel_pitch_deg = std::atan2(-ax_g, std::sqrt(yz_mag_sq)) * RAD_TO_DEG;
        }
        // Integrate gyro rate and combine with accel angle
        current_pitch_deg = m_alpha * (current_pitch_deg + gy_dps * SENSOR_SAMPLE_PERIOD_S) + (1.0f - m_alpha) * accel_pitch_deg;

        // --- Store Yaw Rate ---
        // We just use the latest gyro reading for yaw rate, no complex filtering for now
        latest_yaw_rate_dps = gz_dps;

    } // End loop through samples

    // Store the final updated values (thread-safe)
    {
        std::lock_guard<std::mutex> lock(m_stateMutex);
        m_pitch_deg = current_pitch_deg;
        m_yaw_rate_dps = latest_yaw_rate_dps; // <<< Store Yaw Rate
    }

    ESP_LOGV(TAG, "Batch processed %d samples. Final Filt P:%.2f, YawRate: %.2f",
                 num_samples, current_pitch_deg, latest_yaw_rate_dps);
}

float OrientationEstimator::getPitchDeg() const {
    std::lock_guard<std::mutex> lock(m_stateMutex);
    return m_pitch_deg;
}

// <<< ADDED Yaw Rate Getter >>>
float OrientationEstimator::getYawRateDPS() const {
    std::lock_guard<std::mutex> lock(m_stateMutex);
    return m_yaw_rate_dps;
}