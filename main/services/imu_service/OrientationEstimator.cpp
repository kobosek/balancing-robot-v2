#include "OrientationEstimator.hpp"
#include <cmath>
// #include <vector> // No longer needed
#include "esp_log.h"

// Constructor implementation
OrientationEstimator::OrientationEstimator() :
    m_alpha(0.98f),
    m_sample_period_s(0.001f),
    m_pitch_deg(0.0f),
    m_yaw_rate_dps(0.0f),
    m_gyro_offset_x_dps(0.0f),
    m_gyro_offset_y_dps(0.0f),
    m_gyro_offset_z_dps(0.0f)
{
    ESP_LOGI(TAG, "Orientation Estimator created.");
}

// Initialization method
void OrientationEstimator::init(float alpha, float sample_period_s,
                               float gyro_offset_x_dps,
                               float gyro_offset_y_dps,
                               float gyro_offset_z_dps) {
    std::lock_guard<std::mutex> lock(m_stateMutex);
    m_alpha = alpha;
    m_sample_period_s = sample_period_s;
    m_gyro_offset_x_dps = gyro_offset_x_dps;
    m_gyro_offset_y_dps = gyro_offset_y_dps;
    m_gyro_offset_z_dps = gyro_offset_z_dps;
    m_pitch_deg = 0.0f; // Reset state on init
    m_yaw_rate_dps = 0.0f; // Reset state on init
    ESP_LOGI(TAG, "Estimator init: Alpha=%.3f, dt=%.4fs, Offsets(X:%.3f Y:%.3f Z:%.3f)",
             m_alpha, m_sample_period_s,
             m_gyro_offset_x_dps, m_gyro_offset_y_dps, m_gyro_offset_z_dps);
}

void OrientationEstimator::updateGyroOffsets(float gyro_offset_x_dps,
                                             float gyro_offset_y_dps,
                                             float gyro_offset_z_dps) {
    std::lock_guard<std::mutex> lock(m_stateMutex);
    m_gyro_offset_x_dps = gyro_offset_x_dps;
    m_gyro_offset_y_dps = gyro_offset_y_dps;
    m_gyro_offset_z_dps = gyro_offset_z_dps;
    ESP_LOGI(TAG, "Estimator gyro offsets updated: X:%.3f Y:%.3f Z:%.3f",
             m_gyro_offset_x_dps, m_gyro_offset_y_dps, m_gyro_offset_z_dps);
}

void OrientationEstimator::reset() {
    std::lock_guard<std::mutex> lock(m_stateMutex); // Protect state variables
    m_pitch_deg = 0.0f;
    m_yaw_rate_dps = 0.0f;
    // Gyro offsets are not reset by this method, they are part of configuration.
    // Re-call init() if offsets need to be reset to defaults or reloaded.
    ESP_LOGI(TAG, "Orientation Estimator state (pitch, yaw_rate) reset.");
}

void OrientationEstimator::processSample(float ax_g, float ay_g, float az_g,
                                        float raw_gyro_dps_x, float raw_gyro_dps_y, float raw_gyro_dps_z)
{
    float current_pitch_deg_local;
    float gx_offset_local, gy_offset_local, gz_offset_local;
    float sample_period_s_local;
    float alpha_local;

    // Fetch current state and parameters under lock
    {
        std::lock_guard<std::mutex> lock(m_stateMutex);
        current_pitch_deg_local = m_pitch_deg;
        gx_offset_local = m_gyro_offset_x_dps;
        gy_offset_local = m_gyro_offset_y_dps;
        gz_offset_local = m_gyro_offset_z_dps;
        sample_period_s_local = m_sample_period_s;
        alpha_local = m_alpha;
    }

    // Apply gyro offsets
    float gyro_dps_x = raw_gyro_dps_x - gx_offset_local;
    float gyro_dps_y = raw_gyro_dps_y - gy_offset_local;
    float gyro_dps_z = raw_gyro_dps_z - gz_offset_local;

    // --- Complementary Filter for Pitch ---
    float accel_pitch_deg = current_pitch_deg_local; // Default if calculation fails
    float yz_mag_sq = (ay_g * ay_g) + (az_g * az_g);
    if (yz_mag_sq > 1e-6) {
        accel_pitch_deg = std::atan2(-ax_g, std::sqrt(yz_mag_sq)) * RAD_TO_DEG;
    } else {
        ESP_LOGV(TAG, "Accel YZ magnitude too small for pitch calculation (ay=%.3f, az=%.3f)", ay_g, az_g);
    }

    float dt = sample_period_s_local;
    if (dt <= 0) {
         ESP_LOGW(TAG, "Invalid sample period (%.4f) in estimator, using 0.001s", dt);
         dt = 0.001f;
    }
    float filtered_pitch_deg = alpha_local * (current_pitch_deg_local + gyro_dps_y * dt) + (1.0f - alpha_local) * accel_pitch_deg;

    // --- Store Yaw Rate ---
    // We use the offset-corrected Z-axis gyro reading for yaw rate
    float latest_yaw_rate_dps = gyro_dps_z;

    // Store the final updated values (thread-safe write)
    {
        std::lock_guard<std::mutex> lock(m_stateMutex);
        m_pitch_deg = filtered_pitch_deg;
        m_yaw_rate_dps = latest_yaw_rate_dps;
    }

    ESP_LOGV(TAG, "Processed Sample: Acc(%.2f,%.2f,%.2f) RawGyro(%.2f,%.2f,%.2f) -> Filt P:%.2f CalibYawR:%.2f",
             ax_g, ay_g, az_g, raw_gyro_dps_x, raw_gyro_dps_y, raw_gyro_dps_z, filtered_pitch_deg, latest_yaw_rate_dps);
}

float OrientationEstimator::getPitchDeg() const {
    std::lock_guard<std::mutex> lock(m_stateMutex);
    return m_pitch_deg;
}

float OrientationEstimator::getYawRateDPS() const {
    std::lock_guard<std::mutex> lock(m_stateMutex);
    return m_yaw_rate_dps;
}