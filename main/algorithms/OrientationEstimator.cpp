#include "OrientationEstimator.hpp"
#include <cmath>
#include <vector> // No longer needed for FIFO buffer
#include "esp_log.h"
// #include "ConfigData.hpp" // No longer needed here

// Constructor implementation
OrientationEstimator::OrientationEstimator() :
    m_alpha(0.98f),             // Default alpha
    m_sample_period_s(0.001f),  // Default sample period
    m_pitch_deg(0.0f),
    m_yaw_rate_dps(0.0f)
    // m_pitch_rate_dps(0.0f)
{
    // Mutex is default initialized
    ESP_LOGI(TAG, "Orientation Estimator created.");
}

// Initialization method
void OrientationEstimator::init(float alpha, float sample_period_s) {
    std::lock_guard<std::mutex> lock(m_stateMutex);
    m_alpha = alpha;
    m_sample_period_s = sample_period_s;
    m_pitch_deg = 0.0f;
    m_yaw_rate_dps = 0.0f;
    ESP_LOGI(TAG, "Orientation Estimator initialized with alpha=%.3f, Filter dt=%.4fs", m_alpha, m_sample_period_s);
}


void OrientationEstimator::reset() {
    std::lock_guard<std::mutex> lock(m_stateMutex); // Protect state variables
    m_pitch_deg = 0.0f;
    m_yaw_rate_dps = 0.0f;
    ESP_LOGI(TAG, "Orientation Estimator state reset.");
}

// --- MODIFIED: Process a single sample of processed data ---
void OrientationEstimator::processSample(float ax_g, float ay_g, float az_g,
                                        float gx_dps, float gy_dps, float gz_dps)
{
    // Fetch current state (read within lock)
    float current_pitch_deg;
    {
        std::lock_guard<std::mutex> lock(m_stateMutex);
        current_pitch_deg = m_pitch_deg;
    }

    // --- Complementary Filter for Pitch ---
    // Calculate pitch angle from accelerometer
    float accel_pitch_deg = current_pitch_deg; // Default if calculation fails
    float yz_mag_sq = (ay_g * ay_g) + (az_g * az_g);
    if (yz_mag_sq > 1e-6) { // Avoid division by zero / sqrt of negative
        accel_pitch_deg = std::atan2(-ax_g, std::sqrt(yz_mag_sq)) * RAD_TO_DEG;
    } else {
        ESP_LOGV(TAG, "Accel YZ magnitude too small for pitch calculation (ay=%.3f, az=%.3f)", ay_g, az_g);
    }

    // Integrate gyro rate (gy_dps is the pitch rate) and combine with accel angle
    // Ensure sample period is valid
    float dt = m_sample_period_s;
    if (dt <= 0) {
         ESP_LOGW(TAG, "Invalid sample period (%.4f) in estimator, using 0.001s", dt);
         dt = 0.001f;
    }
    float filtered_pitch_deg = m_alpha * (current_pitch_deg + gy_dps * dt) + (1.0f - m_alpha) * accel_pitch_deg;

    // --- Store Yaw Rate ---
    // We just use the provided gyro reading for yaw rate (already offset-corrected)
    float latest_yaw_rate_dps = gz_dps;

    // Store the final updated values (thread-safe write)
    {
        std::lock_guard<std::mutex> lock(m_stateMutex);
        m_pitch_deg = filtered_pitch_deg;
        m_yaw_rate_dps = latest_yaw_rate_dps;
    }

    ESP_LOGV(TAG, "Processed Sample: Acc(%.2f,%.2f,%.2f) Gyro(%.2f,%.2f,%.2f) -> Filt P:%.2f YawR:%.2f",
             ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps, filtered_pitch_deg, latest_yaw_rate_dps);
}

// --- Getters remain the same ---
float OrientationEstimator::getPitchDeg() const {
    std::lock_guard<std::mutex> lock(m_stateMutex);
    return m_pitch_deg;
}

float OrientationEstimator::getYawRateDPS() const {
    std::lock_guard<std::mutex> lock(m_stateMutex);
    return m_yaw_rate_dps;
}