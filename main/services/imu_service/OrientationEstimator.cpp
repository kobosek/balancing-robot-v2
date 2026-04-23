#include "OrientationEstimator.hpp"
#include <cmath>
#include "esp_log.h"

// Constructor implementation
OrientationEstimator::OrientationEstimator() :
    m_alpha(0.98f),
    m_sample_period_s(MPU6050Profile::DEFAULT_SAMPLE_PERIOD_S),
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
    m_alpha.store(alpha, std::memory_order_relaxed);
    m_sample_period_s.store(sample_period_s, std::memory_order_relaxed);
    m_gyro_offset_x_dps.store(gyro_offset_x_dps, std::memory_order_relaxed);
    m_gyro_offset_y_dps.store(gyro_offset_y_dps, std::memory_order_relaxed);
    m_gyro_offset_z_dps.store(gyro_offset_z_dps, std::memory_order_relaxed);
    m_pitch_deg.store(0.0f, std::memory_order_relaxed);
    m_yaw_rate_dps.store(0.0f, std::memory_order_relaxed);
    
    ESP_LOGI(TAG, "Estimator init: Alpha=%.3f, dt=%.4fs, Offsets(X:%.3f Y:%.3f Z:%.3f)",
             alpha, sample_period_s, gyro_offset_x_dps, gyro_offset_y_dps, gyro_offset_z_dps);
}

void OrientationEstimator::updateGyroOffsets(float gyro_offset_x_dps,
                                             float gyro_offset_y_dps,
                                             float gyro_offset_z_dps) {
    m_gyro_offset_x_dps.store(gyro_offset_x_dps, std::memory_order_relaxed);
    m_gyro_offset_y_dps.store(gyro_offset_y_dps, std::memory_order_relaxed);
    m_gyro_offset_z_dps.store(gyro_offset_z_dps, std::memory_order_relaxed);
    
    ESP_LOGI(TAG, "Estimator gyro offsets updated: X:%.3f Y:%.3f Z:%.3f",
             gyro_offset_x_dps, gyro_offset_y_dps, gyro_offset_z_dps);
}

void OrientationEstimator::reset() {
    m_pitch_deg.store(0.0f, std::memory_order_relaxed);
    m_yaw_rate_dps.store(0.0f, std::memory_order_relaxed);
    
    ESP_LOGI(TAG, "Orientation Estimator state (pitch, yaw_rate) reset.");
}

void OrientationEstimator::processSample(float ax_g, float ay_g, float az_g,
                                        float raw_gyro_dps_x, float raw_gyro_dps_y, float raw_gyro_dps_z)
{
    // Suppress unused parameter warning for raw_gyro_dps_x (kept for interface compatibility)
    (void)raw_gyro_dps_x;
    const float current_pitch_deg_local = m_pitch_deg.load(std::memory_order_relaxed);
    const float gy_offset_local = m_gyro_offset_y_dps.load(std::memory_order_relaxed);
    const float gz_offset_local = m_gyro_offset_z_dps.load(std::memory_order_relaxed);
    const float sample_period_s_local = m_sample_period_s.load(std::memory_order_relaxed);
    const float alpha_local = m_alpha.load(std::memory_order_relaxed);

    // Apply gyro offsets (only Y and Z are used in current implementation)
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
         ESP_LOGW(TAG,
                  "Invalid sample period (%.4f) in estimator, using %.4fs",
                  dt,
                  MPU6050Profile::DEFAULT_SAMPLE_PERIOD_S);
         dt = MPU6050Profile::DEFAULT_SAMPLE_PERIOD_S;
     }
    float filtered_pitch_deg = alpha_local * (current_pitch_deg_local + gyro_dps_y * dt) + (1.0f - alpha_local) * accel_pitch_deg;

    // --- Store Yaw Rate ---
    // We use the offset-corrected Z-axis gyro reading for yaw rate
    float latest_yaw_rate_dps = gyro_dps_z;

    m_pitch_deg.store(filtered_pitch_deg, std::memory_order_relaxed);
    m_yaw_rate_dps.store(latest_yaw_rate_dps, std::memory_order_relaxed);

    ESP_LOGV(TAG, "Processed Sample: Acc(%.2f,%.2f,%.2f) RawGyro(%.2f,%.2f,%.2f) -> Filt P:%.2f CalibYawR:%.2f",
             ax_g, ay_g, az_g, raw_gyro_dps_x, raw_gyro_dps_y, raw_gyro_dps_z, filtered_pitch_deg, latest_yaw_rate_dps);
}

float OrientationEstimator::getPitchDeg() const {
    return m_pitch_deg.load(std::memory_order_relaxed);
}

float OrientationEstimator::getYawRateDPS() const {
    return m_yaw_rate_dps.load(std::memory_order_relaxed);
}

std::pair<float, float> OrientationEstimator::getPitchAndYawRate() const {
    return {
        m_pitch_deg.load(std::memory_order_relaxed),
        m_yaw_rate_dps.load(std::memory_order_relaxed)
    };
}
