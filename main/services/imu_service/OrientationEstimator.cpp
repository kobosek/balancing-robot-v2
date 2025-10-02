#include "OrientationEstimator.hpp"
#include <cmath>
#include <tuple>
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
    safeExecuteVoid([&]() {
        m_alpha = alpha;
        m_sample_period_s = sample_period_s;
        m_gyro_offset_x_dps = gyro_offset_x_dps;
        m_gyro_offset_y_dps = gyro_offset_y_dps;
        m_gyro_offset_z_dps = gyro_offset_z_dps;
        m_pitch_deg = 0.0f; // Reset state on init
        m_yaw_rate_dps = 0.0f; // Reset state on init
    });
    
    ESP_LOGI(TAG, "Estimator init: Alpha=%.3f, dt=%.4fs, Offsets(X:%.3f Y:%.3f Z:%.3f)",
             alpha, sample_period_s, gyro_offset_x_dps, gyro_offset_y_dps, gyro_offset_z_dps);
}

void OrientationEstimator::updateGyroOffsets(float gyro_offset_x_dps,
                                             float gyro_offset_y_dps,
                                             float gyro_offset_z_dps) {
    safeExecuteVoid([&]() {
        m_gyro_offset_x_dps = gyro_offset_x_dps;
        m_gyro_offset_y_dps = gyro_offset_y_dps;
        m_gyro_offset_z_dps = gyro_offset_z_dps;
    });
    
    ESP_LOGI(TAG, "Estimator gyro offsets updated: X:%.3f Y:%.3f Z:%.3f",
             gyro_offset_x_dps, gyro_offset_y_dps, gyro_offset_z_dps);
}

void OrientationEstimator::reset() {
    safeExecuteVoid([this]() {
        m_pitch_deg = 0.0f;
        m_yaw_rate_dps = 0.0f;
        // Gyro offsets are not reset by this method, they are part of configuration.
        // Re-call init() if offsets need to be reset to defaults or reloaded.
    });
    
    ESP_LOGI(TAG, "Orientation Estimator state (pitch, yaw_rate) reset.");
}

void OrientationEstimator::processSample(float ax_g, float ay_g, float az_g,
                                        float raw_gyro_dps_x, float raw_gyro_dps_y, float raw_gyro_dps_z)
{
    // Suppress unused parameter warning for raw_gyro_dps_x (kept for interface compatibility)
    (void)raw_gyro_dps_x;
    // Fetch current state and parameters under lock with exception safety
    auto state_tuple = safeExecute([this]() {
        return std::make_tuple(m_pitch_deg, m_gyro_offset_y_dps, 
                              m_gyro_offset_z_dps, m_sample_period_s, m_alpha);
    });
    
    float current_pitch_deg_local = std::get<0>(state_tuple);
    float gy_offset_local = std::get<1>(state_tuple);
    float gz_offset_local = std::get<2>(state_tuple);
    float sample_period_s_local = std::get<3>(state_tuple);
    float alpha_local = std::get<4>(state_tuple);

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
         ESP_LOGW(TAG, "Invalid sample period (%.4f) in estimator, using 0.001s", dt);
         dt = 0.001f;
    }
    float filtered_pitch_deg = alpha_local * (current_pitch_deg_local + gyro_dps_y * dt) + (1.0f - alpha_local) * accel_pitch_deg;

    // --- Store Yaw Rate ---
    // We use the offset-corrected Z-axis gyro reading for yaw rate
    float latest_yaw_rate_dps = gyro_dps_z;

    // Store the final updated values atomically (thread-safe write)
    safeExecuteVoid([&]() {
        m_pitch_deg = filtered_pitch_deg;
        m_yaw_rate_dps = latest_yaw_rate_dps;
    });

    ESP_LOGV(TAG, "Processed Sample: Acc(%.2f,%.2f,%.2f) RawGyro(%.2f,%.2f,%.2f) -> Filt P:%.2f CalibYawR:%.2f",
             ax_g, ay_g, az_g, raw_gyro_dps_x, raw_gyro_dps_y, raw_gyro_dps_z, filtered_pitch_deg, latest_yaw_rate_dps);
}

float OrientationEstimator::getPitchDeg() const {
    return safeExecute([this]() { return m_pitch_deg; });
}

float OrientationEstimator::getYawRateDPS() const {
    return safeExecute([this]() { return m_yaw_rate_dps; });
}

std::pair<float, float> OrientationEstimator::getPitchAndYawRate() const {
    return safeExecute([this]() {
        return std::make_pair(m_pitch_deg, m_yaw_rate_dps);
    });
}

// Thread-safe mutex operations implementation (ESP32 compatible - no exceptions)
template<typename Func>
auto OrientationEstimator::safeExecute(Func&& func) const -> decltype(func()) {
    std::lock_guard<std::mutex> lock(m_stateMutex);
    return func();
}

template<typename Func>
void OrientationEstimator::safeExecuteVoid(Func&& func) const {
    std::lock_guard<std::mutex> lock(m_stateMutex);
    func();
}