#pragma once

#include <stdint.h>
#include <cmath>
#include <mutex>
#include "esp_log.h"

// Using a standard way to define M_PI if not available, or prefer std::numbers::pi in C++20
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class OrientationEstimator {
public:
    OrientationEstimator();

    // Initialize with filter parameters and initial gyro offsets
    void init(float alpha = 0.98f, float sample_period_s = 0.001f,
              float gyro_offset_x_dps = 0.0f,
              float gyro_offset_y_dps = 0.0f,
              float gyro_offset_z_dps = 0.0f);

    // Method to update gyro offsets at runtime
    void updateGyroOffsets(float gyro_offset_x_dps,
                           float gyro_offset_y_dps,
                           float gyro_offset_z_dps);

    // Process sample now takes raw (but scaled) gyro data
    void processSample(float accel_g_x, float accel_g_y, float accel_g_z,
                       float raw_gyro_dps_x, float raw_gyro_dps_y, float raw_gyro_dps_z);

    // Getters (thread-safe)
    float getPitchDeg() const;
    float getYawRateDPS() const;
    
    // Get both values atomically to avoid inconsistent reads
    std::pair<float, float> getPitchAndYawRate() const;

    void reset(); // Reset filter state

    static constexpr float RAD_TO_DEG = 180.0f / M_PI;
    static constexpr float DEG_TO_RAD = M_PI / 180.0f;

private:
    static constexpr const char* TAG = "OrientationEst";

    float m_alpha; // Complementary filter coefficient for pitch
    float m_sample_period_s; // Sample period

    // State variables (protected by mutex)
    float m_pitch_deg;
    float m_yaw_rate_dps;

    // Gyro offsets (protected by mutex, applied internally)
    float m_gyro_offset_x_dps;
    float m_gyro_offset_y_dps;
    float m_gyro_offset_z_dps;

    mutable std::mutex m_stateMutex;
    
    // Thread-safe mutex operations (ESP32 compatible - no exceptions)
    template<typename Func>
    auto safeExecute(Func&& func) const -> decltype(func());
    
    template<typename Func>
    void safeExecuteVoid(Func&& func) const;
};