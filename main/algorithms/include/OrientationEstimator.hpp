#pragma once

#include <stdint.h>
#include <cmath>
#include <mutex>
#include "esp_log.h"

#ifndef M_PI // Define if not used elsewhere that includes cmath indirectly
#define M_PI 3.14159265358979323846
#endif

class OrientationEstimator {
public:
    // Constructor no longer needs MPU6050Config
    OrientationEstimator();

    // Initialize with filter parameters
    void init(float alpha = 0.98f, float sample_period_s = 0.001f);

    // Method called by IMUService to process ONE sample of processed physical data
    void processSample(float accel_g_x, float accel_g_y, float accel_g_z,
                       float gyro_dps_x, float gyro_dps_y, float gyro_dps_z); // Changed signature

    // Getters (thread-safe)
    float getPitchDeg() const;
    float getYawRateDPS() const;
    // float getPitchRateDPS() const; // Still optional

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
    // float m_pitch_rate_dps = 0.0f; // Filtered pitch rate (optional)

    mutable std::mutex m_stateMutex;
};