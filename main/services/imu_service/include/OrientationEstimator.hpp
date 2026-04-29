#pragma once

#include "IIMUDataSink.hpp"
#include "MPU6050Profile.hpp"
#include <stdint.h>
#include <atomic>
#include <cmath>
#include <utility>
#include "esp_log.h"

// Using a standard way to define M_PI if not available, or prefer std::numbers::pi in C++20
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct OrientationEstimate {
    float pitch_deg = 0.0f;
    float pitch_rate_dps = 0.0f;
    float yaw_deg = 0.0f;
    float yaw_rate_dps = 0.0f;
};

class OrientationEstimator : public IIMUDataSink {
public:
    OrientationEstimator();

    // Initialize with filter parameters and initial gyro offsets
    void init(float alpha = 0.98f, float sample_period_s = MPU6050Profile::DEFAULT_SAMPLE_PERIOD_S,
              float gyro_offset_x_dps = 0.0f,
              float gyro_offset_y_dps = 0.0f,
              float gyro_offset_z_dps = 0.0f);

    // Method to update gyro offsets at runtime
    void updateGyroOffsets(float gyro_offset_x_dps,
                           float gyro_offset_y_dps,
                           float gyro_offset_z_dps);

    // Process sample now takes raw (but scaled) gyro data
    void processSample(float accel_g_x, float accel_g_y, float accel_g_z,
                       float raw_gyro_dps_x, float raw_gyro_dps_y, float raw_gyro_dps_z) override;

    // Getters (thread-safe)
    float getPitchDeg() const;
    float getPitchRateDPS() const;
    float getYawDeg() const;
    float getYawRateDPS() const;
    
    // Get all controller-facing IMU values through one read path.
    OrientationEstimate getOrientation() const;

    // Compatibility helper for existing consumers that only need pitch/yaw.
    std::pair<float, float> getPitchAndYawRate() const;

    void reset(); // Reset filter state

    static constexpr float RAD_TO_DEG = 180.0f / M_PI;
    static constexpr float DEG_TO_RAD = M_PI / 180.0f;

private:
    static constexpr const char* TAG = "OrientationEst";

    std::atomic<float> m_alpha; // Legacy accel trust knob from the old complementary filter config.
    std::atomic<float> m_sample_period_s; // Sample period

    // State variables updated from the IMU task and read by the control task.
    std::atomic<float> m_pitch_deg;
    std::atomic<float> m_pitch_rate_dps;
    std::atomic<float> m_yaw_deg;
    std::atomic<float> m_yaw_rate_dps;
    std::atomic<float> m_pitch_bias_dps;
    std::atomic<float> m_p00;
    std::atomic<float> m_p01;
    std::atomic<float> m_p10;
    std::atomic<float> m_p11;
    std::atomic<bool> m_has_estimate;

    // Gyro offsets are configuration values updated infrequently at runtime.
    std::atomic<float> m_gyro_offset_x_dps;
    std::atomic<float> m_gyro_offset_y_dps;
    std::atomic<float> m_gyro_offset_z_dps;

    static constexpr float KALMAN_PROCESS_NOISE_ANGLE = 0.005f;
    static constexpr float KALMAN_PROCESS_NOISE_BIAS = 0.003f;
    static constexpr float KALMAN_BASE_ACCEL_NOISE_DEG2 = 0.02f;
    static constexpr float ACCEL_TRUST_DEADBAND_G = 0.08f;
    static constexpr float ACCEL_REJECTION_THRESHOLD_G = 0.35f;
    static constexpr float MIN_ACCEL_NOISE_DEG2 = 0.02f;
    static constexpr float MAX_ACCEL_NOISE_SCALE = 200.0f;
};
