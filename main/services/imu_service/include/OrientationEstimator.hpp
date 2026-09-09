#pragma once

#include "IIMUDataSink.hpp"
#include "IMUDataTypes.hpp"
#include "freertos/FreeRTOS.h"
#include "MPU6050Profile.hpp"
#include <stdint.h>
#include <cmath>
#include <utility>
#include "esp_log.h"

// Using a standard way to define M_PI if not available, or prefer std::numbers::pi in C++20
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


class OrientationEstimator : public IIMUDataSink {
public:
    OrientationEstimator();

    // Initialize with filter parameters and initial gyro offsets
    void init(float alpha = 0.98f, float sample_period_s = MPU6050Profile::DEFAULT_SAMPLE_PERIOD_S,
              float gyro_offset_x_dps = 0.0f,
              float gyro_offset_y_dps = 0.0f,
              float gyro_offset_z_dps = 0.0f);

    // Process sample now takes raw (but scaled) gyro data
    bool processSample(float accel_g_x, float accel_g_y, float accel_g_z,
                       float raw_gyro_dps_x, float raw_gyro_dps_y, float raw_gyro_dps_z,
                       int64_t sampleTimestampUs, uint32_t generation,
                       const IMUSampleMetadata& metadata = {}) override;
    void recordFifoLoss(uint64_t knownDiscarded, bool uncertain) override;

    // Getters (thread-safe)
    float getPitchDeg() const;
    float getPitchRateDPS() const;
    float getYawDeg() const;
    float getYawRateDPS() const;
    
    // Get all controller-facing IMU values through one read path.
    OrientationEstimate getOrientation() const;

    // Compatibility helper for existing consumers that only need pitch/yaw.
    std::pair<float, float> getPitchAndYawRate() const;

    bool setValidated(); // Owner only; requires five consecutive usable initialization samples.
    void reset(); // Reset filter state

    static constexpr float RAD_TO_DEG = 180.0f / M_PI;
    static constexpr float DEG_TO_RAD = M_PI / 180.0f;

private:
    mutable portMUX_TYPE m_snapshotMux = portMUX_INITIALIZER_UNLOCKED;
    OrientationEstimate m_snapshot;
    static constexpr const char* TAG = "OrientationEst";

    float m_sample_period_s; // Sample period

    // Private filter state: exclusively owned by IMUTask. Readers only copy m_snapshot.
    float m_pitch_deg;
    double m_yaw_deg;
    float m_pitch_bias_dps;
    float m_p00;
    float m_p01;
    float m_p10;
    float m_p11;
    bool m_has_estimate;
    unsigned m_initializationSamples = 0;
    bool m_gyroContinuityLost = false;
    float m_accelNoise = 0.98f;

    // Gyro offsets are configuration values updated infrequently at runtime.
    float m_gyro_offset_x_dps;
    float m_gyro_offset_y_dps;
    float m_gyro_offset_z_dps;

    static constexpr float KALMAN_PROCESS_NOISE_ANGLE = 0.005f;
    static constexpr float KALMAN_PROCESS_NOISE_BIAS = 0.003f;
    static constexpr float KALMAN_BASE_ACCEL_NOISE_DEG2 = 0.02f;
    static constexpr float ACCEL_TRUST_DEADBAND_G = 0.08f;
    static constexpr float ACCEL_REJECTION_THRESHOLD_G = 0.35f;
    static constexpr float MIN_ACCEL_NOISE_DEG2 = 0.02f;
    static constexpr float MAX_ACCEL_NOISE_SCALE = 200.0f;
};
