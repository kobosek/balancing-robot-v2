#pragma once

#include "ConfigData.hpp"               // For MPU6050Config
#include <stdint.h>
#include <cmath>
#include <mutex>
#include "esp_log.h" // Added for logging tag

class OrientationEstimator {
public:
    OrientationEstimator(const MPU6050Config& imuConfig);

    // Method called by IMUService's fifo_task to process a batch of raw FIFO data
    // <<< ADDED gyroOffsetZ_dps >>>
    void processFifoBatch(const uint8_t* fifoBuffer, uint16_t fifoCount,
                          float accelScale_lsb_g, float gyroScale_lsb_dps,
                          float gyroOffsetY_dps, float gyroOffsetZ_dps);

    // Getters (thread-safe)
    float getPitchDeg() const;
    float getYawRateDPS() const; // <<< ADDED Yaw Rate Getter
    // float getPitchRateDPS() const; // Still optional

    void reset(); // Reset filter state

    static constexpr float RAD_TO_DEG = 180.0f / M_PI;
    static constexpr float DEG_TO_RAD = M_PI / 180.0f;
private:
    static constexpr const char* TAG = "OrientationEst";
    static constexpr float SENSOR_SAMPLE_PERIOD_S = 0.001f; // Assuming 1kHz from config

    const float m_alpha; // Complementary filter coefficient for pitch

    // State variables (protected by mutex)
    float m_pitch_deg = 0.0f;
    float m_yaw_rate_dps = 0.0f; // <<< ADDED Yaw Rate State
    // float m_pitch_rate_dps = 0.0f; // Filtered pitch rate (optional)

    mutable std::mutex m_stateMutex;
};