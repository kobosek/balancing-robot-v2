// main/include/OrientationEstimator.hpp
#pragma once

#include "ConfigData.hpp"               // Found via INCLUDE_DIRS (needed for MPU6050Config)
#include <stdint.h>                     // For uint8_t, uint16_t
#include <cmath>                        // For atan2, sqrt, M_PI
#include <mutex>                        // For thread safety
class OrientationEstimator {
public:
    OrientationEstimator(const MPU6050Config& imuConfig);

    // Method called by IMUService's fifo_task to process a batch of raw FIFO data
    // Takes raw scales from sensor driver (LSB/g, LSB/dps) and calibrated gyro offset (dps)
    void processFifoBatch(const uint8_t* fifoBuffer, uint16_t fifoCount,
                          float accelScale_lsb_g, float gyroScale_lsb_dps,
                          float gyroOffsetY_dps);

    // Getters for estimated orientation (thread-safe) - Returns DEGREES
    float getPitchDeg() const;
    // float getPitchRateDPS() const; // Optional: return filtered rate if needed

    void reset(); // Reset filter state
    static constexpr float RAD_TO_DEG = 180.0f / M_PI;
    static constexpr float DEG_TO_RAD = M_PI / 180.0f;
private:
    static constexpr const char* TAG = "OrientationEst";
    // Define conversion factors here if needed frequently

    // Sensor sample period for integration (assuming 1kHz)
    static constexpr float SENSOR_SAMPLE_PERIOD_S = 0.001f;

    const float m_alpha; // Complementary filter coefficient

    float m_pitch_deg = 0.0f;
    mutable std::mutex m_stateMutex;
};