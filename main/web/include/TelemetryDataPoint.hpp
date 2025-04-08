// main/web/include/TelemetryDataPoint.hpp
#pragma once

#include <stdint.h> // For int64_t

// Define TelemetryDataPoint structure
struct TelemetryDataPoint {
    int64_t timestamp_us = 0;
    float pitch_deg = 0.0f;
    float balanceSpeed_dps = 0.0f; // Target speed from angle PID
    float targetAngVel_dps = 0.0f; // <<<--- Final Target Angular Velocity (deg/s)
    float speedLeft_dps = 0.0f;   // Actual speed dps
    float speedRight_dps = 0.0f;  // Actual speed dps
    float effortLeft = 0.0f;      // Motor effort -1 to 1
    float effortRight = 0.0f;
    float batteryVoltage = 0.0f;
    int systemState = 0; // Current system state enum value
    // Add other fields here if needed later (PID terms, raw gyro etc.)
};