#pragma once

#include <stdint.h> // For int64_t

// Define TelemetryDataPoint structure
struct TelemetryDataPoint {
    int64_t timestamp_us = 0;         // Common
    float pitch_deg = 0.0f;           // Index 0
    float speedLeft_dps = 0.0f;       // Index 1
    float speedRight_dps = 0.0f;      // Index 2
    float batteryVoltage = 0.0f;      // Index 3
    int systemState = 0;              // Index 4
    float speedSetpointLeft_dps = 0.0f; // Index 5
    float speedSetpointRight_dps = 0.0f;// Index 6
    float desiredAngle_deg = 0.0f;    // Index 7
    float yawRate_dps = 0.0f;         // <<< ADDED: Index 8
};