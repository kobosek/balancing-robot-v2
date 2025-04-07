#pragma once

enum class SystemState {
    INIT,             // System is initializing
    IDLE,             // Initialized, sensors ready, motors off, waiting for command
    CALIBRATING_IMU,  // Performing IMU calibration sequence
    BALANCING,        // Actively balancing
    FALLEN,           // Robot has fallen (detected by angle threshold)
    ERROR             // Unrecoverable error state
    // Add more states as needed (e.g., LOW_BATTERY, MANUAL_CONTROL)
};