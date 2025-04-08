#pragma once

enum class SystemState {
    INIT,             // System is initializing
    IDLE,             // Initialized, sensors ready, motors off, waiting for command
    CALIBRATING_IMU,  // Performing IMU calibration sequence
    BALANCING,        // Actively balancing
    FALLEN,           // Robot has fallen (detected by angle threshold)
    IMU_RECOVERY,     // Attempting to recover IMU communication
    FATAL_ERROR       // Unrecoverable error state (renamed from ERROR)
    // Add more states as needed (e.g., LOW_BATTERY, MANUAL_CONTROL)
};