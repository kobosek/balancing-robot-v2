#pragma once

enum class SystemState {
    INIT,             // System is initializing
    IDLE,             // Initialized, sensors ready, motors off, waiting for command
    BALANCING,        // Actively balancing
    FALLEN,           // Robot has fallen (detected by angle threshold)
    SHUTDOWN,         // System is shutting down or being turned off
    FATAL_ERROR       // Unrecoverable error state (renamed from ERROR)
    // Add more states as needed (e.g., LOW_BATTERY, MANUAL_CONTROL)
};