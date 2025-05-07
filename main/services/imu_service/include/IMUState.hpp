#pragma once

enum class IMUState {
    INITIALIZED,   // Initial state after hardware initialization
    OPERATIONAL,   // Normal operation with FIFO active and health monitoring
    CALIBRATION,   // FIFO stopped during calibration, basic health monitoring active
    RECOVERY       // FIFO stopped during sensor recovery
};
