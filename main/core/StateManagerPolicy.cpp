#include "StateManagerPolicy.hpp"

namespace state_manager_policy {

const char* toString(SystemState state) {
    switch (state) {
        case SystemState::INIT:
            return "INIT";
        case SystemState::IDLE:
            return "IDLE";
        case SystemState::BALANCING:
            return "BALANCING";
        case SystemState::PID_TUNING:
            return "PID_TUNING";
        case SystemState::GUIDED_CALIBRATION:
            return "GUIDED_CALIBRATION";
        case SystemState::FALLEN:
            return "FALLEN";
        case SystemState::SHUTDOWN:
            return "SHUTDOWN";
        case SystemState::FATAL_ERROR:
            return "FATAL_ERROR";
        default:
            return "UNKNOWN";
    }
}

const char* toApiStateName(SystemState state) {
    switch (state) {
        case SystemState::INIT:
            return "INITIALIZING";
        case SystemState::IDLE:
            return "IDLE";
        case SystemState::BALANCING:
            return "BALANCING";
        case SystemState::PID_TUNING:
            return "PID_TUNING";
        case SystemState::GUIDED_CALIBRATION:
            return "GUIDED_CALIBRATION";
        case SystemState::FALLEN:
            return "FALLEN";
        case SystemState::SHUTDOWN:
            return "SHUTDOWN";
        case SystemState::FATAL_ERROR:
            return "ERROR";
        default:
            return "UNKNOWN";
    }
}

bool canStartBalancingFrom(SystemState state) {
    return state == SystemState::IDLE || state == SystemState::FALLEN;
}

bool canStartPidTuningFrom(SystemState state) {
    return state == SystemState::IDLE;
}

bool canStartGuidedCalibrationFrom(SystemState state) {
    return state == SystemState::IDLE;
}

bool isMotorActiveState(SystemState state) {
    return state == SystemState::BALANCING ||
           state == SystemState::PID_TUNING ||
           state == SystemState::GUIDED_CALIBRATION;
}

bool shouldReturnToIdleOnImuError(SystemState state) {
    return state == SystemState::BALANCING ||
           state == SystemState::FALLEN ||
           state == SystemState::PID_TUNING;
}

bool batteryBlocksMotion(bool criticalBatteryMotorShutdownEnabled, bool batteryCritical) {
    return criticalBatteryMotorShutdownEnabled && batteryCritical;
}

bool shouldRequestCalibrationNow(SystemState state, bool force) {
    return state == SystemState::IDLE || force;
}

bool canDeferCalibration(SystemState state, bool force) {
    return isMotorActiveState(state) && !force;
}

bool shouldInitiatePendingCalibration(SystemState previousState, SystemState newState, bool pendingCalibration) {
    return previousState != SystemState::IDLE &&
           newState == SystemState::IDLE &&
           pendingCalibration;
}

bool isFallDetectionActive(SystemState state, bool fallDetectionEnabled) {
    return fallDetectionEnabled && state == SystemState::BALANCING;
}

bool isAutoBalancingActive(SystemState state, bool autoBalancingEnabled) {
    return autoBalancingEnabled &&
           (state == SystemState::IDLE || state == SystemState::FALLEN);
}

bool isCommandInputEnabled(SystemState state) {
    return state == SystemState::BALANCING;
}

ControlRunMode controlRunModeFor(SystemState state) {
    switch (state) {
        case SystemState::BALANCING:
            return ControlRunMode::BALANCING;
        case SystemState::PID_TUNING:
            return ControlRunMode::PID_TUNING;
        case SystemState::GUIDED_CALIBRATION:
            return ControlRunMode::GUIDED_CALIBRATION;
        default:
            return ControlRunMode::DISABLED;
    }
}

bool isTelemetryEnabled(SystemState state) {
    return state != SystemState::INIT &&
           state != SystemState::SHUTDOWN &&
           state != SystemState::FATAL_ERROR;
}

bool isImuCalibrationAllowed(SystemState state) {
    return state == SystemState::IDLE;
}

bool isImuAutoAttachAllowed(SystemState state) {
    return state == SystemState::IDLE;
}

bool isImuHardwareConfigApplyAllowed(SystemState state) {
    return state != SystemState::BALANCING;
}

bool isOtaUpdateAllowed(SystemState state) {
    return state == SystemState::IDLE;
}

} // namespace state_manager_policy
