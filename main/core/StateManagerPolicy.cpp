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

} // namespace state_manager_policy
