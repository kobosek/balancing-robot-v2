#pragma once

#include "SystemState.hpp"

namespace state_manager_policy {

const char* toString(SystemState state);

bool canStartBalancingFrom(SystemState state);
bool canStartPidTuningFrom(SystemState state);
bool canStartGuidedCalibrationFrom(SystemState state);
bool isMotorActiveState(SystemState state);
bool shouldReturnToIdleOnImuError(SystemState state);
bool batteryBlocksMotion(bool criticalBatteryMotorShutdownEnabled, bool batteryCritical);
bool shouldRequestCalibrationNow(SystemState state, bool force);
bool canDeferCalibration(SystemState state, bool force);

} // namespace state_manager_policy
