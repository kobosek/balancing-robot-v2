#pragma once

#include "SystemState.hpp"
#include "CONTROL_RunModeChanged.hpp"

namespace state_manager_policy {

const char* toString(SystemState state);
const char* toApiStateName(SystemState state);

bool canStartBalancingFrom(SystemState state);
bool canStartPidTuningFrom(SystemState state);
bool canStartGuidedCalibrationFrom(SystemState state);
bool isMotorActiveState(SystemState state);
bool shouldReturnToIdleOnImuError(SystemState state);
bool batteryBlocksMotion(bool criticalBatteryMotorShutdownEnabled, bool batteryCritical);
bool shouldRequestCalibrationNow(SystemState state, bool force);
bool canDeferCalibration(SystemState state, bool force);
bool shouldInitiatePendingCalibration(SystemState previousState, SystemState newState, bool pendingCalibration);

bool isFallDetectionActive(SystemState state, bool fallDetectionEnabled);
bool isAutoBalancingActive(SystemState state, bool autoBalancingEnabled);
bool isCommandInputEnabled(SystemState state);
ControlRunMode controlRunModeFor(SystemState state);
bool isTelemetryEnabled(SystemState state);
bool isImuCalibrationAllowed(SystemState state);
bool isImuAutoAttachAllowed(SystemState state);
bool isImuHardwareConfigApplyAllowed(SystemState state);
bool isOtaUpdateAllowed(SystemState state);

} // namespace state_manager_policy
