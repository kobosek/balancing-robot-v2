#pragma once

#include "BalancingAlgorithm.hpp"
#include "CONTROL_RunModeChanged.hpp"

class GuidedCalibrationService;
class PidTuningService;

struct ControlModeInput {
    ControlRunMode mode = ControlRunMode::DISABLED;
    float dt = 0.0f;
    float pitch_deg = 0.0f;
    float pitch_rate_dps = 0.0f;
    float yaw_rate_dps = 0.0f;
    float speedLeft_dps = 0.0f;
    float speedRight_dps = 0.0f;
    float targetPitchOffset_deg = 0.0f;
    float targetAngularVelocity_dps = 0.0f;
};

struct ControlModeResult {
    MotorEffort effort = {};
    float telemetryTargetPitchOffset_deg = 0.0f;
    float telemetryTargetAngularVelocity_dps = 0.0f;
    float speedSetpointLeft_dps = 0.0f;
    float speedSetpointRight_dps = 0.0f;
};

class ControlModeExecutor {
public:
    ControlModeExecutor(BalancingAlgorithm& balancingAlgorithm,
                        PidTuningService& pidTuningService,
                        GuidedCalibrationService& guidedCalibrationService);

    ControlModeResult execute(const ControlModeInput& input);
    void reset();

private:
    BalancingAlgorithm& m_balancingAlgorithm;
    PidTuningService& m_pidTuningService;
    GuidedCalibrationService& m_guidedCalibrationService;

    ControlModeResult executeBalancing(const ControlModeInput& input);
    ControlModeResult executePidTuning(const ControlModeInput& input);
    ControlModeResult executeGuidedCalibration(const ControlModeInput& input);
    ControlModeResult executeDisabled();
};
