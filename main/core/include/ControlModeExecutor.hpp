#pragma once

#include "BalancingAlgorithm.hpp"
#include "CONTROL_RunModeChanged.hpp"
#include "LongitudinalOdometry.hpp"

class GuidedCalibrationService;
class PidTuningService;

struct ControlModeInput {
    ControlRunMode mode = ControlRunMode::DISABLED;
    float dt = 0.0f;
    float pitch_deg = 0.0f;
    float pitch_rate_dps = 0.0f;
    float yaw_deg = 0.0f;
    float yaw_rate_dps = 0.0f;
    float speedLeft_dps = 0.0f;
    float speedRight_dps = 0.0f;
    float targetPitchOffset_deg = 0.0f;
    float targetAngularVelocity_dps = 0.0f;
    int64_t nowUs = 0;
    int64_t motionTimeoutUs = 0;
    LongitudinalMotionCommand motion = {};
    LongitudinalOdometryResult odometry = {};
    uint64_t controlArmId = 0;
};

struct ControlModeResult {
    MotorEffort effort = {};
    BalanceStrategyId strategyId = BalanceStrategyId::NESTED_PID;
    uint32_t strategyRevision = 0;
    uint32_t configRevision = 0;
    bool valid = false;
    float telemetryTargetPitchOffset_deg = 0.0f;
    float telemetryTargetAngularVelocity_dps = 0.0f;
    float telemetryTargetYaw_deg = 0.0f;
    float telemetryDesiredYawRate_dps = 0.0f;
    float speedSetpointLeft_dps = 0.0f;
    float speedSetpointRight_dps = 0.0f;
    LongitudinalOdometryResult odometry = {};
    BalanceControlDiagnostics diagnostics = {};
};

class ControlModeExecutor {
public:
    ControlModeExecutor(BalancingAlgorithm& balancingAlgorithm,
                        PidTuningService& pidTuningService,
                        GuidedCalibrationService& guidedCalibrationService);

    ControlModeResult execute(const ControlModeInput& input);
    void reset();
    BalanceStrategyId activeBalanceStrategyId() const;
    uint32_t activeBalanceStrategyRevision() const;
    uint32_t appliedConfigRevision() const;

private:
    BalancingAlgorithm& m_balancingAlgorithm;
    PidTuningService& m_pidTuningService;
    GuidedCalibrationService& m_guidedCalibrationService;

    ControlModeResult executeBalancing(const ControlModeInput& input);
    ControlModeResult executePidTuning(const ControlModeInput& input);
    ControlModeResult executeGuidedCalibration(const ControlModeInput& input);
    ControlModeResult executeDisabled();
};
