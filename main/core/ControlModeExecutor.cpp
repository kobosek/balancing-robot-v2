#include "ControlModeExecutor.hpp"

#include "GuidedCalibrationService.hpp"
#include "PidTuningService.hpp"
#include <cmath>

ControlModeExecutor::ControlModeExecutor(BalancingAlgorithm& balancingAlgorithm,
                                         PidTuningService& pidTuningService,
                                         GuidedCalibrationService& guidedCalibrationService)
    : m_balancingAlgorithm(balancingAlgorithm),
      m_pidTuningService(pidTuningService),
      m_guidedCalibrationService(guidedCalibrationService) {}

ControlModeResult ControlModeExecutor::execute(const ControlModeInput& input)
{
    switch (input.mode) {
        case ControlRunMode::BALANCING:
            return executeBalancing(input);

        case ControlRunMode::PID_TUNING:
            return executePidTuning(input);

        case ControlRunMode::GUIDED_CALIBRATION:
            return executeGuidedCalibration(input);

        default:
            return executeDisabled();
    }
}

void ControlModeExecutor::reset()
{
    m_balancingAlgorithm.resetState();
}

BalanceStrategyId ControlModeExecutor::activeBalanceStrategyId() const
{
    return m_balancingAlgorithm.getActiveStrategyId();
}

uint32_t ControlModeExecutor::activeBalanceStrategyRevision() const
{
    return m_balancingAlgorithm.getActiveStrategyRevision();
}

uint32_t ControlModeExecutor::appliedConfigRevision() const
{
    return m_balancingAlgorithm.getAppliedConfigRevision();
}

ControlModeResult ControlModeExecutor::executeBalancing(const ControlModeInput& input)
{
    ControlModeResult result = {};
    const BalanceControlResult control = m_balancingAlgorithm.updateDetailed(
        input.dt,
        input.pitch_deg,
        input.pitch_rate_dps,
        input.yaw_deg,
        input.yaw_rate_dps,
        input.speedLeft_dps,
        input.speedRight_dps,
        input.targetPitchOffset_deg,
        input.targetAngularVelocity_dps,
        input.odometry,
        input.nowUs,
        input.motionTimeoutUs,
        input.motion,
        input.controlArmId);
    result.effort = control.effort;
    result.strategyId = control.strategyId;
    result.strategyRevision = control.configurationRevision;
    result.configRevision = control.configRevision;
    result.valid = control.valid;
    result.telemetryTargetPitchOffset_deg = input.targetPitchOffset_deg;
    result.telemetryTargetAngularVelocity_dps = input.targetAngularVelocity_dps;
    result.telemetryTargetYaw_deg = control.targetYaw_deg;
    result.telemetryDesiredYawRate_dps = control.desiredYawRate_dps;
    result.speedSetpointLeft_dps = control.speedSetpointLeft_dps;
    result.speedSetpointRight_dps = control.speedSetpointRight_dps;
    result.odometry = input.odometry;
    result.diagnostics = control.diagnostics;
    return result;
}

ControlModeResult ControlModeExecutor::executePidTuning(const ControlModeInput& input)
{
    reset();

    ControlModeResult result = {};
    result.effort = m_pidTuningService.update(input.dt, input.speedLeft_dps, input.speedRight_dps);
    result.valid = std::isfinite(result.effort.left) && std::isfinite(result.effort.right);
    result.speedSetpointLeft_dps = m_pidTuningService.getLastSpeedSetpointLeftDPS();
    result.speedSetpointRight_dps = m_pidTuningService.getLastSpeedSetpointRightDPS();
    return result;
}

ControlModeResult ControlModeExecutor::executeGuidedCalibration(const ControlModeInput& input)
{
    reset();

    GuidedCalibrationSample guidedSample = {};
    guidedSample.pitch_deg = input.pitch_deg;
    guidedSample.speedLeft_dps = input.speedLeft_dps;
    guidedSample.speedRight_dps = input.speedRight_dps;

    ControlModeResult result = {};
    result.effort = m_guidedCalibrationService.update(input.dt, guidedSample);
    result.valid = std::isfinite(result.effort.left) && std::isfinite(result.effort.right);
    return result;
}

ControlModeResult ControlModeExecutor::executeDisabled()
{
    reset();
    return {};
}
