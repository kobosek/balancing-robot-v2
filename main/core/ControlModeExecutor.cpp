#include "ControlModeExecutor.hpp"

#include "GuidedCalibrationService.hpp"
#include "PidTuningService.hpp"

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

ControlModeResult ControlModeExecutor::executeBalancing(const ControlModeInput& input)
{
    ControlModeResult result = {};
    result.effort = m_balancingAlgorithm.update(input.dt,
                                                input.pitch_deg,
                                                input.pitch_rate_dps,
                                                input.yaw_rate_dps,
                                                input.speedLeft_dps,
                                                input.speedRight_dps,
                                                input.targetPitchOffset_deg,
                                                input.targetAngularVelocity_dps);
    result.telemetryTargetPitchOffset_deg = input.targetPitchOffset_deg;
    result.telemetryTargetAngularVelocity_dps = input.targetAngularVelocity_dps;
    result.speedSetpointLeft_dps = m_balancingAlgorithm.getLastSpeedSetpointLeftDPS();
    result.speedSetpointRight_dps = m_balancingAlgorithm.getLastSpeedSetpointRightDPS();
    return result;
}

ControlModeResult ControlModeExecutor::executePidTuning(const ControlModeInput& input)
{
    reset();

    ControlModeResult result = {};
    result.effort = m_pidTuningService.update(input.dt, input.speedLeft_dps, input.speedRight_dps);
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
    return result;
}

ControlModeResult ControlModeExecutor::executeDisabled()
{
    reset();
    return {};
}
