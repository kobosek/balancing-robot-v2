#include "NestedPidBalanceStrategy.hpp"

#include <algorithm>
#include <cmath>

namespace {
constexpr float YAW_COMMAND_DEADBAND_DPS = 1e-3f;
}

NestedPidBalanceStrategy::NestedPidBalanceStrategy() :
    m_anglePid("angle"),
    m_speedPidLeft(),
    m_speedPidRight(),
    m_yawAnglePid("yaw_angle"),
    m_yawRatePid("yaw_rate") {}

MotorEffort NestedPidBalanceStrategy::update(const BalanceControlInput& input)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_last_diagnostics = {};
    m_last_diagnostics.strategyId = BalanceStrategyId::NESTED_PID;
    m_last_diagnostics.yawControlEnabled = m_yaw_control_enabled;
    m_last_diagnostics.phase = BalanceControlPhase::INACTIVE;
    MotorEffort effort = {0.0f, 0.0f};
    if (!std::isfinite(input.dt) || input.dt <= 0.0f ||
        !std::isfinite(input.currentPitch_deg) ||
        !std::isfinite(input.currentPitchRate_dps) ||
        !std::isfinite(input.currentYaw_deg) ||
        !std::isfinite(input.currentYawRate_dps) ||
        !std::isfinite(input.currentSpeedLeft_dps) ||
        !std::isfinite(input.currentSpeedRight_dps) ||
        !std::isfinite(input.targetPitchOffset_deg) ||
        !std::isfinite(input.targetAngularVelocity_dps) ||
        !std::isfinite(m_max_control_effort) || m_max_control_effort < 0.0f ||
        !std::isfinite(m_angle_pid_output_min) ||
        !std::isfinite(m_angle_pid_output_max) ||
        m_angle_pid_output_min > m_angle_pid_output_max) {
        ESP_LOGW(TAG, "Invalid NestedPid input (dt %.4f)", input.dt);
        return effort;
    }

    const bool captureYaw = m_yaw_control_enabled && !m_has_target_yaw;
    double nextTargetYaw_deg = m_target_yaw_deg;
    bool nextHasTargetYaw = m_has_target_yaw;
    if (!m_yaw_control_enabled) {
        nextHasTargetYaw = false;
        nextTargetYaw_deg = input.currentYaw_deg;
    } else if (captureYaw) {
        nextTargetYaw_deg = input.currentYaw_deg;
        nextHasTargetYaw = true;
    }

    if (m_yaw_control_enabled && std::fabs(input.targetAngularVelocity_dps) > YAW_COMMAND_DEADBAND_DPS) {
        nextTargetYaw_deg += static_cast<double>(input.targetAngularVelocity_dps) * input.dt;
    }
    if (!std::isfinite(nextTargetYaw_deg)) {
        return effort;
    }

    // Preview every stateful controller before committing any of them.  A
    // finite input can still overflow a downstream multiplication; in that
    // case the complete NestedPid step is rejected without advancing only
    // the angle or yaw history.
    const auto anglePreview = m_anglePid.previewWithMeasurementRate(
        input.targetPitchOffset_deg,
        input.currentPitch_deg,
        input.currentPitchRate_dps,
        input.dt);
    if (!anglePreview.valid) {
        return effort;
    }
    const float baseSpeed_dps = anglePreview.output;

    const float targetYaw_deg = m_yaw_control_enabled
        ? static_cast<float>(nextTargetYaw_deg) : input.currentYaw_deg;
    if (!std::isfinite(targetYaw_deg)) {
        return effort;
    }
    const float yawAngleError_deg = targetYaw_deg - input.currentYaw_deg;
    float yawAngleCorrectionRate_dps = 0.0f;
    control_math::PidStepResult yawAnglePreview = {};
    if (m_yaw_control_enabled) {
        yawAnglePreview = captureYaw
            ? m_yawAnglePid.previewWithMeasurementRateAfterReset(
                targetYaw_deg,
                input.currentYaw_deg,
                input.currentYawRate_dps,
                input.dt)
            : m_yawAnglePid.previewWithMeasurementRate(
                targetYaw_deg,
                input.currentYaw_deg,
                input.currentYawRate_dps,
                input.dt);
        if (!yawAnglePreview.valid) {
            return effort;
        }
        yawAngleCorrectionRate_dps = yawAnglePreview.output;
    }

    const float desiredYawRate_dps = m_yaw_control_enabled
        ? (input.targetAngularVelocity_dps + yawAngleCorrectionRate_dps)
        : input.targetAngularVelocity_dps;
    if (!std::isfinite(desiredYawRate_dps)) {
        return effort;
    }
    const float yawRateError_dps = desiredYawRate_dps - input.currentYawRate_dps;
    const float commandedTurnDiff_dps = yawRateToWheelDiffDps(desiredYawRate_dps);
    if (!std::isfinite(commandedTurnDiff_dps)) {
        return effort;
    }
    float yawCorrectionDiff_dps = 0.0f;
    control_math::PidStepResult yawRatePreview = {};
    if (m_yaw_control_enabled) {
        yawRatePreview = captureYaw
            ? m_yawRatePid.previewAfterReset(
                desiredYawRate_dps,
                input.currentYawRate_dps,
                input.dt)
            : m_yawRatePid.preview(
                desiredYawRate_dps,
                input.currentYawRate_dps,
                input.dt);
        if (!yawRatePreview.valid) {
            return effort;
        }
        yawCorrectionDiff_dps = yawRatePreview.output;
    }
    if (!std::isfinite(yawCorrectionDiff_dps)) {
        return effort;
    }

    float speedSetpointLeft_dps = baseSpeed_dps - commandedTurnDiff_dps;
    float speedSetpointRight_dps = baseSpeed_dps + commandedTurnDiff_dps;
    if (m_yaw_control_enabled) {
        speedSetpointLeft_dps -= yawCorrectionDiff_dps;
        speedSetpointRight_dps += yawCorrectionDiff_dps;
    }

    speedSetpointLeft_dps = std::max(m_angle_pid_output_min, std::min(m_angle_pid_output_max, speedSetpointLeft_dps));
    speedSetpointRight_dps = std::max(m_angle_pid_output_min, std::min(m_angle_pid_output_max, speedSetpointRight_dps));
    if (!std::isfinite(speedSetpointLeft_dps) ||
        !std::isfinite(speedSetpointRight_dps)) {
        return effort;
    }

    const auto leftSpeedPreview = m_speedPidLeft.preview(
        speedSetpointLeft_dps, input.currentSpeedLeft_dps, input.dt);
    const auto rightSpeedPreview = m_speedPidRight.preview(
        speedSetpointRight_dps, input.currentSpeedRight_dps, input.dt);
    if (!leftSpeedPreview.valid || !rightSpeedPreview.valid) {
        return effort;
    }

    const auto rejectCommittedStep = [&]() -> MotorEffort {
        // Preview and commit use identical state and arithmetic.  This is a
        // defensive path for an unexpected commit failure; clearing every
        // controller keeps a partially committed step from becoming the
        // derivative/integral reference for the next sample.
        m_anglePid.reset();
        m_speedPidLeft.reset();
        m_speedPidRight.reset();
        m_yawAnglePid.reset();
        m_yawRatePid.reset();
        return effort;
    };

    // Commit the capture reset only after all previews, including both wheel
    // controllers, have succeeded. An unexpected later failure is handled by
    // rejectCommittedStep, which clears the complete state consistently.
    if (captureYaw) {
        m_yawAnglePid.reset();
        m_yawRatePid.reset();
    }

    const auto angleStep = m_anglePid.computeWithMeasurementRateDetailed(
        input.targetPitchOffset_deg,
        input.currentPitch_deg,
        input.currentPitchRate_dps,
        input.dt);
    if (!angleStep.valid) {
        return rejectCommittedStep();
    }
    control_math::PidStepResult yawAngleStep = {};
    if (m_yaw_control_enabled) {
        yawAngleStep = m_yawAnglePid.computeWithMeasurementRateDetailed(
            targetYaw_deg,
            input.currentYaw_deg,
            input.currentYawRate_dps,
            input.dt);
        if (!yawAngleStep.valid) {
            return rejectCommittedStep();
        }
    }
    control_math::PidStepResult yawRateStep = {};
    if (m_yaw_control_enabled) {
        yawRateStep = m_yawRatePid.computeDetailed(
            desiredYawRate_dps,
            input.currentYawRate_dps,
            input.dt);
        if (!yawRateStep.valid) {
            return rejectCommittedStep();
        }
    }
    const auto leftSpeedStep = m_speedPidLeft.update(
        speedSetpointLeft_dps, input.currentSpeedLeft_dps, input.dt);
    const auto rightSpeedStep = m_speedPidRight.update(
        speedSetpointRight_dps, input.currentSpeedRight_dps, input.dt);
    if (!leftSpeedStep.valid || !rightSpeedStep.valid) {
        return rejectCommittedStep();
    }

    m_target_yaw_deg = nextTargetYaw_deg;
    m_has_target_yaw = nextHasTargetYaw;
    m_last_speed_setpoint_left_dps = speedSetpointLeft_dps;
    m_last_speed_setpoint_right_dps = speedSetpointRight_dps;
    m_last_target_yaw_deg = targetYaw_deg;
    m_last_desired_yaw_rate_dps = desiredYawRate_dps;

    effort.left = leftSpeedStep.effort;
    effort.right = rightSpeedStep.effort;

    effort.left = std::max(-m_max_control_effort, std::min(m_max_control_effort, effort.left));
    effort.right = std::max(-m_max_control_effort, std::min(m_max_control_effort, effort.right));

    m_last_diagnostics = {};
    m_last_diagnostics.strategyId = BalanceStrategyId::NESTED_PID;
    m_last_diagnostics.yawControlEnabled = m_yaw_control_enabled;
    m_last_diagnostics.phase = BalanceControlPhase::LEGACY;
    m_last_diagnostics.valid = true;
    m_last_diagnostics.targetPitchValid = true;
    m_last_diagnostics.targetPitch_deg = input.targetPitchOffset_deg;
    m_last_diagnostics.leftEffort = effort.left;
    m_last_diagnostics.rightEffort = effort.right;

    ESP_LOGV(TAG, "P:%.1f|PR:%.1f|TgtP:%.1f|Yaw:%.1f|TgtYaw:%.1f|YawErr:%.1f|YawR:%.1f|CmdYawR:%.1f|DesYawR:%.1f|YawRE:%.1f|YawAC:%.1f|YawRC:%.1f|TurnFF:%.1f|LSet:%.1f RSet:%.1f|LCur:%.1f RCur:%.1f|LEff:%.2f REff:%.2f",
             input.currentPitch_deg, input.currentPitchRate_dps, input.targetPitchOffset_deg,
             input.currentYaw_deg, targetYaw_deg, yawAngleError_deg,
             input.currentYawRate_dps, input.targetAngularVelocity_dps, desiredYawRate_dps, yawRateError_dps,
             yawAngleCorrectionRate_dps, yawCorrectionDiff_dps, commandedTurnDiff_dps,
             speedSetpointLeft_dps, speedSetpointRight_dps,
             input.currentSpeedLeft_dps, input.currentSpeedRight_dps,
             effort.left, effort.right);

    return effort;
}

void NestedPidBalanceStrategy::reset()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_anglePid.reset();
    m_speedPidLeft.reset();
    m_speedPidRight.reset();
    m_yawAnglePid.reset();
    m_yawRatePid.reset();
    m_has_target_yaw = false;
    m_target_yaw_deg = 0.0f;
    m_last_speed_setpoint_left_dps = 0.0f;
    m_last_speed_setpoint_right_dps = 0.0f;
    m_last_target_yaw_deg = 0.0f;
    m_last_desired_yaw_rate_dps = 0.0f;
    m_last_diagnostics = {};
    m_last_diagnostics.strategyId = BalanceStrategyId::NESTED_PID;
    m_last_diagnostics.phase = BalanceControlPhase::INACTIVE;
}

void NestedPidBalanceStrategy::applyConfig(const ConfigData& config)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    const auto& nested = config.control.strategies.nested_pid;
    ESP_LOGD(TAG, "Applying nested PID balance config.");
    auto previous = m_config;
    previous.revision = nested.revision;
    const bool firstConfig = !m_has_config;
    const bool angleChanged = firstConfig || previous.angle != nested.angle;
    const bool speedLeftChanged = firstConfig || previous.speed_left != nested.speed_left;
    const bool speedRightChanged = firstConfig || previous.speed_right != nested.speed_right;
    const bool yawAngleChanged = firstConfig || previous.yaw_angle != nested.yaw_angle;
    const bool yawRateChanged = firstConfig || previous.yaw_rate != nested.yaw_rate;
    const bool yawModeChanged = firstConfig ||
        previous.yaw_control_enabled != nested.yaw_control_enabled;
    const bool dimensionsChanged = firstConfig ||
        m_encoder_config != config.encoder || m_dimensions_config != config.dimensions;

    if (angleChanged) {
        m_anglePid.updateParams(nested.angle);
    }
    if (speedLeftChanged) {
        m_speedPidLeft.setParameters({
            nested.speed_left.pid_kp,
            nested.speed_left.pid_ki,
            nested.speed_left.pid_kd,
            nested.speed_left.pid_output_min,
            nested.speed_left.pid_output_max,
            nested.speed_left.pid_iterm_min,
            nested.speed_left.pid_iterm_max
        });
    }
    if (speedRightChanged) {
        m_speedPidRight.setParameters({
            nested.speed_right.pid_kp,
            nested.speed_right.pid_ki,
            nested.speed_right.pid_kd,
            nested.speed_right.pid_output_min,
            nested.speed_right.pid_output_max,
            nested.speed_right.pid_iterm_min,
            nested.speed_right.pid_iterm_max
        });
    }
    if (yawAngleChanged) {
        m_yawAnglePid.updateParams(nested.yaw_angle);
    }
    if (yawRateChanged) {
        m_yawRatePid.updateParams(nested.yaw_rate);
    }

    const bool previousYawControlEnabled = m_yaw_control_enabled;
    m_yaw_control_enabled = nested.yaw_control_enabled;
    if (yawModeChanged && previousYawControlEnabled != m_yaw_control_enabled) {
        m_yawAnglePid.reset();
        m_yawRatePid.reset();
        m_has_target_yaw = false;
    }

    m_angle_pid_output_min = nested.angle.getOutputMin();
    m_angle_pid_output_max = nested.angle.getOutputMax();
    const bool meaningfulChange = firstConfig || angleChanged || speedLeftChanged ||
        speedRightChanged || yawAngleChanged || yawRateChanged || yawModeChanged ||
        previous.max_target_pitch_offset_deg != nested.max_target_pitch_offset_deg ||
        dimensionsChanged;
    if (meaningfulChange) {
        m_last_diagnostics = {};
        m_last_diagnostics.strategyId = BalanceStrategyId::NESTED_PID;
        m_last_diagnostics.phase = BalanceControlPhase::INACTIVE;
    }
    m_config = nested;
    m_encoder_config = config.encoder;
    m_dimensions_config = config.dimensions;
    m_has_config = true;
    if (dimensionsChanged) {
        updateDimensions(config.encoder, config.dimensions);
    }
}

void NestedPidBalanceStrategy::updatePidConfig(const std::string& pidName, const PIDConfig& config)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    ESP_LOGD(TAG, "Handling PID config update for '%s'", pidName.c_str());

    if (pidName == "angle") {
        m_anglePid.updateParams(config);
        m_config.angle = config;
        m_angle_pid_output_min = config.getOutputMin();
        m_angle_pid_output_max = config.getOutputMax();
    } else if (pidName == "speed_left") {
        m_speedPidLeft.setParameters({
            config.pid_kp, config.pid_ki, config.pid_kd,
            config.pid_output_min, config.pid_output_max,
            config.pid_iterm_min, config.pid_iterm_max
        });
        m_config.speed_left = config;
    } else if (pidName == "speed_right") {
        m_speedPidRight.setParameters({
            config.pid_kp, config.pid_ki, config.pid_kd,
            config.pid_output_min, config.pid_output_max,
            config.pid_iterm_min, config.pid_iterm_max
        });
        m_config.speed_right = config;
    } else if (pidName == "yaw_angle") {
        m_yawAnglePid.updateParams(config);
        m_config.yaw_angle = config;
    } else if (pidName == "yaw_rate") {
        m_yawRatePid.updateParams(config);
        m_config.yaw_rate = config;
    } else {
        ESP_LOGW(TAG, "Received PID config update for unknown controller: %s", pidName.c_str());
    }
}

float NestedPidBalanceStrategy::getLastSpeedSetpointLeftDPS() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_last_speed_setpoint_left_dps;
}

float NestedPidBalanceStrategy::getLastSpeedSetpointRightDPS() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_last_speed_setpoint_right_dps;
}

float NestedPidBalanceStrategy::getLastTargetYawDeg() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_last_target_yaw_deg;
}

float NestedPidBalanceStrategy::getLastDesiredYawRateDPS() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_last_desired_yaw_rate_dps;
}

bool NestedPidBalanceStrategy::isYawControlEnabled() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_yaw_control_enabled;
}

BalanceControlDiagnostics NestedPidBalanceStrategy::getDiagnostics() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_last_diagnostics;
}

void NestedPidBalanceStrategy::updateDimensions(const EncoderConfig& encoderConfig,
                                                const RobotDimensionsConfig& dimensionsConfig)
{
    m_wheel_radius_m = encoderConfig.wheel_diameter_mm / 2000.0f;
    m_robot_wheelbase_m = dimensionsConfig.wheelbase_m;
    ESP_LOGD(TAG, "Internal dimensions updated. Wheel Radius: %.4f m, Wheelbase: %.4f m",
             m_wheel_radius_m, m_robot_wheelbase_m);
}

float NestedPidBalanceStrategy::yawRateToWheelDiffDps(float yawRate_dps) const
{
    if (m_wheel_radius_m <= 1e-5f || m_robot_wheelbase_m <= 1e-5f) {
        return 0.0f;
    }

    return (m_robot_wheelbase_m / (2.0f * m_wheel_radius_m)) * yawRate_dps;
}
