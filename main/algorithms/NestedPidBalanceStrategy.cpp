#include "NestedPidBalanceStrategy.hpp"

#include <algorithm>
#include <cmath>

namespace {
constexpr float YAW_COMMAND_DEADBAND_DPS = 1e-3f;
}

NestedPidBalanceStrategy::NestedPidBalanceStrategy() :
    m_anglePid("angle"),
    m_speedPidLeft("speed_left"),
    m_speedPidRight("speed_right"),
    m_yawAnglePid("yaw_angle"),
    m_yawRatePid("yaw_rate") {}

MotorEffort NestedPidBalanceStrategy::update(const BalanceControlInput& input)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    MotorEffort effort = {0.0f, 0.0f};
    if (input.dt <= 0.0f) {
        ESP_LOGW(TAG, "Invalid dt (%.4f)", input.dt);
        return effort;
    }

    float baseSpeed_dps = m_anglePid.computeWithMeasurementRate(
        input.targetPitchOffset_deg,
        input.currentPitch_deg,
        input.currentPitchRate_dps,
        input.dt);

    if (!m_yaw_control_enabled) {
        m_has_target_yaw = false;
        m_target_yaw_deg = input.currentYaw_deg;
    } else if (!m_has_target_yaw) {
        m_target_yaw_deg = input.currentYaw_deg;
        m_has_target_yaw = true;
        m_yawAnglePid.reset();
        m_yawRatePid.reset();
    }

    if (m_yaw_control_enabled && std::fabs(input.targetAngularVelocity_dps) > YAW_COMMAND_DEADBAND_DPS) {
        m_target_yaw_deg += input.targetAngularVelocity_dps * input.dt;
    }

    const float targetYaw_deg = m_yaw_control_enabled ? m_target_yaw_deg : input.currentYaw_deg;
    const float yawAngleError_deg = targetYaw_deg - input.currentYaw_deg;
    float yawAngleCorrectionRate_dps = 0.0f;
    if (m_yaw_control_enabled) {
        yawAngleCorrectionRate_dps = m_yawAnglePid.computeWithMeasurementRate(
            targetYaw_deg,
            input.currentYaw_deg,
            input.currentYawRate_dps,
            input.dt);
    }

    const float desiredYawRate_dps = m_yaw_control_enabled
        ? (input.targetAngularVelocity_dps + yawAngleCorrectionRate_dps)
        : input.targetAngularVelocity_dps;
    const float commandedTurnDiff_dps = yawRateToWheelDiffDps(desiredYawRate_dps);
    const float yawRateError_dps = desiredYawRate_dps - input.currentYawRate_dps;
    float yawCorrectionDiff_dps = 0.0f;
    if (m_yaw_control_enabled) {
        yawCorrectionDiff_dps = m_yawRatePid.compute(
            desiredYawRate_dps,
            input.currentYawRate_dps,
            input.dt);
    }

    float speedSetpointLeft_dps = baseSpeed_dps - commandedTurnDiff_dps;
    float speedSetpointRight_dps = baseSpeed_dps + commandedTurnDiff_dps;
    if (m_yaw_control_enabled) {
        speedSetpointLeft_dps -= yawCorrectionDiff_dps;
        speedSetpointRight_dps += yawCorrectionDiff_dps;
    }

    speedSetpointLeft_dps = std::max(m_angle_pid_output_min, std::min(m_angle_pid_output_max, speedSetpointLeft_dps));
    speedSetpointRight_dps = std::max(m_angle_pid_output_min, std::min(m_angle_pid_output_max, speedSetpointRight_dps));

    m_last_speed_setpoint_left_dps = speedSetpointLeft_dps;
    m_last_speed_setpoint_right_dps = speedSetpointRight_dps;
    m_last_target_yaw_deg = targetYaw_deg;
    m_last_desired_yaw_rate_dps = desiredYawRate_dps;

    effort.left = m_speedPidLeft.compute(speedSetpointLeft_dps, input.currentSpeedLeft_dps, input.dt);
    effort.right = m_speedPidRight.compute(speedSetpointRight_dps, input.currentSpeedRight_dps, input.dt);

    effort.left = std::max(-m_max_control_effort, std::min(m_max_control_effort, effort.left));
    effort.right = std::max(-m_max_control_effort, std::min(m_max_control_effort, effort.right));

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
}

void NestedPidBalanceStrategy::applyConfig(const ConfigData& config)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    ESP_LOGD(TAG, "Applying nested PID balance config.");
    m_anglePid.updateParams(config.pid_angle);
    m_speedPidLeft.updateParams(config.pid_speed_left);
    m_speedPidRight.updateParams(config.pid_speed_right);
    m_yawAnglePid.updateParams(config.pid_yaw_angle);
    m_yawRatePid.updateParams(config.pid_yaw_rate);

    const bool previousYawControlEnabled = m_yaw_control_enabled;
    m_yaw_control_enabled = config.control.yaw_control_enabled;
    if (previousYawControlEnabled != m_yaw_control_enabled) {
        m_yawAnglePid.reset();
        m_yawRatePid.reset();
        m_has_target_yaw = false;
    }

    m_angle_pid_output_min = config.pid_angle.getOutputMin();
    m_angle_pid_output_max = config.pid_angle.getOutputMax();
    updateDimensions(config.encoder, config.dimensions);
}

void NestedPidBalanceStrategy::updatePidConfig(const std::string& pidName, const PIDConfig& config)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    ESP_LOGD(TAG, "Handling PID config update for '%s'", pidName.c_str());

    if (pidName == "angle") {
        m_anglePid.updateParams(config);
        m_angle_pid_output_min = config.getOutputMin();
        m_angle_pid_output_max = config.getOutputMax();
    } else if (pidName == "speed_left") {
        m_speedPidLeft.updateParams(config);
    } else if (pidName == "speed_right") {
        m_speedPidRight.updateParams(config);
    } else if (pidName == "yaw_angle") {
        m_yawAnglePid.updateParams(config);
    } else if (pidName == "yaw_rate") {
        m_yawRatePid.updateParams(config);
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
