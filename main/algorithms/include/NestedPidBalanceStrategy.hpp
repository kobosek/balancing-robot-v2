#pragma once

#include "IBalanceControlStrategy.hpp"
#include "PIDController.hpp"
#include "esp_log.h"
#include <mutex>

class NestedPidBalanceStrategy : public IBalanceControlStrategy {
public:
    NestedPidBalanceStrategy();

    const char* name() const override { return "nested_pid"; }
    MotorEffort update(const BalanceControlInput& input) override;
    void reset() override;
    void applyConfig(const ConfigData& config) override;
    void updatePidConfig(const std::string& pidName, const PIDConfig& config) override;

    float getLastSpeedSetpointLeftDPS() const override;
    float getLastSpeedSetpointRightDPS() const override;
    bool isYawControlEnabled() const override;

private:
    static constexpr const char* TAG = "NestedPidBalance";

    mutable std::mutex m_mutex;
    PIDController m_anglePid;
    PIDController m_speedPidLeft;
    PIDController m_speedPidRight;
    PIDController m_yawRatePid;

    float m_max_control_effort = 1.0f;
    float m_wheel_radius_m = 0.0f;
    float m_robot_wheelbase_m = 0.0f;
    float m_angle_pid_output_min = 0.0f;
    float m_angle_pid_output_max = 0.0f;
    bool m_yaw_control_enabled = false;
    float m_last_speed_setpoint_left_dps = 0.0f;
    float m_last_speed_setpoint_right_dps = 0.0f;

    void updateDimensions(const EncoderConfig& encoderConfig, const RobotDimensionsConfig& dimensionsConfig);
};

