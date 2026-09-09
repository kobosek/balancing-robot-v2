#pragma once

#include "config/PIDConfig.hpp"
#include <cstdint>
#include <string>

enum class BalanceStrategyId : uint8_t {
    NESTED_PID = 0,
    LONGITUDINAL_CASCADE = 1
};

inline const char* balanceStrategyIdToString(BalanceStrategyId id)
{
    switch (id) {
        case BalanceStrategyId::NESTED_PID: return "nested_pid";
        case BalanceStrategyId::LONGITUDINAL_CASCADE: return "longitudinal_cascade";
        default: return "unknown";
    }
}

inline bool balanceStrategyIdFromString(const std::string& value, BalanceStrategyId& id)
{
    if (value == "nested_pid") {
        id = BalanceStrategyId::NESTED_PID;
        return true;
    }
    if (value == "longitudinal_cascade") {
        id = BalanceStrategyId::LONGITUDINAL_CASCADE;
        return true;
    }
    return false;
}

struct NestedPidStrategyConfig {
    PIDConfig angle;
    PIDConfig speed_left;
    PIDConfig speed_right;
    PIDConfig yaw_angle = {2.0f, 0.0f, 0.05f, -60.0f, 60.0f, -20.0f, 20.0f};
    PIDConfig yaw_rate;
    float max_target_pitch_offset_deg = 5.0f;
    bool yaw_control_enabled = false;
    uint32_t revision = 0;

    bool operator!=(const NestedPidStrategyConfig& other) const
    {
        return angle != other.angle ||
               speed_left != other.speed_left ||
               speed_right != other.speed_right ||
               yaw_angle != other.yaw_angle ||
               yaw_rate != other.yaw_rate ||
               max_target_pitch_offset_deg != other.max_target_pitch_offset_deg ||
               yaw_control_enabled != other.yaw_control_enabled ||
               revision != other.revision;
    }

    bool operator==(const NestedPidStrategyConfig& other) const { return !(*this != other); }
};

// The longitudinal fields are typed now so they cannot be confused with the
// NestedPid gains. Their runtime consumer is introduced in a later stage.
struct LongitudinalCascadeStrategyConfig {
    PIDConfig pitch;
    PIDConfig velocity;
    float position_kp = 0.0f;
    float pitch_trim_deg = 0.0f;
    float max_pitch_offset_deg = 0.0f;
    float max_pitch_rate_dps = 0.0f;
    float max_velocity_mps = 0.0f;
    float max_hold_velocity_mps = 0.0f;
    float max_acceleration_mps2 = 0.0f;
    float max_deceleration_mps2 = 0.0f;
    float hold_position_deadband_m = 0.0f;
    float hold_velocity_deadband_mps = 0.0f;
    float sync_kp = 0.0f;
    float sync_kd = 0.0f;
    float sync_max_effort = 0.0f;
    float max_effort = 1.0f;
    bool configured = false;
    uint32_t revision = 0;

    bool operator!=(const LongitudinalCascadeStrategyConfig& other) const
    {
        return pitch != other.pitch ||
               velocity != other.velocity ||
               position_kp != other.position_kp ||
               pitch_trim_deg != other.pitch_trim_deg ||
               max_pitch_offset_deg != other.max_pitch_offset_deg ||
               max_pitch_rate_dps != other.max_pitch_rate_dps ||
               max_velocity_mps != other.max_velocity_mps ||
               max_hold_velocity_mps != other.max_hold_velocity_mps ||
               max_acceleration_mps2 != other.max_acceleration_mps2 ||
               max_deceleration_mps2 != other.max_deceleration_mps2 ||
               hold_position_deadband_m != other.hold_position_deadband_m ||
               hold_velocity_deadband_mps != other.hold_velocity_deadband_mps ||
               sync_kp != other.sync_kp ||
               sync_kd != other.sync_kd ||
               sync_max_effort != other.sync_max_effort ||
               max_effort != other.max_effort ||
               configured != other.configured ||
               revision != other.revision;
    }

    bool operator==(const LongitudinalCascadeStrategyConfig& other) const { return !(*this != other); }
};

struct BalanceStrategiesConfig {
    BalanceStrategyId active = BalanceStrategyId::NESTED_PID;
    uint32_t revision = 0;
    NestedPidStrategyConfig nested_pid;
    LongitudinalCascadeStrategyConfig longitudinal_cascade;

    bool operator!=(const BalanceStrategiesConfig& other) const
    {
        return active != other.active ||
               revision != other.revision ||
               nested_pid != other.nested_pid ||
               longitudinal_cascade != other.longitudinal_cascade;
    }

    bool operator==(const BalanceStrategiesConfig& other) const { return !(*this != other); }
};
