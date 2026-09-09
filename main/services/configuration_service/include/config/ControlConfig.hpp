#pragma once

#include "config/BalanceStrategyConfig.hpp"

struct ControlConfig {
    float joystick_exponent = 1.5f;
    // Kept as an application-facing compatibility mirror for CommandProcessor.
    // The canonical strategy-specific values live under strategies.nested_pid.
    float max_target_pitch_offset_deg = 5.0f;
    bool yaw_control_enabled = false;
    BalanceStrategiesConfig strategies;

    bool operator!=(const ControlConfig& other) const {
        return joystick_exponent != other.joystick_exponent ||
               max_target_pitch_offset_deg != other.max_target_pitch_offset_deg ||
               yaw_control_enabled != other.yaw_control_enabled ||
               strategies != other.strategies;
    }

    bool operator==(const ControlConfig& other) const {
        return !(*this != other);
    }
};
