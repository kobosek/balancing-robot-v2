#pragma once

#include "config/PIDConfig.hpp"
#include <cstdint>
#include <string>

enum class BalanceStrategyId : uint8_t {
    NESTED_PID = 0,
    LONGITUDINAL_CASCADE = 1
};

enum class LongitudinalLoopMode : uint8_t {
    PITCH_ONLY = 0,
    VELOCITY = 1,
    POSITION_HOLD = 2
};

inline const char* longitudinalLoopModeToString(LongitudinalLoopMode mode)
{
    switch (mode) {
        case LongitudinalLoopMode::PITCH_ONLY: return "pitch_only";
        case LongitudinalLoopMode::VELOCITY: return "velocity";
        case LongitudinalLoopMode::POSITION_HOLD: return "position_hold";
        default: return "unknown";
    }
}

inline bool longitudinalLoopModeFromString(const std::string& value,
                                           LongitudinalLoopMode& mode)
{
    if (value == "pitch_only") {
        mode = LongitudinalLoopMode::PITCH_ONLY;
        return true;
    }
    if (value == "velocity") {
        mode = LongitudinalLoopMode::VELOCITY;
        return true;
    }
    if (value == "position_hold") {
        mode = LongitudinalLoopMode::POSITION_HOLD;
        return true;
    }
    return false;
}

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

// The longitudinal fields are typed so they cannot be confused with the
// NestedPid gains. LongitudinalCascadeBalanceStrategy consumes the pitch,
// velocity, position-hold and synchronization fields when the corresponding
// loop mode/options are validated and enabled.
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
    // Synchronization is an optional differential correction.  The existing
    // gains are interpreted in physical odometry units: sync_kp multiplies
    // distance error (m) and sync_kd multiplies wheel velocity difference
    // (m/s).  They are signed so the measured installation convention can be
    // selected without changing NestedPid or the encoder service.
    bool sync_enabled = false;
    float sync_kp = 0.0f;
    float sync_kd = 0.0f;
    // Independent noise deadbands for the distance and velocity-difference
    // terms.  They suppress encoder quantization before the differential
    // effort limit is applied.
    float sync_position_deadband_m = 0.0f;
    float sync_velocity_deadband_mps = 0.0f;
    float sync_max_effort = 0.0f;
    float max_effort = 1.0f;
    LongitudinalLoopMode loop_mode = LongitudinalLoopMode::PITCH_ONLY;
    // The profile reaching zero is only the beginning of braking.  These
    // independent thresholds describe the measured conditions required to
    // capture a new HOLD position.
    float hold_enter_velocity_mps = 0.02f;
    float hold_exit_velocity_mps = 0.05f;
    float hold_pitch_error_deadband_deg = 1.0f;
    float hold_pitch_rate_deadband_dps = 8.0f;
    uint32_t hold_settle_time_ms = 150;
    // Reduce a fresh longitudinal velocity request when the measured pitch
    // or the previous balance effort leaves too little recovery headroom.
    // The release thresholds provide hysteresis; the policy is applied before
    // the motion profile and never changes the emergency motor cut-off.
    bool motion_request_limit_enabled = true;
    float motion_request_limit_pitch_start_deg = 4.0f;
    float motion_request_limit_pitch_full_deg = 8.0f;
    float motion_request_limit_pitch_release_deg = 3.0f;
    float motion_request_limit_effort_start = 0.75f;
    float motion_request_limit_effort_full = 0.95f;
    float motion_request_limit_effort_release = 0.65f;
    float motion_request_limit_min_scale = 0.2f;
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
               sync_enabled != other.sync_enabled ||
               sync_kp != other.sync_kp ||
               sync_kd != other.sync_kd ||
               sync_position_deadband_m != other.sync_position_deadband_m ||
               sync_velocity_deadband_mps != other.sync_velocity_deadband_mps ||
               sync_max_effort != other.sync_max_effort ||
               max_effort != other.max_effort ||
               loop_mode != other.loop_mode ||
               hold_enter_velocity_mps != other.hold_enter_velocity_mps ||
               hold_exit_velocity_mps != other.hold_exit_velocity_mps ||
               hold_pitch_error_deadband_deg != other.hold_pitch_error_deadband_deg ||
               hold_pitch_rate_deadband_dps != other.hold_pitch_rate_deadband_dps ||
               hold_settle_time_ms != other.hold_settle_time_ms ||
               motion_request_limit_enabled != other.motion_request_limit_enabled ||
               motion_request_limit_pitch_start_deg != other.motion_request_limit_pitch_start_deg ||
               motion_request_limit_pitch_full_deg != other.motion_request_limit_pitch_full_deg ||
               motion_request_limit_pitch_release_deg != other.motion_request_limit_pitch_release_deg ||
               motion_request_limit_effort_start != other.motion_request_limit_effort_start ||
               motion_request_limit_effort_full != other.motion_request_limit_effort_full ||
               motion_request_limit_effort_release != other.motion_request_limit_effort_release ||
               motion_request_limit_min_scale != other.motion_request_limit_min_scale ||
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
