#pragma once

struct SystemBehaviorConfig {
    float joystick_deadzone = 0.10f;
    int joystick_timeout_ms = 500;
    int joystick_check_interval_ms = 100;
    float max_target_angular_velocity_dps = 60.0f;
    float fall_pitch_threshold_deg = 45.0f;
    int fall_threshold_duration_ms = 500;
    float recovery_pitch_threshold_deg = 5.0f;
    int recovery_hold_duration_ms = 2000;
    int imu_recovery_max_attempts = 3;
    int imu_recovery_delay_ms = 1000;
    int battery_oversampling_count = 64;
    int battery_read_interval_ms = 5000;
    int imu_health_i2c_fail_threshold = 5;
    int imu_health_no_data_threshold = 5;
    int imu_health_data_timeout_ms = 500;
    int imu_health_proactive_check_ms = 10000;

    bool operator!=(const SystemBehaviorConfig& other) const {
        return joystick_deadzone != other.joystick_deadzone ||
               joystick_timeout_ms != other.joystick_timeout_ms ||
               joystick_check_interval_ms != other.joystick_check_interval_ms ||
               max_target_angular_velocity_dps != other.max_target_angular_velocity_dps ||
               fall_pitch_threshold_deg != other.fall_pitch_threshold_deg ||
               fall_threshold_duration_ms != other.fall_threshold_duration_ms ||
               recovery_pitch_threshold_deg != other.recovery_pitch_threshold_deg ||
               recovery_hold_duration_ms != other.recovery_hold_duration_ms ||
               imu_recovery_max_attempts != other.imu_recovery_max_attempts ||
               imu_recovery_delay_ms != other.imu_recovery_delay_ms ||
               battery_oversampling_count != other.battery_oversampling_count ||
               battery_read_interval_ms != other.battery_read_interval_ms ||
               imu_health_i2c_fail_threshold != other.imu_health_i2c_fail_threshold ||
               imu_health_no_data_threshold != other.imu_health_no_data_threshold ||
               imu_health_data_timeout_ms != other.imu_health_data_timeout_ms ||
               imu_health_proactive_check_ms != other.imu_health_proactive_check_ms;
    }

    bool operator==(const SystemBehaviorConfig& other) const {
        return !(*this != other);
    }
};
