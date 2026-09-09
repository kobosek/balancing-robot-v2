#include "MPU6050Profile.hpp"
#include "ConfigValidator.hpp"
#include <cmath>

bool ConfigValidator::validate(const ConfigData& config, std::string& error) const {
    if (config.config_version != 3) {
        error = "config_version must be 3";
        return false;
    }
    if (config.wifi.ssid.empty() || config.wifi.ssid.length() > 32) {
        error = "WiFi SSID must be 1-32 characters";
        return false;
    }
    if (!config.wifi.password.empty() && (config.wifi.password.length() < 8 || config.wifi.password.length() > 63)) {
        error = "WiFi password must be 8-63 characters (or empty for open networks)";
        return false;
    }
    if (config.mainLoop.interval_ms < 1 || config.mainLoop.interval_ms > 1000) {
        error = "mainLoop.interval_ms out of range (1-1000)";
        return false;
    }
    if (config.control.joystick_exponent < 0.5f || config.control.joystick_exponent > 5.0f) {
        error = "control.joystick_exponent out of range (0.5-5.0)";
        return false;
    }
    if (config.control.max_target_pitch_offset_deg < 0.0f || config.control.max_target_pitch_offset_deg > 90.0f) {
        error = "control.max_target_pitch_offset_deg out of range (0-90)";
        return false;
    }
    if (config.imu.i2c_port != 0 && config.imu.i2c_port != 1) {
        error = "imu.i2c_port must be 0 or 1";
        return false;
    }

    auto is_valid_gpio = [](int pin) { return pin >= 0 && pin <= 39; };
    if (!is_valid_gpio(config.imu.sda_pin) || !is_valid_gpio(config.imu.scl_pin) ||
        (config.imu.int_pin != -1 && !is_valid_gpio(config.imu.int_pin))) {
        error = "imu pins (SDA, SCL, INT) must be valid GPIOs (0-39 or -1 for INT)";
        return false;
    }
    if (config.imu.device_address < 0x08 || config.imu.device_address > 0x77) {
        error = "imu.device_address out of range (0x08-0x77)";
        return false;
    }
    if (config.imu.i2c_freq_hz < 10000 || config.imu.i2c_freq_hz > 400000) {
        error = "imu.i2c_freq_hz out of range (10k-400k)";
        return false;
    }
    if (config.imu.accel_range < 0 || config.imu.accel_range > 3) {
        error = "imu.accel_range out of range (0-3)";
        return false;
    }
    if (config.imu.gyro_range < 0 || config.imu.gyro_range > 3) {
        error = "imu.gyro_range out of range (0-3)";
        return false;
    }
    if (config.imu.dlpf_config < 0 || config.imu.dlpf_config > 6) {
        error = "imu.dlpf_config out of range (0-6)";
        return false;
    }
    if (config.imu.sample_rate_divisor < 0 || config.imu.sample_rate_divisor > 255) {
        error = "imu.sample_rate_divisor out of range (0-255)";
        return false;
    }
    if (config.imu.calibration_samples < 10 || config.imu.calibration_samples > 10000) {
        error = "imu.calibration_samples out of range (10-10000)";
        return false;
    }
    if (!std::isfinite(config.imu.gyro_offset_x) || !std::isfinite(config.imu.gyro_offset_y) ||
        !std::isfinite(config.imu.gyro_offset_z) || !std::isfinite(config.imu.comp_filter_alpha)) {
        error = "IMU offsets and filter coefficient must be finite";
        return false;
    }
    if (config.imu.comp_filter_alpha < 0.0f || config.imu.comp_filter_alpha > 1.0f) {
        error = "imu.comp_filter_alpha out of range (0.0-1.0)";
        return false;
    }
    if (config.imu.fifo_read_threshold < 1 || config.imu.fifo_read_threshold > 240) {
        error = "imu.fifo_read_threshold out of range (1-240)";
        return false;
    }
    if (!is_valid_gpio(config.encoder.left_pin_a) || !is_valid_gpio(config.encoder.left_pin_b) ||
        !is_valid_gpio(config.encoder.right_pin_a) || !is_valid_gpio(config.encoder.right_pin_b)) {
        error = "encoder pins must be valid GPIOs (0-39)";
        return false;
    }
    if (config.encoder.pcnt_high_limit < -32768 || config.encoder.pcnt_high_limit > 32767 ||
        config.encoder.pcnt_low_limit < -32768 || config.encoder.pcnt_low_limit > 32767 ||
        config.encoder.pcnt_low_limit >= 0 || config.encoder.pcnt_high_limit <= 0) {
        error = "encoder.pcnt limits out of range [-32768, 32767] or limits do not straddle zero";
        return false;
    }
    if (config.encoder.pcnt_filter_ns < 0 || config.encoder.pcnt_filter_ns > 10000) {
        error = "encoder.pcnt_filter_ns out of range (0-10000)";
        return false;
    }
    if (config.encoder.pulses_per_revolution_motor <= 0.0f || config.encoder.pulses_per_revolution_motor > 10000.0f) {
        error = "encoder.pulses_per_revolution_motor must be > 0 and reasonable (<10k)";
        return false;
    }
    if (config.encoder.gear_ratio <= 0.0f || config.encoder.gear_ratio > 1000.0f) {
        error = "encoder.gear_ratio must be > 0 and reasonable (<1k)";
        return false;
    }
    if (config.encoder.wheel_diameter_mm <= 0.0f || config.encoder.wheel_diameter_mm > 1000.0f) {
        error = "encoder.wheel_diameter_mm must be > 0 and reasonable (<1000)";
        return false;
    }
    if (config.encoder.speed_filter_alpha < 0.0f || config.encoder.speed_filter_alpha > 1.0f) {
        error = "encoder.speed_filter_alpha out of range (0.0-1.0)";
        return false;
    }
    if (!is_valid_gpio(config.motor.left_pin_in1) || !is_valid_gpio(config.motor.left_pin_in2) ||
        !is_valid_gpio(config.motor.right_pin_in1) || !is_valid_gpio(config.motor.right_pin_in2)) {
        error = "motor pins must be valid GPIOs (0-39)";
        return false;
    }
    if (config.motor.pwm_frequency_hz < 1000 || config.motor.pwm_frequency_hz > 100000) {
        error = "motor.pwm_frequency_hz out of range (1k-100k)";
        return false;
    }

    uint32_t max_duty_for_res = (1 << config.motor.duty_resolution) - 1;
    if (config.motor.deadzone_duty > max_duty_for_res) {
        error = "motor.deadzone_duty cannot exceed max duty for resolution";
        return false;
    }
    if (config.battery.adc_pin != -1 && !is_valid_gpio(config.battery.adc_pin)) {
        error = "battery.adc_pin must be valid GPIO (0-39) or -1";
        return false;
    }
    if (config.battery.voltage_divider_ratio < 0.1f || config.battery.voltage_divider_ratio > 100.0f) {
        error = "battery.voltage_divider_ratio out of range (0.1-100)";
        return false;
    }
    if (config.battery.voltage_max < 0.0f || config.battery.voltage_max > 100.0f) {
        error = "battery.voltage_max out of range (0-100)";
        return false;
    }
    if (config.battery.voltage_min < 0.0f || config.battery.voltage_min >= config.battery.voltage_max) {
        error = "battery.voltage_min out of range (0 - voltage_max)";
        return false;
    }

    auto validate_pid = [&](const PIDConfig& pid, const std::string& name) -> bool {
        if (!std::isfinite(pid.pid_kp) || !std::isfinite(pid.pid_ki) ||
            !std::isfinite(pid.pid_kd) || !std::isfinite(pid.pid_output_min) ||
            !std::isfinite(pid.pid_output_max) || !std::isfinite(pid.pid_iterm_min) ||
            !std::isfinite(pid.pid_iterm_max)) {
            error = name + " contains a non-finite value";
            return false;
        }
        if (pid.pid_kp < 0.0f || pid.pid_kp > 500.0f) {
            error = name + ".kp out of range [0, 500]";
            return false;
        }
        if (pid.pid_ki < 0.0f || pid.pid_ki > 500.0f) {
            error = name + ".ki out of range [0, 500]";
            return false;
        }
        if (pid.pid_kd < 0.0f || pid.pid_kd > 500.0f) {
            error = name + ".kd out of range [0, 500]";
            return false;
        }
        if (pid.pid_output_max < pid.pid_output_min) {
            error = name + ".output_max < output_min";
            return false;
        }
        if (pid.pid_iterm_max < pid.pid_iterm_min) {
            error = name + ".iterm_max < iterm_min";
            return false;
        }
        return true;
    };

    const auto& nested = config.control.strategies.nested_pid;
    if (config.control.max_target_pitch_offset_deg != nested.max_target_pitch_offset_deg ||
        config.control.yaw_control_enabled != nested.yaw_control_enabled) {
        error = "control compatibility fields do not match control.strategies.nested_pid";
        return false;
    }
    if (!validate_pid(nested.angle, "control.strategies.nested_pid.angle") ||
        !validate_pid(nested.speed_left, "control.strategies.nested_pid.speed_left") ||
        !validate_pid(nested.speed_right, "control.strategies.nested_pid.speed_right") ||
        !validate_pid(nested.yaw_angle, "control.strategies.nested_pid.yaw_angle") ||
        !validate_pid(nested.yaw_rate, "control.strategies.nested_pid.yaw_rate")) {
        return false;
    }

    const auto& longitudinal = config.control.strategies.longitudinal_cascade;
    if (!validate_pid(longitudinal.pitch, "control.strategies.longitudinal_cascade.pitch") ||
        !validate_pid(longitudinal.velocity, "control.strategies.longitudinal_cascade.velocity")) {
        return false;
    }
    const float longitudinalValues[] = {
        longitudinal.position_kp,
        longitudinal.pitch_trim_deg,
        longitudinal.max_pitch_offset_deg,
        longitudinal.max_pitch_rate_dps,
        longitudinal.max_velocity_mps,
        longitudinal.max_hold_velocity_mps,
        longitudinal.max_acceleration_mps2,
        longitudinal.max_deceleration_mps2,
        longitudinal.hold_position_deadband_m,
        longitudinal.hold_velocity_deadband_mps,
        longitudinal.sync_kp,
        longitudinal.sync_kd,
        longitudinal.sync_max_effort,
        longitudinal.max_effort
    };
    for (float value : longitudinalValues) {
        if (!std::isfinite(value)) {
            error = "control.strategies.longitudinal_cascade contains a non-finite value";
            return false;
        }
    }
    if (longitudinal.max_effort < 0.0f || longitudinal.max_effort > 1.0f ||
        longitudinal.sync_max_effort < 0.0f || longitudinal.sync_max_effort > 1.0f) {
        error = "control.strategies.longitudinal_cascade effort limits out of range [0,1]";
        return false;
    }
    if (longitudinal.configured) {
        if (longitudinal.max_effort <= 0.0f ||
            longitudinal.max_pitch_offset_deg <= 0.0f ||
            longitudinal.max_pitch_offset_deg > 45.0f ||
            longitudinal.max_pitch_rate_dps <= 0.0f ||
            longitudinal.max_pitch_rate_dps > 720.0f) {
            error = "configured longitudinal_cascade requires pitch and effort limits in the supported range";
            return false;
        }
        if (longitudinal.pitch.pid_output_min >= 0.0f ||
            longitudinal.pitch.pid_output_max <= 0.0f ||
            longitudinal.pitch.pid_ki != 0.0f ||
            (longitudinal.pitch.pid_kp == 0.0f &&
             longitudinal.pitch.pid_ki == 0.0f &&
             longitudinal.pitch.pid_kd == 0.0f)) {
            error = "configured longitudinal_cascade.pitch must be a signed PD controller with non-zero gain";
            return false;
        }
        if (std::fabs(longitudinal.pitch_trim_deg) > 45.0f) {
            error = "configured longitudinal_cascade.pitch_trim_deg out of range [-45,45]";
            return false;
        }
    }
    if (config.control.strategies.active == BalanceStrategyId::LONGITUDINAL_CASCADE &&
        !longitudinal.configured) {
        error = "longitudinal_cascade strategy requires configured pitch baseline";
        return false;
    }
    if (config.control.strategies.active == BalanceStrategyId::LONGITUDINAL_CASCADE) {
        if (longitudinal.max_velocity_mps <= 0.0f ||
            longitudinal.max_velocity_mps > 5.0f ||
            longitudinal.max_acceleration_mps2 <= 0.0f ||
            longitudinal.max_acceleration_mps2 > 20.0f ||
            longitudinal.max_deceleration_mps2 <= 0.0f ||
            longitudinal.max_deceleration_mps2 > 20.0f ||
            longitudinal.max_hold_velocity_mps < 0.0f ||
            longitudinal.max_hold_velocity_mps > longitudinal.max_velocity_mps) {
            error = "active longitudinal_cascade requires usable velocity and acceleration limits";
            return false;
        }
        if (longitudinal.hold_position_deadband_m < 0.0f ||
            longitudinal.hold_velocity_deadband_mps < 0.0f) {
            error = "active longitudinal_cascade HOLD deadbands must be non-negative";
            return false;
        }
        if (longitudinal.velocity.pid_output_min >= 0.0f ||
            longitudinal.velocity.pid_output_max <= 0.0f ||
            longitudinal.velocity.pid_ki <= 0.0f ||
            longitudinal.velocity.pid_kd != 0.0f ||
            (longitudinal.velocity.pid_kp == 0.0f &&
             longitudinal.velocity.pid_ki == 0.0f)) {
            error = "active longitudinal_cascade.velocity must be a signed PI controller";
            return false;
        }
        if (longitudinal.velocity.pid_output_min < -longitudinal.max_pitch_offset_deg ||
            longitudinal.velocity.pid_output_max > longitudinal.max_pitch_offset_deg) {
            error = "active longitudinal_cascade.velocity output must fit max_pitch_offset_deg";
            return false;
        }
    }

    if (config.pid_tuning.step_effort <= 0.0f || config.pid_tuning.step_effort > 1.0f) {
        error = "pid_tuning.step_effort out of range (0,1]";
        return false;
    }
    if (config.pid_tuning.max_effort < config.pid_tuning.step_effort || config.pid_tuning.max_effort > 1.0f) {
        error = "pid_tuning.max_effort must be >= step_effort and <= 1";
        return false;
    }
    if (config.pid_tuning.step_duration_ms < 100 || config.pid_tuning.step_duration_ms > 10000) {
        error = "pid_tuning.step_duration_ms [100,10000]";
        return false;
    }
    if (config.pid_tuning.rest_duration_ms < 0 || config.pid_tuning.rest_duration_ms > 10000) {
        error = "pid_tuning.rest_duration_ms [0,10000]";
        return false;
    }
    if (config.pid_tuning.min_response_dps <= 0.0f || config.pid_tuning.min_response_dps > 5000.0f) {
        error = "pid_tuning.min_response_dps (0,5000]";
        return false;
    }
    if (config.pid_tuning.max_speed_dps < config.pid_tuning.min_response_dps || config.pid_tuning.max_speed_dps > 10000.0f) {
        error = "pid_tuning.max_speed_dps must be >= min_response_dps and <= 10000";
        return false;
    }
    if (config.pid_tuning.validation_target_dps <= 0.0f || config.pid_tuning.validation_target_dps > config.pid_tuning.max_speed_dps) {
        error = "pid_tuning.validation_target_dps must be >0 and <= max_speed_dps";
        return false;
    }
    if (config.pid_tuning.gain_scale <= 0.0f || config.pid_tuning.gain_scale > 1.0f) {
        error = "pid_tuning.gain_scale (0,1]";
        return false;
    }
    if (config.behavior.joystick_deadzone < 0.0f || config.behavior.joystick_deadzone > 1.0f) {
        error = "behavior.joystick_deadzone [0,1]";
        return false;
    }
    if (config.behavior.joystick_timeout_ms < 1 || config.behavior.joystick_timeout_ms > 10000) {
        error = "behavior.joystick_timeout_ms [1,10k]";
        return false;
    }
    if (config.behavior.joystick_check_interval_ms < 1 || config.behavior.joystick_check_interval_ms > 10000) {
        error = "behavior.joystick_check_interval_ms [1,10k]";
        return false;
    }
    if (config.behavior.max_target_angular_velocity_dps <= 0.0f || config.behavior.max_target_angular_velocity_dps > 1000.0f) {
        error = "behavior.max_target_angular_velocity_dps (>0, <=1k)";
        return false;
    }
    if (config.behavior.fall_pitch_threshold_deg < 10.0f || config.behavior.fall_pitch_threshold_deg > 90.0f) {
        error = "behavior.fall_pitch_threshold_deg [10,90]";
        return false;
    }
    if (config.behavior.fall_threshold_duration_ms < 1 || config.behavior.fall_threshold_duration_ms > 10000) {
        error = "behavior.fall_threshold_duration_ms [1,10k]";
        return false;
    }
    if (config.behavior.auto_balance_pitch_threshold_deg < 0.0f || config.behavior.auto_balance_pitch_threshold_deg > 30.0f) {
        error = "behavior.auto_balance_pitch_threshold_deg [0,30]";
        return false;
    }
    if (config.behavior.auto_balance_hold_duration_ms < 1 || config.behavior.auto_balance_hold_duration_ms > 60000) {
        error = "behavior.auto_balance_hold_duration_ms [1,60k]";
        return false;
    }
    if (config.behavior.battery_oversampling_count < 1 || config.behavior.battery_oversampling_count > 1024) {
        error = "behavior.battery_oversampling_count [1,1024]";
        return false;
    }
    if (config.behavior.battery_read_interval_ms < 100 || config.behavior.battery_read_interval_ms > 60000) {
        error = "behavior.battery_read_interval_ms [100,60k]";
        return false;
    }
    if (config.dimensions.wheelbase_m <= 0.01f || config.dimensions.wheelbase_m > 1.0f) {
        error = "dimensions.wheelbase_m [0.01, 1.0]";
        return false;
    }
    if (config.web.telemetry_buffer_size < 1 || config.web.telemetry_buffer_size > 10000) {
        error = "web.telemetry_buffer_size [1,10k]";
        return false;
    }
    if (config.web.max_config_post_size < 512 || config.web.max_config_post_size > 65536) {
        error = "web.max_config_post_size [512, 64k]";
        return false;
    }
    if (config.web.log_buffer_lines < 1 || config.web.log_buffer_lines > 500) {
        error = "web.log_buffer_lines [1,500]";
        return false;
    }
    if (config.web.log_line_max_length < 32 || config.web.log_line_max_length > 512) {
        error = "web.log_line_max_length [32,512]";
        return false;
    }

    if (config.behavior.imu_max_sample_age_ms < 10 || config.behavior.imu_max_sample_age_ms > 50 ||
        config.behavior.imu_reconnect_interval_ms < 250 || config.behavior.imu_reconnect_interval_ms > 10000) {
        error = "IMU sample age must be 10-50 ms; reconnect interval 250-10000 ms";
        return false;
    }
    if (!MPU6050Profile::timingValid(config.imu, config.behavior.imu_max_sample_age_ms, config.mainLoop.interval_ms)) {
        error = "IMU sample rate, FIFO threshold, bus speed and control interval exceed freshness/transport budget";
        return false;
    }
    return true;
}
