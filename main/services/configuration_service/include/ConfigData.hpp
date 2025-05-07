#pragma once

#include <string>
#include "driver/gpio.h" // For gpio_num_t
#include "driver/ledc.h" // For ledc_channel_t etc.
#include "driver/i2c_master.h" // For i2c_port_t
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
struct PIDConfig {
    float pid_kp = 0.0f;
    float pid_ki = 0.0f;
    float pid_kd = 0.0f;
    float pid_output_min = -1.0f;
    float pid_output_max = 1.0f;
    float pid_iterm_min = -1.0f;
    float pid_iterm_max = 1.0f;

    float getOutputMin() const { return pid_output_min; }
    float getOutputMax() const { return pid_output_max; }
    
    bool operator!=(const PIDConfig& other) const {
        return pid_kp != other.pid_kp ||
               pid_ki != other.pid_ki ||
               pid_kd != other.pid_kd ||
               pid_output_min != other.pid_output_min ||
               pid_output_max != other.pid_output_max ||
               pid_iterm_min != other.pid_iterm_min ||
               pid_iterm_max != other.pid_iterm_max;
    }
    
    bool operator==(const PIDConfig& other) const {
        return !(*this != other);
    }
};

struct WiFiConfig {
    std::string ssid = "DEFAULT_SSID";
    std::string password = "DEFAULT_PASSWORD";
    
    bool operator!=(const WiFiConfig& other) const {
        return ssid != other.ssid ||
               password != other.password;
    }
    
    bool operator==(const WiFiConfig& other) const {
        return !(*this != other);
    }
};

struct MainLoopConfig {
    int interval_ms = 5;
    
    bool operator!=(const MainLoopConfig& other) const {
        return interval_ms != other.interval_ms;
    }
    
    bool operator==(const MainLoopConfig& other) const {
        return !(*this != other);
    }
};

struct ControlConfig {
    float joystick_exponent = 1.5f;
    float max_target_pitch_offset_deg = 5.0f;
    
    bool operator!=(const ControlConfig& other) const {
        return joystick_exponent != other.joystick_exponent ||
               max_target_pitch_offset_deg != other.max_target_pitch_offset_deg;
    }
    
    bool operator==(const ControlConfig& other) const {
        return !(*this != other);
    }
};

struct MPU6050Config {
    // --- I2C Communication (Restart/Recovery Required on Change) ---
    i2c_port_t i2c_port = I2C_NUM_0;
    gpio_num_t sda_pin = GPIO_NUM_7;
    gpio_num_t scl_pin = GPIO_NUM_8;
    uint8_t device_address = 0x68;
    uint32_t i2c_freq_hz = 400000;

    // --- Interrupt Pin (Restart/Recovery Required on Change) ---
    gpio_num_t int_pin = GPIO_NUM_9;
    bool interrupt_active_high = true;

    // --- Hardware Sensor Settings (Restart/Recovery Required on Change) ---
    int accel_range = 1; // 0: +/-2g, 1: +/-4g, 2: +/-8g, 3: +/-16g
    int gyro_range = 1;  // 0: +/-250dps, 1: +/-500dps, 2: +/-1000dps, 3: +/-2000dps
    int dlpf_config = 3; // Digital Low Pass Filter setting (0-6)
    int sample_rate_divisor = 0; // Divides Gyro Output Rate (1kHz or 8kHz based on DLPF)

    // --- Calibration & Filtering (Partially Live Updatable) ---
    int calibration_samples = 1000; // Used during explicit calibration command
    float comp_filter_alpha = 0.98f; // Complementary filter alpha (Live Update: Yes)
    int fifo_read_threshold = 10; // Target samples per FIFO read (Live Update: Affects IMUFifoTask behavior if logic uses it, currently impacts processing limit)

    // --- Persistent Offsets (Live Update: Yes, via Calibration/Config Save) ---
    float gyro_offset_x = 0.0f; // Software offset applied after reading
    float gyro_offset_y = 0.0f;
    float gyro_offset_z = 0.0f;
    
    bool operator!=(const MPU6050Config& other) const {
        return i2c_port != other.i2c_port ||
               sda_pin != other.sda_pin ||
               scl_pin != other.scl_pin ||
               device_address != other.device_address ||
               i2c_freq_hz != other.i2c_freq_hz ||
               int_pin != other.int_pin ||
               interrupt_active_high != other.interrupt_active_high ||
               accel_range != other.accel_range ||
               gyro_range != other.gyro_range ||
               dlpf_config != other.dlpf_config ||
               sample_rate_divisor != other.sample_rate_divisor ||
               calibration_samples != other.calibration_samples ||
               comp_filter_alpha != other.comp_filter_alpha ||
               fifo_read_threshold != other.fifo_read_threshold ||
               gyro_offset_x != other.gyro_offset_x ||
               gyro_offset_y != other.gyro_offset_y ||
               gyro_offset_z != other.gyro_offset_z;
    }
    
    bool operator==(const MPU6050Config& other) const {
        return !(*this != other);
    }
    
    /**
     * Checks if hardware-related parameters have changed, which would require
     * sensor reinitialization
     */
    bool requiresHardwareInit(const MPU6050Config& other) const {
        return i2c_port != other.i2c_port ||
               sda_pin != other.sda_pin ||
               scl_pin != other.scl_pin ||
               device_address != other.device_address ||
               i2c_freq_hz != other.i2c_freq_hz ||
               int_pin != other.int_pin ||
               interrupt_active_high != other.interrupt_active_high ||
               accel_range != other.accel_range ||
               gyro_range != other.gyro_range ||
               dlpf_config != other.dlpf_config ||
               sample_rate_divisor != other.sample_rate_divisor;
    }
};

struct EncoderConfig {
    gpio_num_t left_pin_a = GPIO_NUM_11;
    gpio_num_t left_pin_b = GPIO_NUM_10;
    gpio_num_t right_pin_a = GPIO_NUM_12;
    gpio_num_t right_pin_b = GPIO_NUM_13;
    int pcnt_high_limit = 30000;
    int pcnt_low_limit = -30000;
    int pcnt_filter_ns = 1000;
    float pulses_per_revolution_motor = 28.0f;
    float gear_ratio = 100.0f;
    float wheel_diameter_mm = 65.0f;
    float speed_filter_alpha = 0.1f;
    
    bool operator!=(const EncoderConfig& other) const {
        return left_pin_a != other.left_pin_a ||
               left_pin_b != other.left_pin_b ||
               right_pin_a != other.right_pin_a ||
               right_pin_b != other.right_pin_b ||
               pcnt_high_limit != other.pcnt_high_limit ||
               pcnt_low_limit != other.pcnt_low_limit ||
               pcnt_filter_ns != other.pcnt_filter_ns ||
               pulses_per_revolution_motor != other.pulses_per_revolution_motor ||
               gear_ratio != other.gear_ratio ||
               wheel_diameter_mm != other.wheel_diameter_mm ||
               speed_filter_alpha != other.speed_filter_alpha;
    }
    
    bool operator==(const EncoderConfig& other) const {
        return !(*this != other);
    }
    
    /**
     * Checks if hardware-related parameters have changed, which would require
     * encoder reinitialization
     */
    bool requiresHardwareInit(const EncoderConfig& other) const {
        return left_pin_a != other.left_pin_a ||
               left_pin_b != other.left_pin_b ||
               right_pin_a != other.right_pin_a ||
               right_pin_b != other.right_pin_b ||
               pcnt_high_limit != other.pcnt_high_limit ||
               pcnt_low_limit != other.pcnt_low_limit ||
               pcnt_filter_ns != other.pcnt_filter_ns;
    }
};

struct MotorConfig {
    gpio_num_t left_pin_in1 = GPIO_NUM_6;
    gpio_num_t left_pin_in2 = GPIO_NUM_5;
    ledc_channel_t left_channel_1 = LEDC_CHANNEL_2;
    ledc_channel_t left_channel_2 = LEDC_CHANNEL_3;
    gpio_num_t right_pin_in1 = GPIO_NUM_3;
    gpio_num_t right_pin_in2 = GPIO_NUM_4;
    ledc_channel_t right_channel_1 = LEDC_CHANNEL_0;
    ledc_channel_t right_channel_2 = LEDC_CHANNEL_1;
    ledc_timer_t timer_num = LEDC_TIMER_0;
    ledc_mode_t speed_mode = LEDC_LOW_SPEED_MODE;
    uint32_t pwm_frequency_hz = 25000;
    ledc_timer_bit_t duty_resolution = LEDC_TIMER_10_BIT;
    uint32_t deadzone_duty = 500;
    
    bool operator!=(const MotorConfig& other) const {
        return left_pin_in1 != other.left_pin_in1 ||
               left_pin_in2 != other.left_pin_in2 ||
               left_channel_1 != other.left_channel_1 ||
               left_channel_2 != other.left_channel_2 ||
               right_pin_in1 != other.right_pin_in1 ||
               right_pin_in2 != other.right_pin_in2 ||
               right_channel_1 != other.right_channel_1 ||
               right_channel_2 != other.right_channel_2 ||
               timer_num != other.timer_num ||
               speed_mode != other.speed_mode ||
               pwm_frequency_hz != other.pwm_frequency_hz ||
               duty_resolution != other.duty_resolution ||
               deadzone_duty != other.deadzone_duty;
    }
    
    bool operator==(const MotorConfig& other) const {
        return !(*this != other);
    }
    
    /**
     * Checks if hardware-related parameters have changed, which would require
     * motor reinitialization
     */
    bool requiresHardwareInit(const MotorConfig& other) const {
        return left_pin_in1 != other.left_pin_in1 ||
               left_pin_in2 != other.left_pin_in2 ||
               left_channel_1 != other.left_channel_1 ||
               left_channel_2 != other.left_channel_2 ||
               right_pin_in1 != other.right_pin_in1 ||
               right_pin_in2 != other.right_pin_in2 ||
               right_channel_1 != other.right_channel_1 ||
               right_channel_2 != other.right_channel_2 ||
               timer_num != other.timer_num ||
               speed_mode != other.speed_mode ||
               pwm_frequency_hz != other.pwm_frequency_hz ||
               duty_resolution != other.duty_resolution;
    }
};

struct BatteryConfig {
    gpio_num_t adc_pin = GPIO_NUM_2;
    float voltage_divider_ratio = 2.0;
    float voltage_max = 4.2f;
    float voltage_min = 3.3f;
    // Moved from BatteryService constants
    adc_bitwidth_t adc_bitwidth = ADC_BITWIDTH_12;
    adc_atten_t adc_atten = ADC_ATTEN_DB_12;
    
    bool operator!=(const BatteryConfig& other) const {
        return adc_pin != other.adc_pin ||
               voltage_divider_ratio != other.voltage_divider_ratio ||
               voltage_max != other.voltage_max ||
               voltage_min != other.voltage_min ||
               adc_bitwidth != other.adc_bitwidth ||
               adc_atten != other.adc_atten;
    }
    
    bool operator==(const BatteryConfig& other) const {
        return !(*this != other);
    }
    
    /**
     * Checks if hardware-related parameters have changed, which would require
     * ADC reinitialization
     */
    bool requiresHardwareInit(const BatteryConfig& other) const {
        return adc_pin != other.adc_pin ||
               adc_bitwidth != other.adc_bitwidth ||
               adc_atten != other.adc_atten;
    }
};

// --- System Behavior Config ---
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
    int imu_recovery_delay_ms = 1000; // Note: Delay isn't explicitly used in StateManager currently
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

// --- Robot Dimensions Config ---
struct RobotDimensionsConfig {
    float wheelbase_m = 0.15f;
    // wheel_diameter_mm is already in EncoderConfig
    
    bool operator!=(const RobotDimensionsConfig& other) const {
        return wheelbase_m != other.wheelbase_m;
    }
    
    bool operator==(const RobotDimensionsConfig& other) const {
        return !(*this != other);
    }
};

// --- Web Server Config ---
struct WebServerConfig {
    int telemetry_buffer_size = 100;
    int max_config_post_size = 4096;
    
    bool operator!=(const WebServerConfig& other) const {
        return telemetry_buffer_size != other.telemetry_buffer_size ||
               max_config_post_size != other.max_config_post_size;
    }
    
    bool operator==(const WebServerConfig& other) const {
        return !(*this != other);
    }
};

// --- Main ConfigData Struct ---
struct ConfigData {
    int config_version = 1; // For future schema upgrades
    WiFiConfig wifi;
    MainLoopConfig mainLoop;
    ControlConfig control;
    MPU6050Config imu;
    EncoderConfig encoder;
    MotorConfig motor;
    BatteryConfig battery;
    PIDConfig pid_angle;
    PIDConfig pid_speed_left;
    PIDConfig pid_speed_right;
    PIDConfig pid_yaw_rate;
    SystemBehaviorConfig behavior;
    RobotDimensionsConfig dimensions;
    WebServerConfig web;
};

struct IMURecoveryConfig {
    bool minimal_interruption;    // Try to minimize interruption (true in BALANCING state)
    int max_attempts;             // Maximum number of attempts for this recovery session
    int retry_delay_ms;           // Delay between retry attempts
};