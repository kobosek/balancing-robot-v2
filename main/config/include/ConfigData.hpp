#pragma once

#include <string>
#include "driver/gpio.h" // For gpio_num_t
#include "driver/ledc.h" // For ledc_channel_t etc.
#include "driver/i2c_master.h" // For i2c_port_t

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
};

struct WiFiConfig {
    std::string ssid = "DEFAULT_SSID";
    std::string password = "DEFAULT_PASSWORD";
};

struct MainLoopConfig {
    int interval_ms = 5;
};

struct ControlConfig {
    float joystick_exponent = 1.5f;
    float max_target_pitch_offset_deg = 5.0f;
};

struct MPU6050Config {
    i2c_port_t i2c_port = I2C_NUM_0;
    gpio_num_t sda_pin = GPIO_NUM_7;
    gpio_num_t scl_pin = GPIO_NUM_8;
    uint8_t device_address = 0x68;
    uint32_t i2c_freq_hz = 400000;
    gpio_num_t int_pin = GPIO_NUM_9;
    bool interrupt_active_high = true;
    int accel_range = 8;
    int gyro_range = 8; // Ensure this is appropriate (e.g., 500 deg/s = 8)
    int dlpf_config = 3;
    int sample_rate_divisor = 0;
    int calibration_samples = 1000;
    float comp_filter_alpha = 0.98f;
    int fifo_read_threshold = 10;
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
};

struct BatteryConfig {
    gpio_num_t adc_pin = GPIO_NUM_2;
    float voltage_divider_ratio = 2.0;
    float voltage_max = 4.2f;
    float voltage_min = 3.3f;
};


// --- Main ConfigData Struct ---
struct ConfigData {
    WiFiConfig wifi;
    MainLoopConfig mainLoop;
    ControlConfig control;
    MPU6050Config imu;
    EncoderConfig encoder;
    MotorConfig motor;
    BatteryConfig battery;
    PIDConfig anglePid;
    PIDConfig speedPidLeft;
    PIDConfig speedPidRight;
    PIDConfig yawRatePid; // <<< ADDED Yaw Rate PID Config
};