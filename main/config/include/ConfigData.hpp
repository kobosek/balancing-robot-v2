// main/include/ConfigData.hpp
#pragma once

#include <string>
#include "driver/gpio.h" // For gpio_num_t
#include "driver/ledc.h" // For ledc_channel_t etc.
#include "driver/i2c_master.h" // For i2c_port_t

// --- Existing Structs ---
struct PIDConfig {
    float pid_kp = 0.0f;
    float pid_ki = 0.0f;
    float pid_kd = 0.0f;
    float pid_output_min = -1.0f;
    float pid_output_max = 1.0f;
    float pid_iterm_min = -1.0f;
    float pid_iterm_max = 1.0f;
};

struct WiFiConfig {
    std::string ssid = "DEFAULT_SSID";
    std::string password = "DEFAULT_PASSWORD";
};

struct MainLoopConfig {
    int interval_ms = 5;
};

// --- Updated/New Structs ---

struct MPU6050Config {
    // I2C Configuration
    i2c_port_t i2c_port = I2C_NUM_0;
    gpio_num_t sda_pin = GPIO_NUM_7; // Default Pin
    gpio_num_t scl_pin = GPIO_NUM_8; // Default Pin
    uint8_t device_address = 0x68;  // Default Address
    uint32_t i2c_freq_hz = 400000;

    // Interrupt Pin
    gpio_num_t int_pin = GPIO_NUM_9; // Default Pin
    bool interrupt_active_high = true; // Corresponds to MPU config

    // Sensor Settings (Example defaults, make match mpu6050.hpp enums if preferred)
    int accel_range = 8; // ±4g (0x08)
    int gyro_range = 8; // ±500°/s (0x08)
    int dlpf_config = 3; // BW_44HZ_ACC_42HZ_GYRO (0x03)
    int sample_rate_divisor = 0; // 1kHz (0x00)

    // Calibration & Operation
    int calibration_samples = 1000;
    float comp_filter_alpha = 0.98f; // Complementary filter coefficient
    int fifo_read_threshold = 10; // Samples before notifying task
};

struct EncoderConfig {
    // Left Encoder
    gpio_num_t left_pin_a = GPIO_NUM_11;
    gpio_num_t left_pin_b = GPIO_NUM_10;

    // Right Encoder
    gpio_num_t right_pin_a = GPIO_NUM_12;
    gpio_num_t right_pin_b = GPIO_NUM_13;

    // PCNT Settings (shared?)
    int pcnt_high_limit = 30000;
    int pcnt_low_limit = -30000;
    int pcnt_filter_ns = 1000;

    // Physical Properties (per motor/wheel)
    float pulses_per_revolution_motor = 28.0f; // Pulses from encoder itself
    float gear_ratio = 100.0f;
    float wheel_diameter_mm = 65.0f; // Example

    // Speed Filtering (if used in EncoderService)
    float speed_filter_alpha = 0.1f; // Example simple LPF alpha
};

struct MotorConfig {
    // Left Motor Pins/Channels (using MX1616H example)
    gpio_num_t left_pin_in1 = GPIO_NUM_5;
    gpio_num_t left_pin_in2 = GPIO_NUM_6;
    ledc_channel_t left_channel_1 = LEDC_CHANNEL_2;
    ledc_channel_t left_channel_2 = LEDC_CHANNEL_3;

    // Right Motor Pins/Channels
    gpio_num_t right_pin_in1 = GPIO_NUM_4;
    gpio_num_t right_pin_in2 = GPIO_NUM_3;
    ledc_channel_t right_channel_1 = LEDC_CHANNEL_0;
    ledc_channel_t right_channel_2 = LEDC_CHANNEL_1;

    // LEDC Timer/PWM Settings (assuming shared timer for now)
    ledc_timer_t timer_num = LEDC_TIMER_0;
    ledc_mode_t speed_mode = LEDC_LOW_SPEED_MODE;
    uint32_t pwm_frequency_hz = 25000;
    ledc_timer_bit_t duty_resolution = LEDC_TIMER_10_BIT;

    // Deadzone (raw duty units)
    uint32_t deadzone_duty = 500;
};

struct BatteryConfig {
    gpio_num_t adc_pin = GPIO_NUM_34; // Example ADC pin
    float voltage_divider_ratio = 2.0; // Example: Vin * (R2 / (R1+R2))
    // Add calibration points or polynomial coefficients for voltage -> percentage mapping
    float voltage_max = 4.2f;
    float voltage_min = 3.3f;
};


// --- Main ConfigData Struct ---
struct ConfigData {
    WiFiConfig wifi;
    MainLoopConfig mainLoop;
    MPU6050Config imu; // Renamed section
    EncoderConfig encoder;
    MotorConfig motor;
    BatteryConfig battery; // New section
    PIDConfig anglePid;
    PIDConfig speedPidLeft;
    PIDConfig speedPidRight;
};