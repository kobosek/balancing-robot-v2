#pragma once

#include <cstdint>

struct MotorConfig {
    int left_pin_in1 = 6;
    int left_pin_in2 = 5;
    int left_channel_1 = 2;
    int left_channel_2 = 3;
    int right_pin_in1 = 3;
    int right_pin_in2 = 4;
    int right_channel_1 = 0;
    int right_channel_2 = 1;
    int timer_num = 0;
    int speed_mode = 0;       // LEDC_LOW_SPEED_MODE
    uint32_t pwm_frequency_hz = 25000;
    int duty_resolution = 10; // LEDC_TIMER_10_BIT
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
