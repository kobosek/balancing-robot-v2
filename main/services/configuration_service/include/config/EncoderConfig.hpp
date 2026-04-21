#pragma once

struct EncoderConfig {
    int left_pin_a = 11;
    int left_pin_b = 10;
    int right_pin_a = 12;
    int right_pin_b = 13;
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
