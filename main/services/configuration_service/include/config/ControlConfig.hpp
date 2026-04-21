#pragma once

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
