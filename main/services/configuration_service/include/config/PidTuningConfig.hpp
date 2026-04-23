#pragma once

struct PidTuningConfig {
    float step_effort = 0.35f;
    float max_effort = 0.45f;
    int step_duration_ms = 1500;
    int rest_duration_ms = 500;
    float min_response_dps = 30.0f;
    float max_speed_dps = 1200.0f;
    float validation_target_dps = 360.0f;
    float gain_scale = 0.5f;

    bool operator!=(const PidTuningConfig& other) const {
        return step_effort != other.step_effort ||
               max_effort != other.max_effort ||
               step_duration_ms != other.step_duration_ms ||
               rest_duration_ms != other.rest_duration_ms ||
               min_response_dps != other.min_response_dps ||
               max_speed_dps != other.max_speed_dps ||
               validation_target_dps != other.validation_target_dps ||
               gain_scale != other.gain_scale;
    }

    bool operator==(const PidTuningConfig& other) const {
        return !(*this != other);
    }
};
