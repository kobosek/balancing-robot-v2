#pragma once

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
