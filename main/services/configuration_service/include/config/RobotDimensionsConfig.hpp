#pragma once

struct RobotDimensionsConfig {
    float wheelbase_m = 0.15f;

    bool operator!=(const RobotDimensionsConfig& other) const {
        return wheelbase_m != other.wheelbase_m;
    }

    bool operator==(const RobotDimensionsConfig& other) const {
        return !(*this != other);
    }
};
