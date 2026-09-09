#pragma once

#include "control_math/PidCore.hpp"

namespace control_math {

struct WheelVelocityControlResult {
    float effort = 0.0f;
    float requestedEffort = 0.0f;
    bool valid = false;
    bool saturated = false;
};

// Complete single-wheel velocity servo. It owns its PID state and has no
// knowledge of encoders, motors, tasks, events or a particular robot.
class WheelVelocityController {
public:
    explicit WheelVelocityController(const PidParameters& parameters = {});

    void setParameters(const PidParameters& parameters);
    const PidParameters& parameters() const { return m_pid.parameters(); }

    WheelVelocityControlResult update(float targetSpeedDps,
                                      float measuredSpeedDps,
                                      float dtSeconds);
    WheelVelocityControlResult updateWithMeasurementRate(float targetSpeedDps,
                                                         float measuredSpeedDps,
                                                         float measuredRateDps,
                                                         float dtSeconds);
    float compute(float targetSpeedDps, float measuredSpeedDps, float dtSeconds) {
        return update(targetSpeedDps, measuredSpeedDps, dtSeconds).effort;
    }
    void reset();

private:
    PidCore m_pid;

    static WheelVelocityControlResult makeResult(const PidStepResult& result);
};

} // namespace control_math
