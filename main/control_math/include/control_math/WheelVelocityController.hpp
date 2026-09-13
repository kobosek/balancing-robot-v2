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
    bool trySetParameters(const PidParameters& parameters);
    const PidParameters& parameters() const { return m_pid.parameters(); }

    WheelVelocityControlResult update(float targetSpeedDps,
                                      float measuredSpeedDps,
                                      float dtSeconds);
    // Evaluate a candidate without changing the wheel PID state.  Strategy
    // adapters can validate both wheels before committing either controller,
    // keeping a rejected sample from partially advancing the cascade.
    WheelVelocityControlResult preview(float targetSpeedDps,
                                       float measuredSpeedDps,
                                       float dtSeconds) const;
    WheelVelocityControlResult updateWithMeasurementRate(float targetSpeedDps,
                                                         float measuredSpeedDps,
                                                         float measuredRateDps,
                                                         float dtSeconds);
    WheelVelocityControlResult previewWithMeasurementRate(float targetSpeedDps,
                                                          float measuredSpeedDps,
                                                          float measuredRateDps,
                                                          float dtSeconds) const;
    float compute(float targetSpeedDps, float measuredSpeedDps, float dtSeconds) {
        return update(targetSpeedDps, measuredSpeedDps, dtSeconds).effort;
    }
    void reset();

private:
    PidCore m_pid;

    static WheelVelocityControlResult makeResult(const PidStepResult& result);
};

} // namespace control_math
