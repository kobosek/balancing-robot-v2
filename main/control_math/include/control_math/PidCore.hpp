#pragma once

#include <algorithm>
#include <cmath>

namespace control_math {

// PID parameters are deliberately independent of ESP-IDF and project
// configuration types so this controller can be reused in another firmware
// or in a host-side model.
struct PidParameters {
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    float outputMin = -1.0f;
    float outputMax = 1.0f;
    float integralMin = -1.0f;
    float integralMax = 1.0f;
};

struct PidStepResult {
    float output = 0.0f;
    float unclampedOutput = 0.0f;
    float pTerm = 0.0f;
    float integralTerm = 0.0f;
    float derivativeTerm = 0.0f;
    bool valid = false;
    bool saturated = false;
};

class PidCore {
public:
    explicit PidCore(const PidParameters& parameters = {});

    void setParameters(const PidParameters& parameters);
    const PidParameters& parameters() const { return m_parameters; }

    // `integrate` is true for the legacy behaviour.  A cascade owner can
    // temporarily suppress the integral update when a downstream limit is
    // blocking the requested correction, while the error/derivative state is
    // still advanced for the next sample.
    PidStepResult compute(float setpoint, float measurement, float dt,
                          bool integrate = true);
    PidStepResult computeWithMeasurementRate(float setpoint,
                                             float measurement,
                                             float measurementRate,
                                             float dt,
                                             bool integrate = true);
    // Evaluate the next step without changing integral or derivative state.
    // The returned integral/output include the candidate trapezoidal integral,
    // which lets a cascade decide whether downstream limits require suppressing
    // that update before committing it.
    PidStepResult preview(float setpoint, float measurement, float dt) const;
    PidStepResult previewWithMeasurementRate(float setpoint,
                                             float measurement,
                                             float measurementRate,
                                             float dt) const;
    void reset();

private:
    PidParameters m_parameters;
    float m_integral = 0.0f;
    float m_lastError = 0.0f;

    PidStepResult finish(float pTerm, float dTerm, float error, float dt,
                         bool integrate);
    PidStepResult previewFinish(float pTerm, float dTerm, float error,
                                float dt) const;
};

} // namespace control_math
