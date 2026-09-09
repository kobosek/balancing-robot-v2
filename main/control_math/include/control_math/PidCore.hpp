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

    PidStepResult compute(float setpoint, float measurement, float dt);
    PidStepResult computeWithMeasurementRate(float setpoint,
                                             float measurement,
                                             float measurementRate,
                                             float dt);
    void reset();

private:
    PidParameters m_parameters;
    float m_integral = 0.0f;
    float m_lastError = 0.0f;

    PidStepResult finish(float pTerm, float dTerm, float error, float dt);
};

} // namespace control_math
