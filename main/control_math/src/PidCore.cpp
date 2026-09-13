#include "control_math/PidCore.hpp"

namespace control_math {

PidCore::PidCore(const PidParameters& parameters) : m_parameters(parameters) {}

bool PidCore::parametersValid() const
{
    return areParametersValid(m_parameters);
}

void PidCore::setParameters(const PidParameters& parameters)
{
    (void)trySetParameters(parameters);
}

bool PidCore::trySetParameters(const PidParameters& parameters)
{
    if (!areParametersValid(parameters)) {
        return false;
    }
    m_parameters = parameters;
    reset();
    return true;
}

bool PidCore::areParametersValid(const PidParameters& parameters)
{
    return std::isfinite(parameters.kp) &&
        std::isfinite(parameters.ki) &&
        std::isfinite(parameters.kd) &&
        std::isfinite(parameters.outputMin) &&
        std::isfinite(parameters.outputMax) &&
        std::isfinite(parameters.integralMin) &&
        std::isfinite(parameters.integralMax) &&
        parameters.outputMin <= parameters.outputMax &&
        parameters.integralMin <= parameters.integralMax;
}

PidStepResult PidCore::finish(float pTerm, float dTerm, float error, float dt,
                              bool integrate)
{
    if (!parametersValid() || !std::isfinite(pTerm) || !std::isfinite(dTerm) ||
        !std::isfinite(error) || !std::isfinite(dt) || dt <= 0.0f) {
        return {};
    }

    const float integralDelta = integrate
        ? (error + m_lastError) * 0.5f * dt * m_parameters.ki
        : 0.0f;
    if (!std::isfinite(integralDelta)) {
        return {};
    }
    const float rawIntegral = m_integral + integralDelta;
    if (!std::isfinite(rawIntegral)) {
        return {};
    }
    const float candidateIntegral = std::max(
        m_parameters.integralMin,
        std::min(rawIntegral, m_parameters.integralMax));
    if (!std::isfinite(candidateIntegral)) {
        return {};
    }
    const float pPlusIntegral = pTerm + candidateIntegral;
    if (!std::isfinite(pPlusIntegral)) {
        return {};
    }
    const float unclamped = pPlusIntegral + dTerm;
    if (!std::isfinite(unclamped)) {
        return {};
    }

    const float output = std::max(m_parameters.outputMin,
                                  std::min(unclamped, m_parameters.outputMax));
    if (!std::isfinite(output)) {
        return {};
    }

    // Commit only after every intermediate value has been validated.  An
    // invalid sample must not poison the derivative or integral state used by
    // the next valid sample.
    m_integral = candidateIntegral;
    m_lastError = error;
    PidStepResult result = {
        output,
        unclamped,
        pTerm,
        m_integral,
        dTerm,
        true,
        output != unclamped,
        integralDelta
    };
    return result;
}

PidStepResult PidCore::compute(float setpoint, float measurement, float dt,
                               bool integrate)
{
    if (!parametersValid() || dt <= 0.0f || !std::isfinite(setpoint) ||
        !std::isfinite(measurement) || !std::isfinite(dt)) {
        return {};
    }

    const float error = setpoint - measurement;
    const float pTerm = m_parameters.kp * error;
    const float dTerm = dt > 1e-6f
        ? m_parameters.kd * (error - m_lastError) / dt
        : 0.0f;
    return finish(pTerm, dTerm, error, dt, integrate);
}

PidStepResult PidCore::computeWithMeasurementRate(float setpoint,
                                                  float measurement,
                                                  float measurementRate,
                                                  float dt,
                                                  bool integrate)
{
    if (!parametersValid() || dt <= 0.0f || !std::isfinite(setpoint) ||
        !std::isfinite(measurement) || !std::isfinite(measurementRate) ||
        !std::isfinite(dt)) {
        return {};
    }

    const float error = setpoint - measurement;
    const float pTerm = m_parameters.kp * error;
    const float dTerm = -m_parameters.kd * measurementRate;
    return finish(pTerm, dTerm, error, dt, integrate);
}

PidStepResult PidCore::previewFinish(float pTerm, float dTerm, float error,
                                     float dt) const
{
    if (!parametersValid() || !std::isfinite(pTerm) || !std::isfinite(dTerm) ||
        !std::isfinite(error) || !std::isfinite(dt) || dt <= 0.0f) {
        return {};
    }
    const float integralDelta = (error + m_lastError) * 0.5f * dt * m_parameters.ki;
    if (!std::isfinite(integralDelta)) {
        return {};
    }
    const float rawIntegral = m_integral + integralDelta;
    if (!std::isfinite(rawIntegral)) {
        return {};
    }
    const float candidateIntegral = std::max(
        m_parameters.integralMin,
        std::min(rawIntegral, m_parameters.integralMax));
    if (!std::isfinite(candidateIntegral)) {
        return {};
    }
    const float pPlusIntegral = pTerm + candidateIntegral;
    if (!std::isfinite(pPlusIntegral)) {
        return {};
    }
    const float unclamped = pPlusIntegral + dTerm;
    if (!std::isfinite(unclamped)) {
        return {};
    }

    const float output = std::max(m_parameters.outputMin,
                                  std::min(unclamped, m_parameters.outputMax));
    if (!std::isfinite(output)) {
        return {};
    }

    return {
        output,
        unclamped,
        pTerm,
        candidateIntegral,
        dTerm,
        true,
        output != unclamped,
        integralDelta
    };
}

PidStepResult PidCore::preview(float setpoint, float measurement, float dt) const
{
    if (!parametersValid() || dt <= 0.0f || !std::isfinite(setpoint) ||
        !std::isfinite(measurement) || !std::isfinite(dt)) {
        return {};
    }

    const float error = setpoint - measurement;
    const float pTerm = m_parameters.kp * error;
    const float dTerm = dt > 1e-6f
        ? m_parameters.kd * (error - m_lastError) / dt
        : 0.0f;
    return previewFinish(pTerm, dTerm, error, dt);
}

PidStepResult PidCore::previewWithMeasurementRate(float setpoint,
                                                  float measurement,
                                                  float measurementRate,
                                                  float dt) const
{
    if (!parametersValid() || dt <= 0.0f || !std::isfinite(setpoint) ||
        !std::isfinite(measurement) || !std::isfinite(measurementRate) ||
        !std::isfinite(dt)) {
        return {};
    }

    const float error = setpoint - measurement;
    const float pTerm = m_parameters.kp * error;
    const float dTerm = -m_parameters.kd * measurementRate;
    return previewFinish(pTerm, dTerm, error, dt);
}

void PidCore::reset()
{
    m_integral = 0.0f;
    m_lastError = 0.0f;
}

} // namespace control_math
