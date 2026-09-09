#include "control_math/PidCore.hpp"

namespace control_math {

PidCore::PidCore(const PidParameters& parameters) : m_parameters(parameters) {}

void PidCore::setParameters(const PidParameters& parameters)
{
    m_parameters = parameters;
    reset();
}

PidStepResult PidCore::finish(float pTerm, float dTerm, float error, float dt,
                              bool integrate)
{
    if (integrate) {
        const float integralDelta = (error + m_lastError) * 0.5f * dt * m_parameters.ki;
        m_integral += integralDelta;
    }
    m_integral = std::max(m_parameters.integralMin,
                          std::min(m_integral, m_parameters.integralMax));
    m_lastError = error;

    const float unclamped = pTerm + m_integral + dTerm;
    if (!std::isfinite(unclamped)) {
        return {};
    }

    const float output = std::max(m_parameters.outputMin,
                                  std::min(unclamped, m_parameters.outputMax));
    return {
        output,
        unclamped,
        pTerm,
        m_integral,
        dTerm,
        true,
        output != unclamped
    };
}

PidStepResult PidCore::compute(float setpoint, float measurement, float dt,
                               bool integrate)
{
    if (dt <= 0.0f || !std::isfinite(setpoint) ||
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
    if (dt <= 0.0f || !std::isfinite(setpoint) ||
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
    const float integralDelta = (error + m_lastError) * 0.5f * dt * m_parameters.ki;
    const float candidateIntegral = std::max(
        m_parameters.integralMin,
        std::min(m_integral + integralDelta, m_parameters.integralMax));
    const float unclamped = pTerm + candidateIntegral + dTerm;
    if (!std::isfinite(unclamped)) {
        return {};
    }

    const float output = std::max(m_parameters.outputMin,
                                  std::min(unclamped, m_parameters.outputMax));
    return {
        output,
        unclamped,
        pTerm,
        candidateIntegral,
        dTerm,
        true,
        output != unclamped
    };
}

PidStepResult PidCore::preview(float setpoint, float measurement, float dt) const
{
    if (dt <= 0.0f || !std::isfinite(setpoint) ||
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
    if (dt <= 0.0f || !std::isfinite(setpoint) ||
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
