#include "control_math/WheelVelocityController.hpp"

namespace control_math {

WheelVelocityController::WheelVelocityController(const PidParameters& parameters)
    : m_pid(parameters) {}

void WheelVelocityController::setParameters(const PidParameters& parameters)
{
    m_pid.setParameters(parameters);
}

WheelVelocityControlResult WheelVelocityController::makeResult(const PidStepResult& result)
{
    return {
        result.output,
        result.unclampedOutput,
        result.valid,
        result.saturated
    };
}

WheelVelocityControlResult WheelVelocityController::update(float targetSpeedDps,
                                                            float measuredSpeedDps,
                                                            float dtSeconds)
{
    return makeResult(m_pid.compute(targetSpeedDps, measuredSpeedDps, dtSeconds));
}

WheelVelocityControlResult WheelVelocityController::updateWithMeasurementRate(float targetSpeedDps,
                                                                                float measuredSpeedDps,
                                                                                float measuredRateDps,
                                                                                float dtSeconds)
{
    return makeResult(m_pid.computeWithMeasurementRate(
        targetSpeedDps, measuredSpeedDps, measuredRateDps, dtSeconds));
}

void WheelVelocityController::reset()
{
    m_pid.reset();
}

} // namespace control_math
