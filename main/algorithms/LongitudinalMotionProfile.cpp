#include "LongitudinalMotionProfile.hpp"

#include <algorithm>

namespace {
float clampSymmetric(float value, float limit)
{
    if (!std::isfinite(value) || !std::isfinite(limit) || limit <= 0.0f) {
        return 0.0f;
    }
    return std::max(-limit, std::min(limit, value));
}
}

LongitudinalMotionProfile::LongitudinalMotionProfile(
    const LongitudinalMotionProfileConfig& config)
{
    configure(config);
}

void LongitudinalMotionProfile::configure(
    const LongitudinalMotionProfileConfig& config)
{
    m_config = config;
    if (!std::isfinite(m_config.maxVelocityMps) || m_config.maxVelocityMps <= 0.0f) {
        m_config.maxVelocityMps = 0.0f;
    }
    if (!std::isfinite(m_config.maxAccelerationMps2) ||
        m_config.maxAccelerationMps2 <= 0.0f) {
        m_config.maxAccelerationMps2 = 0.0f;
    }
    if (!std::isfinite(m_config.maxDecelerationMps2) ||
        m_config.maxDecelerationMps2 <= 0.0f) {
        m_config.maxDecelerationMps2 = 0.0f;
    }
    m_targetVelocityMps = clampCommand(m_targetVelocityMps);
}

void LongitudinalMotionProfile::reset(float velocityMps)
{
    m_targetVelocityMps = clampCommand(velocityMps);
}

float LongitudinalMotionProfile::clampCommand(float commandVelocityMps) const
{
    return clampSymmetric(commandVelocityMps, m_config.maxVelocityMps);
}

float LongitudinalMotionProfile::advanceToward(float current,
                                               float target,
                                               float rate,
                                               float dtSeconds,
                                               bool& limited)
{
    if (!std::isfinite(current) || !std::isfinite(target) ||
        !std::isfinite(rate) || !std::isfinite(dtSeconds) ||
        dtSeconds <= 0.0f || rate <= 0.0f) {
        limited = true;
        return current;
    }

    const float delta = target - current;
    const float maxStep = rate * dtSeconds;
    if (std::fabs(delta) <= maxStep) {
        return target;
    }
    limited = true;
    return current + std::copysign(maxStep, delta);
}

LongitudinalMotionProfileResult LongitudinalMotionProfile::update(
    float commandVelocityMps,
    bool commandActive,
    float dtSeconds)
{
    LongitudinalMotionProfileResult result = {};
    result.targetVelocityMps = m_targetVelocityMps;

    if (!std::isfinite(commandVelocityMps) ||
        !std::isfinite(dtSeconds) || dtSeconds <= 0.0f ||
        m_config.maxVelocityMps <= 0.0f ||
        m_config.maxAccelerationMps2 <= 0.0f ||
        m_config.maxDecelerationMps2 <= 0.0f) {
        return result;
    }

    const float requested = commandActive ? clampCommand(commandVelocityMps) : 0.0f;
    bool limited = requested != commandVelocityMps && commandActive;
    float target = requested;

    // A direction reversal always passes through zero at the deceleration
    // limit. This avoids an instantaneous sign change in the velocity loop.
    if (m_targetVelocityMps != 0.0f && requested != 0.0f &&
        std::signbit(m_targetVelocityMps) != std::signbit(requested)) {
        target = 0.0f;
        limited = true;
    }

    const bool accelerating = std::fabs(target) > std::fabs(m_targetVelocityMps);
    const float rate = accelerating
        ? m_config.maxAccelerationMps2
        : m_config.maxDecelerationMps2;
    m_targetVelocityMps = advanceToward(m_targetVelocityMps, target, rate,
                                        dtSeconds, limited);

    result.targetVelocityMps = m_targetVelocityMps;
    result.valid = true;
    result.limited = limited;
    result.atRest = std::fabs(m_targetVelocityMps) <= 1e-5f;
    result.reachedCommand = !commandActive
        ? result.atRest
        : std::fabs(m_targetVelocityMps - requested) <= 1e-5f;
    return result;
}
