#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace control_math {

// Converts the normalized effort used by a balance strategy into the PWM
// magnitude expected by an H-bridge.  A non-zero effort starts at the
// measured motor deadzone and uses the remaining duty range proportionally.
// Keeping this mapping at the actuator boundary is important: strategies
// must return normalized effort and must not apply the deadzone a second time.
inline uint32_t effortToPwmDuty(float effort,
                                uint32_t maxDuty,
                                uint32_t deadzoneDuty,
                                float zeroThreshold = 1e-3f) noexcept
{
    if (!std::isfinite(effort) || !std::isfinite(zeroThreshold) ||
        maxDuty == 0 || zeroThreshold < 0.0f) {
        return 0;
    }

    const float magnitude = std::fabs(effort);
    if (magnitude <= zeroThreshold) {
        return 0;
    }

    const float clampedMagnitude = std::min(1.0f, magnitude);
    const uint32_t deadzone = std::min(deadzoneDuty, maxDuty);
    const uint32_t range = maxDuty > deadzone ? maxDuty - deadzone : 0;
    const float duty = static_cast<float>(deadzone) +
        clampedMagnitude * static_cast<float>(range);
    if (!std::isfinite(duty) || duty <= 0.0f) {
        return 0;
    }
    return std::min(maxDuty, static_cast<uint32_t>(duty));
}

} // namespace control_math
