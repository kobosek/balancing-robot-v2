#pragma once
#include "BaseEvent.hpp"
#include "EventTypes.hpp" // Make sure TARGET_MOVEMENT_CMD_SET is defined

// Event published by CommandProcessor containing the *validated and final*
// target parameters for the BalancingAlgorithm.
class TargetMovementCommand : public BaseEvent {
public:
    // Target pitch offset in degrees (replaces linear velocity)
    const float targetPitchOffset_deg;
    // Target angular velocity in degrees per second
    const float targetAngularVelocity_dps;

    TargetMovementCommand(float pitchOffsetDeg, float angVelDPS) :
        BaseEvent(EventType::TARGET_MOVEMENT_CMD_SET),
        targetPitchOffset_deg(pitchOffsetDeg), // <<< MODIFIED
        targetAngularVelocity_dps(angVelDPS)
        {}
};