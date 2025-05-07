#pragma once
#include "BaseEvent.hpp"
#include "EventTypes.hpp" // Make sure TARGET_MOVEMENT_CMD_SET is defined

// Event published by CommandProcessor containing the *validated and final*
// target parameters for the BalancingAlgorithm.
class MOTION_TargetMovement : public BaseEvent {
public:
    // Target pitch offset in degrees (replaces linear velocity)
    const float targetPitchOffset_deg;
    // Target angular velocity in degrees per second
    const float targetAngularVelocity_dps;

    MOTION_TargetMovement(float pitchOffsetDeg, float angVelDPS) :
        BaseEvent(EventType::MOTION_TARGET_SET),
        targetPitchOffset_deg(pitchOffsetDeg), 
        targetAngularVelocity_dps(angVelDPS)
        {}
};