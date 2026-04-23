#pragma once
#include "BaseEvent.hpp"

// Event published by CommandProcessor containing the *validated and final*
// target parameters for the BalancingAlgorithm.
class MOTION_TargetMovement : public BaseEvent {
public:
    DECLARE_EVENT_IDENTITY(MOTION_TargetMovement)
    // Target pitch offset in degrees (replaces linear velocity)
    const float targetPitchOffset_deg;
    // Target angular velocity in degrees per second
    const float targetAngularVelocity_dps;

    MOTION_TargetMovement(float pitchOffsetDeg, float angVelDPS) :
        BaseEvent(),
        targetPitchOffset_deg(pitchOffsetDeg), 
        targetAngularVelocity_dps(angVelDPS)
        {}
};

