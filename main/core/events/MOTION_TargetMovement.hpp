#pragma once
#include "BaseEvent.hpp"
#include <cstdint>

// Event published by CommandProcessor containing the *validated and final*
// target parameters for the BalancingAlgorithm.
class MOTION_TargetMovement : public BaseEvent {
public:
    DECLARE_EVENT_IDENTITY(MOTION_TargetMovement)
    // Target pitch offset in degrees (replaces linear velocity)
    const float targetPitchOffset_deg;
    // Target angular velocity in degrees per second
    const float targetAngularVelocity_dps;
    // Arm that authorized this legacy NestedPid command.  Keeping the arm on
    // the event prevents a delayed callback from an earlier control session
    // from changing the targets after a stop/rearm or strategy switch.
    const uint64_t armId;

    MOTION_TargetMovement(float pitchOffsetDeg, float angVelDPS,
                          uint64_t armId_ = 0) :
        BaseEvent(),
        targetPitchOffset_deg(pitchOffsetDeg), 
        targetAngularVelocity_dps(angVelDPS),
        armId(armId_)
        {}
};

