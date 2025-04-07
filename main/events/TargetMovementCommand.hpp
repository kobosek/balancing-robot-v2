// main/events/TargetMovementCommand.hpp
#pragma once
#include "BaseEvent.hpp"
#include "EventTypes.hpp" // Make sure TARGET_MOVEMENT_CMD_SET is defined

// Event published by CommandProcessor containing the *validated and final*
// target velocities for the BalancingAlgorithm.
class TargetMovementCommand : public BaseEvent {
public:
    // Target linear velocity in meters per second
    const float targetLinearVelocity_mps;
    // Target angular velocity in degrees per second <<<--- Standardized unit
    const float targetAngularVelocity_dps;

    TargetMovementCommand(float linVel, float angVelDPS) :
        BaseEvent(EventType::TARGET_MOVEMENT_CMD_SET),
        targetLinearVelocity_mps(linVel),
        targetAngularVelocity_dps(angVelDPS)
        {}
};