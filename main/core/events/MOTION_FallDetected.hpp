// main/events/MOTION_FallDetected.hpp
#pragma once
#include "BaseEvent.hpp"

class MOTION_FallDetected : public BaseEvent {
public:
    // Add relevant data if needed, e.g., angle that triggered it
    // float triggeringAngleRad;

    MOTION_FallDetected(/* float angle */) :
        BaseEvent(EventType::MOTION_FALL_DETECTED)
        /* , triggeringAngleRad(angle) */ {}
};