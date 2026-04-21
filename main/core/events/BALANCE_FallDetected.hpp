// main/events/MOTION_FallDetected.hpp
#pragma once
#include "BaseEvent.hpp"

class BALANCE_FallDetected : public BaseEvent {
public:
    // Add relevant data if needed, e.g., angle that triggered it
    // float triggeringAngleRad;

    BALANCE_FallDetected(/* float angle */) :
        BaseEvent(EventType::BALANCE_FALL_DETECTED)
        /* , triggeringAngleRad(angle) */ {}
};