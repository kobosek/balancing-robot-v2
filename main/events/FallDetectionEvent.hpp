// main/include/FallDetectionEvent.hpp
#pragma once
#include "BaseEvent.hpp"

class FallDetectionEvent : public BaseEvent {
public:
    // Add relevant data if needed, e.g., angle that triggered it
    // float triggeringAngleRad;

    FallDetectionEvent(/* float angle */) :
        BaseEvent(EventType::FALL_DETECTED)
        /* , triggeringAngleRad(angle) */ {}
};