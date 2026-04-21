// main/events/MOTION_FallDetected.hpp
#pragma once
#include "BaseEvent.hpp"

class BALANCE_RecoveryDetected : public BaseEvent {
public:
    // Add relevant data if needed, e.g., angle that triggered it
    // float triggeringAngleRad;

    BALANCE_RecoveryDetected(/* float angle */) :
        BaseEvent(EventType::BALANCE_RECOVERY_DETECTED)
        /* , triggeringAngleRad(angle) */ {}
};