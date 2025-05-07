#pragma once

#include "BaseEvent.hpp"
#include "EventTypes.hpp"

class IMU_RecoverySucceeded : public BaseEvent {
public:
    IMU_RecoverySucceeded() : BaseEvent(EventType::IMU_RECOVERY_SUCCEEDED) {}
};
