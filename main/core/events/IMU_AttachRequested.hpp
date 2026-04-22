#pragma once

#include "BaseEvent.hpp"
#include "EventTypes.hpp"

class IMU_AttachRequested : public BaseEvent {
public:
    IMU_AttachRequested() : BaseEvent(EventType::IMU_ATTACH_REQUESTED) {}
};
