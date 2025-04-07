// main/events/EnableFallDetectCommandEvent.hpp
#pragma once
#include "BaseEvent.hpp"
#include "EventTypes.hpp"

class EnableFallDetectCommandEvent : public BaseEvent {
public:
    EnableFallDetectCommandEvent() : BaseEvent(EventType::ENABLE_FALL_DETECT_COMMAND_RECEIVED) {}
};