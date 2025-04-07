// main/events/DisableFallDetectCommandEvent.hpp
#pragma once
#include "BaseEvent.hpp"
#include "EventTypes.hpp"

class DisableFallDetectCommandEvent : public BaseEvent {
public:
    DisableFallDetectCommandEvent() : BaseEvent(EventType::DISABLE_FALL_DETECT_COMMAND_RECEIVED) {}
};