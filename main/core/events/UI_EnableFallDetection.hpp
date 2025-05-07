// main/events/UI_EnableFallDetection.hpp
#pragma once
#include "BaseEvent.hpp"
#include "EventTypes.hpp"

class UI_EnableFallDetection : public BaseEvent {
public:
    UI_EnableFallDetection() : BaseEvent(EventType::UI_ENABLE_FALL_DETECTION) {}
};