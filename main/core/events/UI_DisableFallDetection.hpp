// main/events/UI_DisableFallDetection.hpp
#pragma once
#include "BaseEvent.hpp"
#include "EventTypes.hpp"

class UI_DisableFallDetection : public BaseEvent {
public:
    UI_DisableFallDetection() : BaseEvent(EventType::UI_DISABLE_FALL_DETECTION) {}
};