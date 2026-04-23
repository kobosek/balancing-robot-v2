// main/events/UI_EnableFallDetection.hpp
#pragma once
#include "BaseEvent.hpp"

class UI_EnableFallDetection : public BaseEvent {
public:
    DECLARE_EVENT_IDENTITY(UI_EnableFallDetection)
    UI_EnableFallDetection() : BaseEvent() {}
};

