// main/events/UI_DisableFallDetection.hpp
#pragma once
#include "BaseEvent.hpp"

class UI_DisableFallDetection : public BaseEvent {
public:
    DECLARE_EVENT_IDENTITY(UI_DisableFallDetection)
    UI_DisableFallDetection() : BaseEvent() {}
};

