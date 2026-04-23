// main/events/UI_Stop.hpp
#pragma once
#include "BaseEvent.hpp"

// Command event to request stopping balancing
class UI_Stop : public BaseEvent {
public:
    DECLARE_EVENT_IDENTITY(UI_Stop)
    UI_Stop() : BaseEvent() {}
};

