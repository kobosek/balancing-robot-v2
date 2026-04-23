// main/events/UI_StartBalancing.hpp
#pragma once
#include "BaseEvent.hpp"

// Command event to request starting the balancing state
class UI_StartBalancing : public BaseEvent {
public:
    DECLARE_EVENT_IDENTITY(UI_StartBalancing)
    UI_StartBalancing() : BaseEvent() {}
    // Add parameters if needed
};

