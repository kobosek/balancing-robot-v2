// main/events/UI_EnableAutoBalancing.hpp
#pragma once
#include "BaseEvent.hpp"

// Command event from Web UI/API to enable auto balancing.
class UI_EnableAutoBalancing : public BaseEvent {
public:
    DECLARE_EVENT_IDENTITY(UI_EnableAutoBalancing)
    UI_EnableAutoBalancing() : BaseEvent() {}
};

