// main/events/UI_EnableAutoBalancing.hpp
#pragma once
#include "BaseEvent.hpp"

// Command event from Web UI/API to enable auto balancing.
class UI_EnableAutoBalancing : public BaseEvent {
public:
    UI_EnableAutoBalancing() : BaseEvent(EventType::UI_ENABLE_AUTO_BALANCING) {}
};
