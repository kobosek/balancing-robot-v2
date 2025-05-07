// main/events/UI_Stop.hpp
#pragma once
#include "BaseEvent.hpp"
#include "EventTypes.hpp" // Include EventTypes

// Command event to request stopping balancing
class UI_Stop : public BaseEvent {
public:
    // Use the specific event type defined in EventTypes.hpp
    UI_Stop() : BaseEvent(EventType::UI_STOP) {}
};