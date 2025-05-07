// main/events/UI_StartBalancing.hpp
#pragma once
#include "BaseEvent.hpp"
#include "EventTypes.hpp" // Include EventTypes

// Command event to request starting the balancing state
class UI_StartBalancing : public BaseEvent {
public:
    // Use the specific event type defined in EventTypes.hpp
    UI_StartBalancing() : BaseEvent(EventType::UI_START_BALANCING) {}
    // Add parameters if needed
};