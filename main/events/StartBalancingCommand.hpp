// main/include/StartBalancingCommand.hpp
#pragma once
#include "BaseEvent.hpp"
#include "EventTypes.hpp" // Include EventTypes

// Command event to request starting the balancing state
class StartBalancingCommand : public BaseEvent {
public:
    // Use the specific event type defined in EventTypes.hpp
    StartBalancingCommand() : BaseEvent(EventType::START_COMMAND_RECEIVED) {}
    // Add parameters if needed
};