// main/include/StopCommand.hpp
#pragma once
#include "BaseEvent.hpp"
#include "EventTypes.hpp" // Include EventTypes

// Command event to request stopping balancing
class StopCommand : public BaseEvent {
public:
    // Use the specific event type defined in EventTypes.hpp
    StopCommand() : BaseEvent(EventType::STOP_COMMAND_RECEIVED) {}
};