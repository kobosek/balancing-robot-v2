// main/events/include/IMU_CommunicationErrorEvent.hpp
#pragma once

#include "BaseEvent.hpp"
#include "EventTypes.hpp"

class IMU_CommunicationErrorEvent : public BaseEvent {
public:
    IMU_CommunicationErrorEvent() : BaseEvent(EventType::IMU_COMMUNICATION_ERROR) {}

    // Add any relevant data if needed, e.g., error code, number of failures
    // For now, the event type itself signals the condition.
};