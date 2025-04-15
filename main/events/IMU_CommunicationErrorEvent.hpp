// main/events/include/IMU_CommunicationErrorEvent.hpp
#pragma once

#include "BaseEvent.hpp"
#include "EventTypes.hpp"
#include "esp_err.h" // Include for esp_err_t

class IMU_CommunicationErrorEvent : public BaseEvent {
public:
    // Constructor now takes the error code
    explicit IMU_CommunicationErrorEvent(esp_err_t code) 
        : BaseEvent(EventType::IMU_COMMUNICATION_ERROR), errorCode(code) {}

    // Public member to hold the error code
    const esp_err_t errorCode; 
};