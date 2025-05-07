// main/events/IMU_CommunicationError.hpp
#pragma once

#include "BaseEvent.hpp"
#include "EventTypes.hpp"
#include "esp_err.h" // Include for esp_err_t

class IMU_CommunicationError : public BaseEvent {
public:
    // Constructor now takes the error code
    explicit IMU_CommunicationError(esp_err_t code) 
        : BaseEvent(EventType::IMU_COMMUNICATION_ERROR), errorCode(code) {}

    // Public member to hold the error code
    const esp_err_t errorCode; 
};