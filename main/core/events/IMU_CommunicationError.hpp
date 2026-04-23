// main/events/IMU_CommunicationError.hpp
#pragma once

#include "BaseEvent.hpp"
#include "esp_err.h" // Include for esp_err_t

class IMU_CommunicationError : public BaseEvent {
public:
    DECLARE_EVENT_IDENTITY(IMU_CommunicationError)
    // Constructor now takes the error code
    explicit IMU_CommunicationError(esp_err_t code) 
        : BaseEvent(), errorCode(code) {}

    // Public member to hold the error code
    const esp_err_t errorCode; 
};

