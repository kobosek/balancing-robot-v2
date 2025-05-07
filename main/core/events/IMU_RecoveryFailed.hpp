#pragma once

#include "BaseEvent.hpp"
#include "EventTypes.hpp"
#include "esp_err.h"

class IMU_RecoveryFailed : public BaseEvent {
public:
    IMU_RecoveryFailed(esp_err_t error_code = ESP_FAIL) 
        : BaseEvent(EventType::IMU_RECOVERY_FAILED), 
          errorCode(error_code) {}
    
    esp_err_t errorCode;
};
