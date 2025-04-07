// main/events/CalibrationCompleteEvent.hpp
#pragma once
#include "BaseEvent.hpp"
#include "esp_err.h" // For status

// Event published by IMUService when calibration finishes
class CalibrationCompleteEvent : public BaseEvent {
public:
    const esp_err_t status; // ESP_OK on success, ESP_FAIL or others on error

    CalibrationCompleteEvent(esp_err_t calib_status) :
        BaseEvent(EventType::CALIBRATION_COMPLETE),
        status(calib_status) {}
};