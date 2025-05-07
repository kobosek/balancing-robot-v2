// main/events/IMU_CalibrationCompleted.hpp
#pragma once
#include "BaseEvent.hpp"
#include "esp_err.h" // For status

// Event published by IMUService when calibration finishes
class IMU_CalibrationCompleted : public BaseEvent {
public:
    const esp_err_t status; // ESP_OK on success, ESP_FAIL or others on error

    IMU_CalibrationCompleted(esp_err_t calib_status) :
        BaseEvent(EventType::IMU_CALIBRATION_COMPLETED),
        status(calib_status) {}
};