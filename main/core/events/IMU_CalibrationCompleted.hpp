// main/events/IMU_CalibrationCompleted.hpp
#pragma once
#include "BaseEvent.hpp"
#include "esp_err.h" // For status

// Event published by IMUService when calibration finishes
class IMU_CalibrationCompleted : public BaseEvent {
public:
    DECLARE_EVENT_IDENTITY(IMU_CalibrationCompleted)
    const esp_err_t status; // ESP_OK on success, ESP_FAIL or others on error

    IMU_CalibrationCompleted(esp_err_t calib_status) :
        BaseEvent(),
        status(calib_status) {}
};

