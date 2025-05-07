// main/events/IMU_CalibrationRequest.hpp
#pragma once
#include "BaseEvent.hpp"

// Internal event from StateManager requesting IMUService to start calibration
class IMU_CalibrationRequest : public BaseEvent {
public:
    IMU_CalibrationRequest() : BaseEvent(EventType::IMU_CALIBRATION_REQUEST) {}
};