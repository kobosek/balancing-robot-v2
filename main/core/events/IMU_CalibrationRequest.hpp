// main/events/IMU_CalibrationRequest.hpp
#pragma once
#include "BaseEvent.hpp"

// Internal event from StateManager requesting IMUService to start calibration
class IMU_CalibrationRequest : public BaseEvent {
public:
    DECLARE_EVENT_IDENTITY(IMU_CalibrationRequest)
    IMU_CalibrationRequest() : BaseEvent() {}
};

