// main/events/IMU_CalibrationStarted.hpp
#pragma once

#include "BaseEvent.hpp"
#include "EventTypes.hpp"

class IMU_CalibrationStarted : public BaseEvent {
public:
    IMU_CalibrationStarted() : BaseEvent(EventType::IMU_CALIBRATION_STARTED) {}
    virtual ~IMU_CalibrationStarted() = default;
};
