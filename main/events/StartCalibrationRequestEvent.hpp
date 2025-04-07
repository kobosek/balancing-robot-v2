// main/events/StartCalibrationRequestEvent.hpp
#pragma once
#include "BaseEvent.hpp"

// Internal event from StateManager requesting IMUService to start calibration
class StartCalibrationRequestEvent : public BaseEvent {
public:
    StartCalibrationRequestEvent() : BaseEvent(EventType::START_CALIBRATION_REQUEST) {}
};