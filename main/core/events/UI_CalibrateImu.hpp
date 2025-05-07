// main/events/UI_CalibrateImu.hpp
#pragma once
#include "BaseEvent.hpp"

// Command event from Web UI/API to start calibration
class UI_CalibrateImu : public BaseEvent {
public:
    UI_CalibrateImu() : BaseEvent(EventType::UI_CALIBRATE_IMU) {}
};