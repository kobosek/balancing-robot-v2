// main/events/UI_CalibrateImu.hpp
#pragma once
#include "BaseEvent.hpp"

// Command event from Web UI/API to start calibration
class UI_CalibrateImu : public BaseEvent {
public:
    DECLARE_EVENT_IDENTITY(UI_CalibrateImu)
    UI_CalibrateImu() : BaseEvent() {}
};

