// main/events/CalibrateCommandEvent.hpp
#pragma once
#include "BaseEvent.hpp"

// Command event from Web UI/API to start calibration
class CalibrateCommandEvent : public BaseEvent {
public:
    CalibrateCommandEvent() : BaseEvent(EventType::CALIBRATE_COMMAND_RECEIVED) {}
};