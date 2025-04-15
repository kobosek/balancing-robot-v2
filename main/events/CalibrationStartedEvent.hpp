// main/events/CalibrationStartedEvent.hpp
#pragma once

#include "BaseEvent.hpp"
#include "EventTypes.hpp"

class CalibrationStartedEvent : public BaseEvent {
public:
    CalibrationStartedEvent() : BaseEvent(EventType::CALIBRATION_STARTED) {}
    virtual ~CalibrationStartedEvent() = default;
};
