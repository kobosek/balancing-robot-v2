#pragma once

#include "BaseEvent.hpp"

class UI_StartGuidedCalibration : public BaseEvent {
public:
    UI_StartGuidedCalibration() : BaseEvent(EventType::UI_START_GUIDED_CALIBRATION) {}
};
