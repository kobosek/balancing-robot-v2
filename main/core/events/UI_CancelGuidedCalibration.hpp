#pragma once

#include "BaseEvent.hpp"

class UI_CancelGuidedCalibration : public BaseEvent {
public:
    UI_CancelGuidedCalibration() : BaseEvent(EventType::UI_CANCEL_GUIDED_CALIBRATION) {}
};
