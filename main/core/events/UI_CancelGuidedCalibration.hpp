#pragma once

#include "BaseEvent.hpp"

class UI_CancelGuidedCalibration : public BaseEvent {
public:
    DECLARE_EVENT_IDENTITY(UI_CancelGuidedCalibration)
    UI_CancelGuidedCalibration() : BaseEvent() {}
};

