#pragma once

#include "BaseEvent.hpp"

class UI_StartGuidedCalibration : public BaseEvent {
public:
    DECLARE_EVENT_IDENTITY(UI_StartGuidedCalibration)
    UI_StartGuidedCalibration() : BaseEvent() {}
};

