#pragma once

#include "BaseEvent.hpp"

class GUIDED_CalibrationRunModeChanged : public BaseEvent {
public:
    DECLARE_EVENT_IDENTITY(GUIDED_CalibrationRunModeChanged)

    const bool active;

    explicit GUIDED_CalibrationRunModeChanged(bool active_) :
        BaseEvent(),
        active(active_) {}
};
