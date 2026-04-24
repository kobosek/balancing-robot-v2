#pragma once

#include "BaseEvent.hpp"

class PID_TuningRunModeChanged : public BaseEvent {
public:
    DECLARE_EVENT_IDENTITY(PID_TuningRunModeChanged)

    const bool active;

    explicit PID_TuningRunModeChanged(bool active_) :
        BaseEvent(),
        active(active_) {}
};
