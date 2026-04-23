#pragma once

#include "BaseEvent.hpp"
#include "PidTuningTypes.hpp"

class UI_StartPidTuning : public BaseEvent {
public:
    DECLARE_EVENT_IDENTITY(UI_StartPidTuning)
    const PidTuningTarget target;

    explicit UI_StartPidTuning(PidTuningTarget tuningTarget)
        : BaseEvent(),
          target(tuningTarget) {}
};

