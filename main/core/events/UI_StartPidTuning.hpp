#pragma once

#include "BaseEvent.hpp"
#include "PidTuningTypes.hpp"

class UI_StartPidTuning : public BaseEvent {
public:
    const PidTuningTarget target;

    explicit UI_StartPidTuning(PidTuningTarget tuningTarget)
        : BaseEvent(EventType::UI_START_PID_TUNING),
          target(tuningTarget) {}
};
