#pragma once

#include "BaseEvent.hpp"
#include "PidTuningTypes.hpp"
#include <string>

class PID_TuningFinished : public BaseEvent {
public:
    DECLARE_EVENT_IDENTITY(PID_TuningFinished)
    const PidTuningTarget target;
    const PidTuningState resultState;
    const std::string message;

    PID_TuningFinished(PidTuningTarget tuningTarget, PidTuningState result, const std::string& resultMessage)
        : BaseEvent(),
          target(tuningTarget),
          resultState(result),
          message(resultMessage) {}
};

