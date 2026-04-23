#pragma once

#include "BaseEvent.hpp"
#include "PidTuningTypes.hpp"
#include <string>

class PID_TuningFinished : public BaseEvent {
public:
    const PidTuningTarget target;
    const PidTuningState resultState;
    const std::string message;

    PID_TuningFinished(PidTuningTarget tuningTarget, PidTuningState result, const std::string& resultMessage)
        : BaseEvent(EventType::PID_TUNING_FINISHED),
          target(tuningTarget),
          resultState(result),
          message(resultMessage) {}
};
