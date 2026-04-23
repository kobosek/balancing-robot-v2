#pragma once

#include "BaseEvent.hpp"
#include <string>

class GUIDED_CalibrationFinished : public BaseEvent {
public:
    GUIDED_CalibrationFinished(bool success, std::string message)
        : BaseEvent(EventType::GUIDED_CALIBRATION_FINISHED),
          success(success),
          message(std::move(message)) {}

    bool success;
    std::string message;
};
