#pragma once

#include "BaseEvent.hpp"
#include <string>

class GUIDED_CalibrationFinished : public BaseEvent {
public:
    DECLARE_EVENT_IDENTITY(GUIDED_CalibrationFinished)
    GUIDED_CalibrationFinished(bool success, std::string message)
        : BaseEvent(),
          success(success),
          message(std::move(message)) {}

    bool success;
    std::string message;
};

