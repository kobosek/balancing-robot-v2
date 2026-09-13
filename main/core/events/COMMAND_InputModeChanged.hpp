#pragma once

#include "BaseEvent.hpp"
#include <cstdint>

class COMMAND_InputModeChanged : public BaseEvent {
public:
    DECLARE_EVENT_IDENTITY(COMMAND_InputModeChanged)

    const bool acceptingInput;
    const uint64_t armId;

    explicit COMMAND_InputModeChanged(bool acceptingInput_, uint64_t armId_ = 0) :
        BaseEvent(),
        acceptingInput(acceptingInput_),
        armId(armId_) {}
};
