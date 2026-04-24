#pragma once

#include "BaseEvent.hpp"

class COMMAND_InputModeChanged : public BaseEvent {
public:
    DECLARE_EVENT_IDENTITY(COMMAND_InputModeChanged)

    const bool acceptingInput;

    explicit COMMAND_InputModeChanged(bool acceptingInput_) :
        BaseEvent(),
        acceptingInput(acceptingInput_) {}
};
