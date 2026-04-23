#pragma once

#include "BaseEvent.hpp"
#include "SystemState.hpp"

class SYSTEM_StateChanged : public BaseEvent {
public:
    DECLARE_EVENT_IDENTITY(SYSTEM_StateChanged)
    const SystemState previousState;
    const SystemState newState;

    SYSTEM_StateChanged(SystemState prev, SystemState current) :
        BaseEvent(),
        previousState(prev),
        newState(current) {}
};

