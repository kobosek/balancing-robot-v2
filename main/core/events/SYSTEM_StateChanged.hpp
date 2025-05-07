#pragma once

#include "BaseEvent.hpp"
#include "SystemState.hpp"

class SYSTEM_StateChanged : public BaseEvent {
public:
    const SystemState previousState;
    const SystemState newState;

    SYSTEM_StateChanged(SystemState prev, SystemState current) :
        BaseEvent(EventType::SYSTEM_STATE_CHANGED),
        previousState(prev),
        newState(current) {}
};