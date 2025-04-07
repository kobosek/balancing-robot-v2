#pragma once

#include "BaseEvent.hpp"
#include "SystemState.hpp"

class SystemStateChangedEvent : public BaseEvent {
public:
    const SystemState previousState;
    const SystemState newState;

    SystemStateChangedEvent(SystemState prev, SystemState current) :
        BaseEvent(EventType::SYSTEM_STATE_CHANGED),
        previousState(prev),
        newState(current) {}
};