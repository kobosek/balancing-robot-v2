// main/events/EnableRecoveryCommandEvent.hpp
#pragma once
#include "BaseEvent.hpp"

// Command event from Web UI/API to enable auto-recovery
class EnableRecoveryCommandEvent : public BaseEvent {
public:
    EnableRecoveryCommandEvent() : BaseEvent(EventType::ENABLE_RECOVERY_COMMAND_RECEIVED) {}
};