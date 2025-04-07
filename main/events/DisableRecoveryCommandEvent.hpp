// main/events/DisableRecoveryCommandEvent.hpp
#pragma once
#include "BaseEvent.hpp"

// Command event from Web UI/API to disable auto-recovery
class DisableRecoveryCommandEvent : public BaseEvent {
public:
    DisableRecoveryCommandEvent() : BaseEvent(EventType::DISABLE_RECOVERY_COMMAND_RECEIVED) {}
};