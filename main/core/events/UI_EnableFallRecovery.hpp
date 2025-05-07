// main/events/UI_EnableFallRecovery.hpp
#pragma once
#include "BaseEvent.hpp"

// Command event from Web UI/API to enable auto-recovery
class UI_EnableFallRecovery : public BaseEvent {
public:
    UI_EnableFallRecovery() : BaseEvent(EventType::UI_ENABLE_FALL_RECOVERY) {}
};