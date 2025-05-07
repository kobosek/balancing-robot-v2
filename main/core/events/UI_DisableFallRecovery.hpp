// main/events/UI_DisableFallRecovery.hpp
#pragma once
#include "BaseEvent.hpp"

// Command event from Web UI/API to disable auto-recovery
class UI_DisableFallRecovery : public BaseEvent {
public:
    UI_DisableFallRecovery() : BaseEvent(EventType::UI_DISABLE_FALL_RECOVERY) {}
};