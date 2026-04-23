#pragma once

#include "BaseEvent.hpp"

class UI_DiscardPidTuning : public BaseEvent {
public:
    UI_DiscardPidTuning() : BaseEvent(EventType::UI_DISCARD_PID_TUNING) {}
};
