#pragma once

#include "BaseEvent.hpp"

class UI_CancelPidTuning : public BaseEvent {
public:
    UI_CancelPidTuning() : BaseEvent(EventType::UI_CANCEL_PID_TUNING) {}
};
