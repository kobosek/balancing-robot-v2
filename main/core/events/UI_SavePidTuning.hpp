#pragma once

#include "BaseEvent.hpp"

class UI_SavePidTuning : public BaseEvent {
public:
    UI_SavePidTuning() : BaseEvent(EventType::UI_SAVE_PID_TUNING) {}
};
