#pragma once

#include "BaseEvent.hpp"

class UI_SavePidTuning : public BaseEvent {
public:
    DECLARE_EVENT_IDENTITY(UI_SavePidTuning)
    UI_SavePidTuning() : BaseEvent() {}
};

