#pragma once

#include "BaseEvent.hpp"

class UI_CancelPidTuning : public BaseEvent {
public:
    DECLARE_EVENT_IDENTITY(UI_CancelPidTuning)
    UI_CancelPidTuning() : BaseEvent() {}
};

