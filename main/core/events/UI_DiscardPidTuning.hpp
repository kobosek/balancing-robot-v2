#pragma once

#include "BaseEvent.hpp"

class UI_DiscardPidTuning : public BaseEvent {
public:
    DECLARE_EVENT_IDENTITY(UI_DiscardPidTuning)
    UI_DiscardPidTuning() : BaseEvent() {}
};

