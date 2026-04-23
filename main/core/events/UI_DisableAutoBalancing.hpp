// main/events/UI_DisableAutoBalancing.hpp
#pragma once
#include "BaseEvent.hpp"

// Command event from Web UI/API to disable auto balancing.
class UI_DisableAutoBalancing : public BaseEvent {
public:
    DECLARE_EVENT_IDENTITY(UI_DisableAutoBalancing)
    UI_DisableAutoBalancing() : BaseEvent() {}
};

