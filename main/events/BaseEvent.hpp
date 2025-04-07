#pragma once

#include "EventTypes.hpp"
#include "esp_timer.h"

class BaseEvent {
public:
    const EventType type;
    const int64_t timestamp;

    BaseEvent(EventType eventType) :
        type(eventType),
        timestamp(esp_timer_get_time()) {}

    virtual ~BaseEvent() = default;
};