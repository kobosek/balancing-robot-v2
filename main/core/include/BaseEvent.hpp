#pragma once

#include "esp_timer.h"
#include <type_traits>

using EventKey = const void*;

class BaseEvent {
public:
    const int64_t timestamp;

    BaseEvent() :
        timestamp(esp_timer_get_time()) {}

    virtual ~BaseEvent() = default;

    virtual EventKey eventKey() const = 0;
    virtual const char* eventName() const = 0;

    template <typename EventT>
    bool is() const {
        return eventKey() == EventT::staticEventKey();
    }

    template <typename EventT>
    const EventT& as() const {
        return static_cast<const EventT&>(*this);
    }
};

#define DECLARE_EVENT_IDENTITY(EventClass) \
public: \
    static EventKey staticEventKey() { \
        static const int token = 0; \
        return &token; \
    } \
    EventKey eventKey() const override { return staticEventKey(); } \
    static constexpr const char* staticEventName() { return #EventClass; } \
    const char* eventName() const override { return staticEventName(); }
