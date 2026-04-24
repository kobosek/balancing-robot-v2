// main/include/EventBus.hpp
#pragma once

#include <functional>
#include <vector>
#include <map>
#include <mutex>
#include <memory>
#include <utility>

#include "BaseEvent.hpp"

class EventHandler;

class EventBus {
public:
    EventBus(const EventBus&) = delete;
    EventBus& operator=(const EventBus&) = delete;

    static EventBus& getInstance() {
        static EventBus instance;
        return instance;
    }

    template <typename... EventTs>
    void subscribe(std::shared_ptr<EventHandler> handler) {
        (subscribe(EventTs::staticEventKey(), EventTs::staticEventName(), handler), ...);
    }

    void publish(const BaseEvent& event);

private:
    using HandlerList = std::vector<std::shared_ptr<EventHandler>>;
    using HandlerListPtr = std::shared_ptr<const HandlerList>;

    EventBus() = default;
    ~EventBus() = default;

    void subscribe(EventKey eventKey, const char* eventName, std::shared_ptr<EventHandler> handler);
    
    static constexpr const char* TAG = "EventBus";
    std::map<EventKey, HandlerListPtr> m_subscribers;
    std::mutex m_mutex;
};
