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
    EventBus() = default;
    ~EventBus() = default;

    void subscribe(EventKey eventKey, const char* eventName, std::shared_ptr<EventHandler> handler);
    
    static constexpr const char* TAG = "EventBus";
    std::map<EventKey, std::vector<std::shared_ptr<EventHandler>>> m_subscribers;
    std::mutex m_mutex;
};
