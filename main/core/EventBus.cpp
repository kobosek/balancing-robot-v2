// main/EventBus.cpp
#include "EventBus.hpp"
#include "EventHandler.hpp"
#include "esp_log.h"

void EventBus::subscribe(EventKey eventKey, const char* eventName, std::shared_ptr<EventHandler> handler) {
    std::lock_guard<std::mutex> lock(m_mutex);
    HandlerList updatedHandlers;
    auto it = m_subscribers.find(eventKey);
    if (it != m_subscribers.end() && it->second) {
        updatedHandlers = *it->second;
    }
    updatedHandlers.push_back(handler);
    m_subscribers[eventKey] = std::make_shared<HandlerList>(std::move(updatedHandlers));
    ESP_LOGD(TAG, "Subscribed handler '%s' to event '%s'", handler->getHandlerName().c_str(), eventName);
}

void EventBus::publish(const BaseEvent& event) {
    HandlerListPtr handlers;
    std::unique_lock<std::mutex> lock(m_mutex);
    auto it = m_subscribers.find(event.eventKey());
    if (it != m_subscribers.end()) {
        handlers = it->second;
        lock.unlock();

        for (const auto& handler : *handlers) {
            handler->handleEvent(event);
        }
    } else {
        lock.unlock();
        ESP_LOGV(TAG, "No subscribers for event '%s'", event.eventName());
    }
}
