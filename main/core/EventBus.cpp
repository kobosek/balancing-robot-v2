// main/EventBus.cpp
#include "EventBus.hpp"
#include "EventHandler.hpp"
#include "esp_log.h"

void EventBus::subscribe(EventKey eventKey, const char* eventName, std::shared_ptr<EventHandler> handler) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_subscribers[eventKey].push_back(handler);
    ESP_LOGD(TAG, "Subscribed handler '%s' to event '%s'", handler->getHandlerName().c_str(), eventName);
}

void EventBus::publish(const BaseEvent& event) {
    std::unique_lock<std::mutex> lock(m_mutex);
    auto it = m_subscribers.find(event.eventKey());
    if (it != m_subscribers.end()) {
        auto handlers_copy = it->second;
        lock.unlock();

        for (const auto& handler : handlers_copy) {
            handler->handleEvent(event);
        }
    } else {
        lock.unlock();
        ESP_LOGV(TAG, "No subscribers for event '%s'", event.eventName());
    }
}
