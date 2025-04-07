// main/EventBus.cpp
#include "EventBus.hpp"             // Relative path within module's include dir
#include "BaseEvent.hpp"            // Found via INCLUDE_DIRS (needed for publish)
#include "EventTypes.hpp"           // Found via INCLUDE_DIRS (needed for subscribe/publish)
#include "esp_log.h"                // Moved from header
#include <map>                      // Moved from header
#include <vector>                   // Moved from header
#include <mutex>                    // Moved from header
#include <functional>               // Moved from header

void EventBus::subscribe(EventType type, EventCallback callback) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_subscribers[type].push_back(callback);
    ESP_LOGD(TAG, "Subscribed to event type %d", static_cast<int>(type));
}

void EventBus::publish(const BaseEvent& event) {
    std::unique_lock<std::mutex> lock(m_mutex); // Use unique_lock
    auto it = m_subscribers.find(event.type);
    if (it != m_subscribers.end()) {
        // Copy the callbacks to avoid issues if a callback unsubscribes itself
        auto callbacks_copy = it->second;
        lock.unlock(); // Unlock before calling callbacks

        ESP_LOGV(TAG, "Publishing event type %d to %d subscribers",
                 static_cast<int>(event.type), callbacks_copy.size());

        for (const auto& callback : callbacks_copy) {
            // Removed try-catch block
            callback(event);
        }
    } else {
        lock.unlock(); // Ensure unlock even if no subscribers
        ESP_LOGV(TAG, "No subscribers for event type %d", static_cast<int>(event.type));
    }
}