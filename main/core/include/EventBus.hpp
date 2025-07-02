// main/include/EventBus.hpp
#pragma once

#include <functional>               // For std::function
#include <vector>                   // For std::vector
#include <map>                      // For std::map
#include <mutex>                    // For std::mutex
#include <memory>                   // Not strictly needed here now

// Forward declarations needed in this header
enum class EventType;               // Defined in events/
class BaseEvent;                    // Defined in events/
class EventHandler;                // Event handler base class

class EventBus {
public:
    EventBus(const EventBus&) = delete;
    EventBus& operator=(const EventBus&) = delete;

    static EventBus& getInstance() {
        static EventBus instance;
        return instance;
    }

    // Alternative using shared_ptr if ownership management is needed
    void subscribe(std::shared_ptr<EventHandler> handler, const std::vector<EventType>& eventTypes);

    // Declaration only
    void publish(const BaseEvent& event);

private:
    EventBus() = default;
    ~EventBus() = default;

    void subscribe(EventType type, std::shared_ptr<EventHandler> handler);
    
    static constexpr const char* TAG = "EventBus";
    std::map<EventType, std::vector<std::shared_ptr<EventHandler>>> m_subscribers;
    std::mutex m_mutex;
};