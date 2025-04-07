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

class EventBus {
public:
    EventBus(const EventBus&) = delete;
    EventBus& operator=(const EventBus&) = delete;

    static EventBus& getInstance() {
        static EventBus instance;
        return instance;
    }

    using EventCallback = std::function<void(const BaseEvent&)>;

    // Declaration only
    void subscribe(EventType type, EventCallback callback);

    // Declaration only
    void publish(const BaseEvent& event);

private:
    EventBus() = default;
    ~EventBus() = default;

    static constexpr const char* TAG = "EventBus";
    std::map<EventType, std::vector<EventCallback>> m_subscribers;
    std::mutex m_mutex;
};