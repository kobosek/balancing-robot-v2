// main/EventBus.cpp
#include "EventBus.hpp"             // Relative path within module's include dir
#include "BaseEvent.hpp"            // Found via INCLUDE_DIRS (needed for publish)
#include "EventTypes.hpp"           // Found via INCLUDE_DIRS (needed for subscribe/publish)
#include "EventHandler.hpp"         // For the new subscription methods
#include "esp_log.h"                // Moved from header
#include <map>                      // Moved from header
#include <vector>                   // Moved from header
#include <mutex>                    // Moved from header
#include <functional>               // Moved from header
#include <string>                   // For string conversion

// Helper function to convert EventType to string representation
std::string eventTypeToString(EventType type) {
    switch (type) {
        // Configuration Events
        case EventType::CONFIG_FULL_UPDATE: return "CONFIG_FULL_UPDATE";
        case EventType::CONFIG_PID_UPDATE: return "CONFIG_PID_UPDATE";
        case EventType::CONFIG_IMU_UPDATE: return "CONFIG_IMU_UPDATE";
        case EventType::CONFIG_MOTOR_UPDATE: return "CONFIG_MOTOR_UPDATE";
        case EventType::CONFIG_ENCODER_UPDATE: return "CONFIG_ENCODER_UPDATE";
        case EventType::CONFIG_BATTERY_UPDATE: return "CONFIG_BATTERY_UPDATE";
        case EventType::CONFIG_BEHAVIOR_UPDATE: return "CONFIG_BEHAVIOR_UPDATE";
        case EventType::CONFIG_WIFI_UPDATE: return "CONFIG_WIFI_UPDATE";
        case EventType::CONFIG_CONTROL_UPDATE: return "CONFIG_CONTROL_UPDATE";
        case EventType::CONFIG_GYRO_OFFSETS_UPDATE: return "CONFIG_GYRO_OFFSETS_UPDATE";
        
        // System Events
        case EventType::SYSTEM_STATE_CHANGED: return "SYSTEM_STATE_CHANGED";
        
        // Sensor Events
        case EventType::IMU_ORIENTATION_DATA: return "IMU_ORIENTATION_DATA";
        case EventType::BATTERY_STATUS_UPDATE: return "BATTERY_STATUS_UPDATE";
        
        // UI Command Events
        case EventType::UI_START_BALANCING: return "UI_START_BALANCING";
        case EventType::UI_STOP: return "UI_STOP";
        case EventType::UI_CALIBRATE_IMU: return "UI_CALIBRATE_IMU";
        case EventType::UI_ENABLE_FALL_RECOVERY: return "UI_ENABLE_FALL_RECOVERY";
        case EventType::UI_DISABLE_FALL_RECOVERY: return "UI_DISABLE_FALL_RECOVERY";
        case EventType::UI_ENABLE_FALL_DETECTION: return "UI_ENABLE_FALL_DETECTION";
        case EventType::UI_DISABLE_FALL_DETECTION: return "UI_DISABLE_FALL_DETECTION";
        case EventType::UI_JOYSTICK_INPUT: return "UI_JOYSTICK_INPUT";
        
        // Internal Requests / Notifications
        case EventType::IMU_CALIBRATION_REQUEST: return "IMU_CALIBRATION_REQUEST";
        case EventType::IMU_CALIBRATION_COMPLETED: return "IMU_CALIBRATION_COMPLETED";
        case EventType::MOTION_TARGET_SET: return "MOTION_TARGET_SET";
        
        // Diagnostic/Status Events
        case EventType::TELEMETRY_SNAPSHOT: return "TELEMETRY_SNAPSHOT";
        case EventType::BATTERY_LOW_WARNING: return "BATTERY_LOW_WARNING";
        case EventType::IMU_COMMUNICATION_ERROR: return "IMU_COMMUNICATION_ERROR";
        
        //BALANCE DETECTION
        case EventType::BALANCE_FALL_DETECTED: return "BALANCE_FALL_DETECTED";
        case EventType::BALANCE_RECOVERY_DETECTED: return "BALANCE_RECOVERY_DETECTED";

        // IMU State Management Events
        case EventType::IMU_CALIBRATION_REJECTED: return "IMU_CALIBRATION_REJECTED";
        case EventType::IMU_AVAILABILITY_CHANGED: return "IMU_AVAILABILITY_CHANGED";
        case EventType::IMU_ATTACH_REQUESTED: return "IMU_ATTACH_REQUESTED";
        
        // Error reporting
        case EventType::ERROR_REPORTED: return "ERROR_REPORTED";
        
        default: return "UNKNOWN_EVENT_TYPE";
    }
}

void EventBus::subscribe(EventType type, std::shared_ptr<EventHandler> handler) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_subscribers[type].push_back(handler);
    ESP_LOGD(TAG, "Subscribed to event type %s", eventTypeToString(type).c_str());
}

void EventBus::subscribe(std::shared_ptr<EventHandler> handler, const std::vector<EventType>& eventTypes) {
    ESP_LOGD(TAG, "Subscribing handler '%s' to %zu event types", 
             handler->getHandlerName().c_str(), eventTypes.size());
    
    for (const auto& eventType : eventTypes) {
        subscribe(eventType, handler);
        ESP_LOGD(TAG, "  Registered for event type %s", eventTypeToString(eventType).c_str());
    }
}

void EventBus::publish(const BaseEvent& event) {
    std::unique_lock<std::mutex> lock(m_mutex); // Use unique_lock
    auto it = m_subscribers.find(event.type);
    if (it != m_subscribers.end()) {
        auto handlers_copy = it->second;
        lock.unlock(); // Unlock before calling callbacks

        for (const auto& handler : handlers_copy) {
            handler->handleEvent(event);
        }
    } else {
        lock.unlock(); // Ensure unlock even if no subscribers
        ESP_LOGV(TAG, "No subscribers for event type %s", eventTypeToString(event.type).c_str());
    }
}
