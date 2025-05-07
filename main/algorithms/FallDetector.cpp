#include "FallDetector.hpp"
#include "MOTION_FallDetected.hpp"
#include "EventBus.hpp"
#include "CONFIG_FullConfigUpdate.hpp" // Include event with payload
#include "BaseEvent.hpp"
#include "EventTypes.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include <cmath>

#ifndef M_PI // Define if not used elsewhere that includes cmath indirectly
#define M_PI 3.14159265358979323846
#endif
static constexpr float DEG_TO_RAD = M_PI / 180.0f;


// Constructor takes initial config struct
FallDetector::FallDetector(EventBus& bus, const SystemBehaviorConfig& initialBehaviorConfig) :
    m_eventBus(bus),
    // m_configService removed
    // Initialize thresholds with defaults first
    m_pitch_threshold_rad(0.785f),
    m_threshold_duration_us(500000),
    m_potentially_fallen(false),
    m_fall_start_time_us(0),
    m_fall_event_published(false)
{
    applyConfig(initialBehaviorConfig); // Apply initial config
    reset(); // Initial reset
    ESP_LOGI(TAG, "FallDetector initialized."); // Log message simplified
}

void FallDetector::subscribeToEvents(EventBus& bus) {
    bus.subscribe(EventType::CONFIG_FULL_UPDATE, [this](const BaseEvent& ev) {
        this->handleConfigUpdate(ev);
    });
    ESP_LOGI(TAG, "Subscribed to CONFIG_UPDATED events.");
}


// Apply config values from struct
void FallDetector::applyConfig(const SystemBehaviorConfig& config) {
    m_pitch_threshold_rad = config.fall_pitch_threshold_deg * DEG_TO_RAD;
    m_threshold_duration_us = config.fall_threshold_duration_ms * 1000ULL;
    ESP_LOGI(TAG, "Applied FallDetector params: thresh=%.2f deg (%.3f rad), duration=%llu us",
             config.fall_pitch_threshold_deg, m_pitch_threshold_rad, m_threshold_duration_us);
}

// Handle config update event
void FallDetector::handleConfigUpdate(const BaseEvent& event) {
    if (event.type != EventType::CONFIG_FULL_UPDATE) return;
    ESP_LOGD(TAG, "Handling config update event.");
    const auto& configEvent = static_cast<const CONFIG_FullConfigUpdate&>(event);
    applyConfig(configEvent.configData.behavior);
}

// Call this externally if needed, e.g., when leaving FALLEN state in StateManager
void FallDetector::reset() {
    m_potentially_fallen = false;
    m_fall_start_time_us = 0;
    m_fall_event_published = false; // <<< Reset published flag
    ESP_LOGD(TAG, "Fall detector state reset.");
}

void FallDetector::check(float pitch_rad) {
    bool threshold_exceeded = (std::abs(pitch_rad) > m_pitch_threshold_rad);

    int64_t current_time_us = esp_timer_get_time();

    if (threshold_exceeded) {
        if (!m_potentially_fallen) {
            // Threshold exceeded for the first time, start timer
            m_potentially_fallen = true;
            m_fall_start_time_us = current_time_us;
            m_fall_event_published = false; // Ensure flag is reset when potentially falling again
            ESP_LOGD(TAG, "Potential fall detected, angle threshold (%.2f deg) exceeded (P:%.2f)",
                     m_pitch_threshold_rad * (180.0/M_PI), pitch_rad * (180.0/M_PI));
        } else {
            // Threshold still exceeded, check duration
            // Only publish ONCE per fall event
            if (!m_fall_event_published && (current_time_us - m_fall_start_time_us) >= m_threshold_duration_us) {
                ESP_LOGW(TAG, "Fall confirmed! Angle threshold exceeded for %llu us.", m_threshold_duration_us);
                MOTION_FallDetected event;
                m_eventBus.publish(event);
                m_fall_event_published = true; // <<< Set flag to prevent re-publishing
            }
        }
    } else {
        // Threshold not exceeded, reset the potential fall state and published flag
        if (m_potentially_fallen) {
             ESP_LOGD(TAG, "Potential fall condition cleared (angle %.2f within threshold %.2f deg).",
                     pitch_rad * (180.0/M_PI), m_pitch_threshold_rad * (180.0/M_PI));
             // Reset internal state now that condition is cleared
             m_potentially_fallen = false;
             m_fall_start_time_us = 0;
             m_fall_event_published = false;
        }
    }
}