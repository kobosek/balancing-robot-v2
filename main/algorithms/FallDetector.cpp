// main/algorithms/FallDetector.cpp
#include "FallDetector.hpp"
#include "FallDetectionEvent.hpp"
#include "EventBus.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include <cmath>

#ifndef M_PI // Define if not used elsewhere that includes cmath indirectly
#define M_PI 3.14159265358979323846
#endif


FallDetector::FallDetector(EventBus& bus, float pitch_threshold_rad,
                           uint64_t threshold_duration_ms) :
    m_eventBus(bus),
    m_pitch_threshold_rad(pitch_threshold_rad),
    m_threshold_duration_us(threshold_duration_ms * 1000),
    m_potentially_fallen(false), // Ensure members are initialized
    m_fall_start_time_us(0),
    m_fall_event_published(false) // <<< ADDED flag
{
    reset(); // Initial reset
}

// Call this externally if needed, e.g., when leaving FALLEN state in StateManager
void FallDetector::reset() {
    m_potentially_fallen = false;
    m_fall_start_time_us = 0;
    m_fall_event_published = false; // <<< Reset published flag
    ESP_LOGI(TAG, "Fall detector state reset.");
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
            ESP_LOGD(TAG, "Potential fall detected, angle threshold exceeded (P:%.2f)",
                     pitch_rad * (180.0/M_PI));
        } else {
            // Threshold still exceeded, check duration
            // Only publish ONCE per fall event
            if (!m_fall_event_published && (current_time_us - m_fall_start_time_us) >= m_threshold_duration_us) {
                ESP_LOGW(TAG, "Fall confirmed! Angle threshold exceeded for duration.");
                FallDetectionEvent event;
                m_eventBus.publish(event);
                m_fall_event_published = true; // <<< Set flag to prevent re-publishing
                // --- DO NOT RESET HERE ---
                // reset(); // <<< REMOVED reset() call
                // --- End Removal ---
            }
        }
    } else {
        // Threshold not exceeded, reset the potential fall state and published flag
        if (m_potentially_fallen) {
             ESP_LOGD(TAG, "Potential fall condition cleared (angle within threshold).");
             // Reset internal state now that condition is cleared
             m_potentially_fallen = false;
             m_fall_start_time_us = 0;
             m_fall_event_published = false;
             // Optionally call the public reset() method if it does more complex cleanup
             // reset();
        }
    }
}