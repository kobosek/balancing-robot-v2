// main/algorithms/include/FallDetector.hpp
#pragma once

#include "EventBus.hpp"
#include "FallDetectionEvent.hpp"
#include "esp_log.h"
#include "esp_timer.h"

class FallDetector {
public:
    FallDetector(EventBus& bus, float pitch_threshold_rad = 0.785f, // 45 degrees
                 float roll_threshold_rad = 0.785f, // 45 degrees
                 uint64_t threshold_duration_ms = 500);

    // Called by control loop
    void check(float pitch_rad, float roll_rad);

    // Call externally (e.g. from StateManager) when leaving FALLEN state if needed
    void reset();

private:
    static constexpr const char* TAG = "FallDetector";
    EventBus& m_eventBus;
    const float m_pitch_threshold_rad;
    const float m_roll_threshold_rad;
    const uint64_t m_threshold_duration_us;

    // Internal state
    bool m_potentially_fallen = false;
    int64_t m_fall_start_time_us = 0;
    bool m_fall_event_published = false; // <<< ADDED flag
};