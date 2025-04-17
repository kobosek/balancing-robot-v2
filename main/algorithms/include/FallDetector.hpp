// ================================================
// File: main/algorithms/include/FallDetector.hpp
// ================================================
#pragma once

#include "EventBus.hpp"
#include "FallDetectionEvent.hpp"
#include "ConfigData.hpp" // Include full definition
#include "esp_log.h"
#include "esp_timer.h"

// Forward declarations
// class ConfigurationService; // REMOVE
class BaseEvent;
class ConfigUpdatedEvent;

class FallDetector {
public:
    // Constructor now takes initial SystemBehaviorConfig
    FallDetector(EventBus& bus, const SystemBehaviorConfig& initialBehaviorConfig);
    ~FallDetector() = default;

    // Called by control loop
    void check(float pitch_rad);

    // Call externally (e.g. from StateManager) when leaving FALLEN state if needed
    void reset();

    // Subscribe to config updates
    void subscribeToEvents(EventBus& bus);

private:
    static constexpr const char* TAG = "FallDetector";
    EventBus& m_eventBus;

    float m_pitch_threshold_rad;
    uint64_t m_threshold_duration_us;

    bool m_potentially_fallen = false;
    int64_t m_fall_start_time_us = 0;
    bool m_fall_event_published = false;

    void applyConfig(const SystemBehaviorConfig& config);
    void handleConfigUpdate(const BaseEvent& event);
};