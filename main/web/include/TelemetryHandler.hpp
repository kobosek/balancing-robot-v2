#pragma once

#include "esp_http_server.h"
#include "esp_err.h"
#include "esp_log.h"
#include "TelemetryDataPoint.hpp" // Include the struct definition
#include "ConfigData.hpp" // Include full definition
#include <deque>
#include <mutex>
#include <vector>

// Forward declare dependencies
class BaseEvent;
class ConfigUpdatedEvent;
class EventBus; // Forward declare for subscribeToEvents

class TelemetryHandler {
public: // <-- CHANGE: Move methods to public
    // Constructor takes initial WebServerConfig
    TelemetryHandler(const WebServerConfig& initialWebConfig);

    esp_err_t handleRequest(httpd_req_t *req);
    void addTelemetrySnapshot(const TelemetryDataPoint& data);

    // Method to subscribe to events
    void subscribeToEvents(EventBus& bus);

    // Handler for config updates (now public)
    void handleConfigUpdate(const BaseEvent& event);

private:
    static constexpr const char* TAG = "TelemetryHandler";

    // ConfigurationService& m_configService; // REMOVE
    std::deque<TelemetryDataPoint> m_telemetryBuffer;
    std::mutex m_telemetryMutex;
    size_t m_telemetry_buffer_max_size; // Loaded from config

    // Helper to apply config values
    void applyConfig(const WebServerConfig& config);
    // void handleConfigUpdate(const BaseEvent& event); // <-- MOVED to public
};