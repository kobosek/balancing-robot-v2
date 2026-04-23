#pragma once

#include "esp_http_server.h"
#include "esp_err.h"
#include "esp_log.h"
#include "TelemetryDataPoint.hpp" // Include the struct definition
#include "EventHandler.hpp"
#include "config/WebServerConfig.hpp"
#include <deque>
#include <mutex>
#include <vector>

// Forward declare dependencies
class BaseEvent;
class CONFIG_FullConfigUpdate;
class EventBus; // Forward declare for subscribeToEvents

class TelemetryHandler : public EventHandler {
public: 
    // Constructor takes initial WebServerConfig
    TelemetryHandler(const WebServerConfig& initialWebConfig);

    esp_err_t handleRequest(httpd_req_t *req);
    void addTelemetrySnapshot(const TelemetryDataPoint& data);

    // EventHandler interface implementation
    void handleEvent(const BaseEvent& event) override;
    std::string getHandlerName() const override { return TAG; }

    // Handler for config updates (now public)
    void handleConfigUpdate(const CONFIG_FullConfigUpdate& event);

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
