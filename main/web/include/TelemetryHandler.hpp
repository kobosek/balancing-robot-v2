#pragma once

#include "esp_http_server.h"
#include "esp_err.h"
#include "esp_log.h"
#include "TelemetryDataPoint.hpp" // Include the struct definition
#include "EventHandler.hpp"
#include "config/WebServerConfig.hpp"
#include <deque>
#include <mutex>
#include <cstdint>

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
    static constexpr size_t MAX_BUFFER_SIZE = 500;
    // Keep HTTP work bounded even when the ring has accumulated a full
    // history.  Remaining samples stay queued for the next request.
    static constexpr size_t MAX_RESPONSE_BATCH_SIZE = 32;

    // ConfigurationService& m_configService; // REMOVE
    std::deque<TelemetryDataPoint> m_telemetryBuffer;
    std::mutex m_telemetryMutex;
    // The HTTP response is assembled from this bounded snapshot.  A separate
    // lock keeps concurrent /data requests from sharing the snapshot storage.
    TelemetryDataPoint m_responseBuffer[MAX_RESPONSE_BATCH_SIZE]{};
    std::mutex m_responseMutex;
    size_t m_telemetry_buffer_max_size; // Loaded from config
    uint32_t m_config_revision = 0;
    bool m_has_config_revision = false;
    uint32_t m_dropped_samples = 0;

    // Helper to apply config values
    void applyConfig(const WebServerConfig& config);
    // void handleConfigUpdate(const BaseEvent& event); // <-- MOVED to public
};
