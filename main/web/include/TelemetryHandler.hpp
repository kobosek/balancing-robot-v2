#pragma once

#include "esp_http_server.h"
#include "esp_err.h"
#include "esp_log.h"
#include "TelemetryDataPoint.hpp" // Include the struct definition
#include <deque>
#include <mutex>
#include <vector>

// Forward declare dependencies if needed by header
class ConfigurationService;

class TelemetryHandler {
public:
    // Depends on ConfigurationService to get loop interval for JSON response
    TelemetryHandler(ConfigurationService& configService);

    // Method called by WebServer's static callback
    esp_err_t handleRequest(httpd_req_t *req);

    // Method for control task/RobotController to add data
    void addTelemetrySnapshot(const TelemetryDataPoint& data);

private:
    static constexpr const char* TAG = "TelemetryHandler";
    static constexpr size_t TELEMETRY_BUFFER_MAX_SIZE = 100;

    ConfigurationService& m_configService; // To get loop interval
    std::deque<TelemetryDataPoint> m_telemetryBuffer;
    std::mutex m_telemetryMutex;
};