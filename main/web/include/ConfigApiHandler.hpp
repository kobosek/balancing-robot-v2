#pragma once

#include "esp_http_server.h"
#include "esp_err.h"
#include "esp_log.h"
#include "ConfigData.hpp" // Need full definition

// Forward declare dependencies
class ConfigurationService;
class BaseEvent;
class CONFIG_FullConfigUpdate;
class EventBus; // Forward declare for subscribeToEvents

class ConfigApiHandler {
public: // <-- CHANGE: Move methods to public
    // Still needs direct access to ConfigService
    ConfigApiHandler(ConfigurationService& configService);
    esp_err_t handleGetRequest(httpd_req_t *req);
    esp_err_t handlePostRequest(httpd_req_t *req);

    // Subscribe to events to update its own config parameters if needed
    void subscribeToEvents(EventBus& bus);

    // Handler for config updates (now public)
    void handleConfigUpdate(const BaseEvent& event);

private:
    static constexpr const char* TAG = "ConfigApiHandler";

    ConfigurationService& m_configService;
    size_t m_max_post_data_size; // Loaded from config

    // Helper to apply config values relevant to this handler
    void applyConfig(const WebServerConfig& config);
    // void handleConfigUpdate(const BaseEvent& event); // <-- MOVED to public
};