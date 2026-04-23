#pragma once

#include "esp_http_server.h"
#include "esp_err.h"
#include "esp_log.h"
#include "EventHandler.hpp"
#include "config/WebServerConfig.hpp"

// Forward declare dependencies
class ConfigurationService;
class BaseEvent;
class CONFIG_FullConfigUpdate;

class ConfigApiHandler : public EventHandler {
public: 
    // Still needs direct access to ConfigService
    ConfigApiHandler(ConfigurationService& configService);
    esp_err_t handleGetRequest(httpd_req_t *req);
    esp_err_t handlePostRequest(httpd_req_t *req);

    // EventHandler interface implementation
    void handleEvent(const BaseEvent& event) override;
    std::string getHandlerName() const override { return TAG; }

    // Handler for config updates (now public)
    void handleConfigUpdate(const CONFIG_FullConfigUpdate& event);

private:
    static constexpr const char* TAG = "ConfigApiHandler";

    ConfigurationService& m_configService;
    size_t m_max_post_data_size; // Loaded from config

    // Helper to apply config values relevant to this handler
    void applyConfig(const WebServerConfig& config);
    // void handleConfigUpdate(const BaseEvent& event); // <-- MOVED to public
};
