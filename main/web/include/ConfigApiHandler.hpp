#pragma once

#include "esp_http_server.h"
#include "esp_err.h"
#include "esp_log.h"
#include "EventHandler.hpp"
#include "config/WebServerConfig.hpp"
#include <atomic>

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
    esp_err_t handleOperationRequest(httpd_req_t *req);

    // EventHandler interface implementation
    void handleEvent(const BaseEvent& event) override;
    std::string getHandlerName() const override { return TAG; }

    // Handler for config updates (now public)
    void handleConfigUpdate(const CONFIG_FullConfigUpdate& event);

private:
    static constexpr const char* TAG = "ConfigApiHandler";

    ConfigurationService& m_configService;
    std::atomic<size_t> m_max_post_data_size; // Loaded from config
    std::atomic<uint32_t> m_config_revision{0};
    std::atomic<bool> m_has_config_revision{false};

    // Helper to apply config values relevant to this handler
    void applyConfig(const WebServerConfig& config);
    esp_err_t sendOperationStatus(httpd_req_t* req) const;
    // void handleConfigUpdate(const BaseEvent& event); // <-- MOVED to public
};
