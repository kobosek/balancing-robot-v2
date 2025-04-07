#pragma once

#include "esp_http_server.h"
#include "esp_err.h"
#include "esp_log.h"

// Forward declare dependencies
class ConfigurationService;

class ConfigApiHandler {
public:
    ConfigApiHandler(ConfigurationService& configService);
    esp_err_t handleGetRequest(httpd_req_t *req);
    esp_err_t handlePostRequest(httpd_req_t *req);

private:
    static constexpr const char* TAG = "ConfigApiHandler";
    static constexpr size_t MAX_POST_DATA_SIZE = 4096;
    ConfigurationService& m_configService;
};