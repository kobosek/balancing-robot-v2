#pragma once

#include "esp_http_server.h"
#include "esp_err.h"
#include "esp_log.h"

class StaticFileHandler {
public:
    // Constructor might take base path if needed, or assume "/spiffs"
    StaticFileHandler(const char* base_path = "/spiffs");
    esp_err_t handleRequest(httpd_req_t *req);

private:
    static constexpr const char* TAG = "StaticFileHandler";
    const char* m_base_path;
    void set_content_type_from_file(httpd_req_t *req, const char *filename);
};