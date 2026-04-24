#pragma once

#include "esp_err.h"
#include "esp_http_server.h"

class LogBufferService;

class LogsApiHandler {
public:
    explicit LogsApiHandler(LogBufferService& logBuffer);

    esp_err_t handleGetRequest(httpd_req_t* req);
    esp_err_t handleDeleteRequest(httpd_req_t* req);

private:
    static constexpr const char* TAG = "LogsApiHandler";

    LogBufferService& m_logBuffer;
};
