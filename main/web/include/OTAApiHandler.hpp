#pragma once

#include "esp_err.h"
#include "esp_http_server.h"

class OTAService;
enum class OTAUpdateTarget;

class OTAApiHandler {
public:
    explicit OTAApiHandler(OTAService& otaService);

    esp_err_t handleStatusRequest(httpd_req_t* req);
    esp_err_t handleUploadRequest(httpd_req_t* req);

private:
    static constexpr const char* TAG = "OTAApiHandler";
    OTAService& m_otaService;

    esp_err_t sendStatusJson(httpd_req_t* req);
    OTAUpdateTarget parseUploadTarget(httpd_req_t* req) const;
    esp_err_t sendErrorJson(httpd_req_t* req, httpd_err_code_t status, const char* message);
};
