#include "OTAApiHandler.hpp"

#include "OTAService.hpp"
#include "cJSON.h"
#include "esp_log.h"
#include <memory>
#include <new>
#include <string>

OTAApiHandler::OTAApiHandler(OTAService& otaService)
    : m_otaService(otaService) {}

esp_err_t OTAApiHandler::handleStatusRequest(httpd_req_t* req) {
    return sendStatusJson(req);
}

esp_err_t OTAApiHandler::sendStatusJson(httpd_req_t* req) {
    const OTAStatus status = m_otaService.getStatus();

    auto cjson_deleter = [](cJSON* ptr){ if (ptr) cJSON_Delete(ptr); };
    auto char_deleter = [](char* ptr){ if (ptr) free(ptr); };
    std::unique_ptr<cJSON, decltype(cjson_deleter)> root(cJSON_CreateObject(), cjson_deleter);
    if (!root) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    cJSON_AddBoolToObject(root.get(), "available", status.available);
    cJSON_AddBoolToObject(root.get(), "spiffs_available", status.spiffsAvailable);
    cJSON_AddBoolToObject(root.get(), "update_in_progress", status.updateInProgress);
    cJSON_AddBoolToObject(root.get(), "reboot_required", status.rebootRequired);
    cJSON_AddNumberToObject(root.get(), "bytes_written", static_cast<double>(status.bytesWritten));
    cJSON_AddNumberToObject(root.get(), "expected_size", static_cast<double>(status.expectedSize));
    cJSON_AddNumberToObject(root.get(), "spiffs_partition_size", static_cast<double>(status.spiffsPartitionSize));
    cJSON_AddStringToObject(root.get(), "running_partition", status.runningPartition.c_str());
    cJSON_AddStringToObject(root.get(), "update_partition", status.updatePartition.c_str());
    cJSON_AddStringToObject(root.get(), "spiffs_partition", status.spiffsPartition.c_str());
    cJSON_AddStringToObject(root.get(), "app_version", status.appVersion.c_str());
    cJSON_AddStringToObject(root.get(), "active_target", status.activeTarget.c_str());
    cJSON_AddStringToObject(root.get(), "message", status.message.c_str());

    std::unique_ptr<char, decltype(char_deleter)> json(cJSON_PrintUnformatted(root.get()), char_deleter);
    if (!json) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_sendstr(req, json.get());
}

OTAUpdateTarget OTAApiHandler::parseUploadTarget(httpd_req_t* req) const {
    char query[64] = {};
    char target[16] = {};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK &&
        httpd_query_key_value(query, "target", target, sizeof(target)) == ESP_OK) {
        if (std::string(target) == "spiffs") {
            return OTAUpdateTarget::SPIFFS;
        }
    }
    return OTAUpdateTarget::APP;
}

esp_err_t OTAApiHandler::sendErrorJson(httpd_req_t* req, httpd_err_code_t status, const char* message) {
    char body[160];
    snprintf(body, sizeof(body), "{\"status\":\"error\",\"message\":\"%s\"}", message);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_status(req, status == HTTPD_400_BAD_REQUEST ? "400 Bad Request" : "500 Internal Server Error");
    return httpd_resp_sendstr(req, body);
}

esp_err_t OTAApiHandler::handleUploadRequest(httpd_req_t* req) {
    if (req->content_len <= 0) {
        sendErrorJson(req, HTTPD_400_BAD_REQUEST, "Empty OTA image");
        return ESP_FAIL;
    }

    const OTAUpdateTarget target = parseUploadTarget(req);
    esp_err_t ret = target == OTAUpdateTarget::SPIFFS
        ? m_otaService.beginSpiffsUpdate(req->content_len)
        : m_otaService.beginAppUpdate(req->content_len);
    if (ret != ESP_OK) {
        const OTAStatus status = m_otaService.getStatus();
        ESP_LOGE(TAG, "OTA begin failed: %s", esp_err_to_name(ret));
        sendErrorJson(req, HTTPD_500_INTERNAL_SERVER_ERROR, status.message.c_str());
        return ESP_FAIL;
    }

    constexpr size_t BUFFER_SIZE = 4096;
    std::unique_ptr<uint8_t[]> buffer(new (std::nothrow) uint8_t[BUFFER_SIZE]);
    if (!buffer) {
        m_otaService.abort();
        sendErrorJson(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA buffer allocation failed");
        return ESP_FAIL;
    }

    int remaining = req->content_len;
    while (remaining > 0) {
        const int toRead = remaining > static_cast<int>(BUFFER_SIZE) ? static_cast<int>(BUFFER_SIZE) : remaining;
        const int received = httpd_req_recv(req, reinterpret_cast<char*>(buffer.get()), toRead);
        if (received <= 0) {
            m_otaService.abort();
            if (received == HTTPD_SOCK_ERR_TIMEOUT) {
                httpd_resp_send_408(req);
            } else {
                sendErrorJson(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA upload receive failed");
            }
            return ESP_FAIL;
        }

        ret = m_otaService.write(buffer.get(), static_cast<size_t>(received));
        if (ret != ESP_OK) {
            m_otaService.abort();
            const OTAStatus status = m_otaService.getStatus();
            sendErrorJson(req, HTTPD_500_INTERNAL_SERVER_ERROR, status.message.c_str());
            return ESP_FAIL;
        }
        remaining -= received;
    }

    ret = m_otaService.finish();
    if (ret != ESP_OK) {
        const OTAStatus status = m_otaService.getStatus();
        sendErrorJson(req, HTTPD_500_INTERNAL_SERVER_ERROR, status.message.c_str());
        return ESP_FAIL;
    }

    return sendStatusJson(req);
}
