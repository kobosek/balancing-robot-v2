#include "ConfigApiHandler.hpp"
#include "ConfigurationService.hpp"
#include "ConfigData.hpp"
#include "EventBus.hpp" // Include event bus
#include "CONFIG_FullConfigUpdate.hpp" // Include event definition
#include "BaseEvent.hpp"
#include "HttpResponseUtils.hpp"
#include <memory>
#include <string>
#include <new>
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cerrno>
#include <cmath>
#include <limits>
#include "cJSON.h"

namespace {
const char* operationPhaseToString(ControlOperationPhase phase)
{
    switch (phase) {
        case ControlOperationPhase::RESERVED:  return "reserved";
        case ControlOperationPhase::PREPARING: return "preparing";
        case ControlOperationPhase::RUNNING:   return "running";
        case ControlOperationPhase::WRITING:   return "writing";
        case ControlOperationPhase::APPLYING:  return "applying";
        case ControlOperationPhase::SUCCEEDED: return "succeeded";
        case ControlOperationPhase::FAILED:    return "failed";
        case ControlOperationPhase::ABORTED:   return "aborted";
        case ControlOperationPhase::IDLE:
        default:                                return "idle";
    }
}

const char* operationKindToString(ControlOperationKind kind)
{
    switch (kind) {
        case ControlOperationKind::MOTION:        return "motion";
        case ControlOperationKind::CONFIGURATION: return "configuration";
        case ControlOperationKind::OTA:           return "ota";
        case ControlOperationKind::CALIBRATION:   return "calibration";
        case ControlOperationKind::NONE:
        default:                                  return "none";
    }
}

bool parseOperationId(cJSON* item, uint64_t& operationId)
{
    if (!item) return false;
    if (cJSON_IsString(item) && item->valuestring) {
        const char* value = item->valuestring;
        if (*value == '\0' || *value == '-') return false;
        for (const char* cursor = value; *cursor != '\0'; ++cursor) {
            if (*cursor < '0' || *cursor > '9') return false;
        }
        errno = 0;
        char* end = nullptr;
        const unsigned long long parsed = std::strtoull(value, &end, 10);
        if (errno != 0 || end == value || *end != '\0' || parsed == 0) {
            return false;
        }
        operationId = static_cast<uint64_t>(parsed);
        return operationId != 0;
    }
    if (cJSON_IsNumber(item) && std::isfinite(item->valuedouble) &&
        item->valuedouble >= 1.0 &&
        item->valuedouble <= 9007199254740991.0 &&
        std::floor(item->valuedouble) == item->valuedouble) {
        operationId = static_cast<uint64_t>(item->valuedouble);
        return operationId != 0;
    }
    return false;
}
}

ConfigApiHandler::ConfigApiHandler(ConfigurationService& configService) :
    m_configService(configService),
    m_max_post_data_size(8192) // Default before applying config
{
    applyConfig(m_configService.getWebServerConfig()); // Apply initial config
    ESP_LOGI(TAG, "ConfigApiHandler constructed.");
}

// Apply relevant config values
void ConfigApiHandler::applyConfig(const WebServerConfig& config) {
    m_max_post_data_size = std::max((size_t)512, (size_t)config.max_config_post_size); // Ensure reasonable min
    ESP_LOGI(TAG, "Applied ConfigApiHandler params: MaxPostSize=%zu", m_max_post_data_size.load(std::memory_order_relaxed));
}

// EventHandler implementation
void ConfigApiHandler::handleEvent(const BaseEvent& event) {
    if (event.is<CONFIG_FullConfigUpdate>()) {
        handleConfigUpdate(event.as<CONFIG_FullConfigUpdate>());
    } else {
        ESP_LOGV(TAG, "%s: Received unhandled event '%s'",
                 getHandlerName().c_str(), event.eventName());
    }
}

// Handle config update event
void ConfigApiHandler::handleConfigUpdate(const CONFIG_FullConfigUpdate& event) {
    ESP_LOGD(TAG, "Handling config update event.");
    if (m_has_config_revision.load(std::memory_order_acquire) &&
        event.configData.config_revision < m_config_revision.load(std::memory_order_relaxed)) {
        ESP_LOGW(TAG, "Ignoring stale config API revision %lu (current %lu)",
                 static_cast<unsigned long>(event.configData.config_revision),
                 static_cast<unsigned long>(m_config_revision.load(std::memory_order_relaxed)));
        return;
    }
    m_config_revision.store(event.configData.config_revision, std::memory_order_release);
    m_has_config_revision.store(true, std::memory_order_release);
    applyConfig(event.configData.web);
}

esp_err_t ConfigApiHandler::handleGetRequest(httpd_req_t *req) {
    ESP_LOGD(TAG, "Received request for /api/config (GET)");
    std::string json_output;
    esp_err_t serialize_ret = m_configService.getJsonString(json_output);

    if (serialize_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to serialize configuration for GET request");
        return sendHttpError(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Config generation failed");
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_sendstr(req, json_output.c_str());
    ESP_LOGD(TAG, "Sent current configuration data");
    return ESP_OK;
}

esp_err_t ConfigApiHandler::sendOperationStatus(httpd_req_t* req) const
{
    const ControlOperationStatus status = m_configService.getOperationStatus();
    const ConfigData config = m_configService.getConfigData();
    cJSON* root = cJSON_CreateObject();
    if (!root) return sendHttp500(req);
    cJSON_AddBoolToObject(root, "operation_known", status.known);
    cJSON_AddBoolToObject(root, "operation_active", status.active);
    cJSON_AddStringToObject(root, "operation_kind",
                            operationKindToString(status.kind));
    char operationId[24] = {};
    std::snprintf(operationId, sizeof(operationId), "%llu",
                  static_cast<unsigned long long>(status.operationId));
    cJSON_AddStringToObject(root, "operation_id", operationId);
    cJSON_AddStringToObject(root, "operation_phase",
                            operationPhaseToString(status.phase));
    cJSON_AddNumberToObject(root, "operation_result_code", status.resultCode);
    cJSON_AddNumberToObject(root, "config_revision", config.config_revision);
    cJSON_AddNumberToObject(root, "operation_base_revision", status.baseRevision);
    cJSON_AddNumberToObject(root, "operation_target_revision", status.targetRevision);
    cJSON_AddBoolToObject(root, "operation_recovery_pending",
                          m_configService.isOperationRecoveryPending());
    char* json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!json) return sendHttp500(req);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    const esp_err_t result = httpd_resp_sendstr(req, json);
    cJSON_free(json);
    return httpResponseResult(result);
}

esp_err_t ConfigApiHandler::handleOperationRequest(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        return sendOperationStatus(req);
    }
    if (req->method != HTTP_POST || req->content_len <= 0 || req->content_len > 256) {
        return sendHttpError(req, HTTPD_400_BAD_REQUEST,
                             "Operation reconciliation requires a JSON operation_id");
    }

    std::unique_ptr<char[]> buffer(new (std::nothrow)
                                   char[static_cast<size_t>(req->content_len) + 1]);
    if (!buffer) return sendHttp500(req);
    const int received = receiveHttpRequestBody(
        req, buffer.get(), static_cast<size_t>(req->content_len));
    if (received <= 0) {
        return received == HTTPD_SOCK_ERR_TIMEOUT
            ? sendHttpTimeout(req)
            : sendHttpError(req, HTTPD_400_BAD_REQUEST, "Operation body could not be read");
    }
    buffer[received] = '\0';
    cJSON* root = cJSON_Parse(buffer.get());
    if (!root || !cJSON_IsObject(root)) {
        if (root) cJSON_Delete(root);
        return sendHttpError(req, HTTPD_400_BAD_REQUEST, "Invalid operation JSON");
    }
    cJSON* idItem = cJSON_GetObjectItem(root, "operation_id");
    if (!idItem) idItem = cJSON_GetObjectItem(root, "operationId");
    uint64_t operationId = 0;
    const bool valid = parseOperationId(idItem, operationId);
    cJSON_Delete(root);
    if (!valid) {
        return sendHttpError(req, HTTPD_400_BAD_REQUEST,
                             "operation_id must be a positive decimal integer or string");
    }

    const esp_err_t ret = m_configService.acknowledgeOperation(operationId);
    if (ret != ESP_OK) {
        return sendHttpError(req,
                             ret == ESP_ERR_NOT_FOUND ? HTTPD_404_NOT_FOUND
                                                      : HTTPD_400_BAD_REQUEST,
                             ret == ESP_ERR_NOT_FOUND
                                 ? "Operation is unknown"
                                 : "Operation cannot be reconciled yet");
    }
    return sendOperationStatus(req);
}

esp_err_t ConfigApiHandler::handlePostRequest(httpd_req_t *req) {
    ESP_LOGD(TAG, "Received request for /api/config (POST)");
    size_t content_len = req->content_len;

    // Use configured max size
    if (content_len == 0 || content_len > m_max_post_data_size.load(std::memory_order_relaxed)) {
        return sendHttpError(req,
                             HTTPD_400_BAD_REQUEST,
                             content_len == 0 ? "Empty body" : "Payload too large");
    }

    std::unique_ptr<char[]> buf_ptr(new (std::nothrow) char[content_len + 1]);
    char* buf = buf_ptr.get();
    if (!buf) {
        ESP_LOGE(TAG, "Malloc failed for POST buf");
        return sendHttpError(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Mem alloc error");
    }

    int recv_len = receiveHttpRequestBody(req, buf, content_len);
    if (recv_len <= 0) {
        if (recv_len == HTTPD_SOCK_ERR_TIMEOUT) {
            return sendHttpTimeout(req);
        } else {
            ESP_LOGE(TAG, "POST recv failed (%d)", recv_len);
            return sendHttpError(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Receive error");
        }
    }
    buf[recv_len] = '\0';


    std::string validationError;
    esp_err_t ret = m_configService.updateConfigFromJson(std::string(buf), &validationError);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Config updated via POST.");
        const ConfigData savedConfig = m_configService.getConfigData();
        const ControlOperationStatus operation = m_configService.getOperationStatus();
        char operationId[24] = {};
        std::snprintf(operationId, sizeof(operationId), "%llu",
                      static_cast<unsigned long long>(operation.operationId));
        char response[768];
        std::snprintf(response, sizeof(response),
                      "{\"status\":\"success\",\"message\":\"Configuration updated\",\"config_revision\":%lu,\"strategies_revision\":%lu,\"nested_pid_revision\":%lu,\"longitudinal_cascade_revision\":%lu,\"operation_id\":\"%s\",\"operation_phase\":\"%s\",\"operation_result_code\":%ld,\"operation_base_revision\":%lu,\"operation_target_revision\":%lu}",
                      static_cast<unsigned long>(savedConfig.config_revision),
                      static_cast<unsigned long>(savedConfig.control.strategies.revision),
                      static_cast<unsigned long>(savedConfig.control.strategies.nested_pid.revision),
                      static_cast<unsigned long>(savedConfig.control.strategies.longitudinal_cascade.revision),
                      operationId,
                      operationPhaseToString(operation.phase),
                      static_cast<long>(operation.resultCode),
                      static_cast<unsigned long>(operation.baseRevision),
                      static_cast<unsigned long>(operation.targetRevision));
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, response);
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Config update failed (err: %s)", esp_err_to_name(ret));
        if (!validationError.empty()) return sendHttpError(req, HTTPD_400_BAD_REQUEST, validationError.c_str());
        // Provide a slightly more specific error message if possible
        if (ret == ESP_FAIL) { // Assuming ESP_FAIL is used for validation errors by ConfigurationService
             return sendHttpError(req, HTTPD_400_BAD_REQUEST, "Invalid config data provided");
        } else {
             return sendHttpError(req, HTTPD_400_BAD_REQUEST, "Invalid config format or save failed");
        }
    }
}
