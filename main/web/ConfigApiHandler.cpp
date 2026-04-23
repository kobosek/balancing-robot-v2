#include "ConfigApiHandler.hpp"
#include "ConfigurationService.hpp"
#include "ConfigData.hpp"
#include "EventBus.hpp" // Include event bus
#include "CONFIG_FullConfigUpdate.hpp" // Include event definition
#include "BaseEvent.hpp"
#include <memory>
#include <string>
#include <new>
#include <algorithm>

ConfigApiHandler::ConfigApiHandler(ConfigurationService& configService) :
    m_configService(configService),
    m_max_post_data_size(4096) // Default before applying config
{
    applyConfig(m_configService.getWebServerConfig()); // Apply initial config
    ESP_LOGI(TAG, "ConfigApiHandler constructed.");
}

// Apply relevant config values
void ConfigApiHandler::applyConfig(const WebServerConfig& config) {
    m_max_post_data_size = std::max((size_t)512, (size_t)config.max_config_post_size); // Ensure reasonable min
    ESP_LOGI(TAG, "Applied ConfigApiHandler params: MaxPostSize=%zu", m_max_post_data_size);
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
    applyConfig(event.configData.web);
}

esp_err_t ConfigApiHandler::handleGetRequest(httpd_req_t *req) {
    ESP_LOGD(TAG, "Received request for /api/config (GET)");
    std::string json_output;
    esp_err_t serialize_ret = m_configService.getJsonString(json_output);

    if (serialize_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to serialize configuration for GET request");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Config generation failed");
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_sendstr(req, json_output.c_str());
    ESP_LOGD(TAG, "Sent current configuration data");
    return ESP_OK;
}

esp_err_t ConfigApiHandler::handlePostRequest(httpd_req_t *req) {
    ESP_LOGD(TAG, "Received request for /api/config (POST)");
    size_t content_len = req->content_len;

    // Use configured max size
    if (content_len == 0 || content_len > m_max_post_data_size) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                            content_len == 0 ? "Empty body" : "Payload too large");
        return ESP_FAIL;
    }

    std::unique_ptr<char[]> buf_ptr(new (std::nothrow) char[content_len + 1]);
    char* buf = buf_ptr.get();
    if (!buf) {
        ESP_LOGE(TAG, "Malloc failed for POST buf");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Mem alloc error");
        return ESP_FAIL;
    }

    int recv_len = httpd_req_recv(req, buf, content_len);
    if (recv_len <= 0) {
        if (recv_len == HTTPD_SOCK_ERR_TIMEOUT) {
            httpd_resp_send_408(req);
        } else {
            ESP_LOGE(TAG, "POST recv failed (%d)", recv_len);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Receive error");
        }
        return ESP_FAIL;
    }
    buf[recv_len] = '\0';
    ESP_LOGD(TAG, "Received JSON: %s", buf);

    esp_err_t ret = m_configService.updateConfigFromJson(std::string(buf));
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Config updated via POST.");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"status\":\"success\",\"message\":\"Configuration updated\"}");
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Config update failed (err: %s)", esp_err_to_name(ret));
        // Provide a slightly more specific error message if possible
        if (ret == ESP_FAIL) { // Assuming ESP_FAIL is used for validation errors by ConfigurationService
             httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid config data provided");
        } else {
             httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid config format or save failed");
        }
        return ESP_FAIL;
    }
}
