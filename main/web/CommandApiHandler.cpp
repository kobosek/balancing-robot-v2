// main/web/CommandApiHandler.cpp
#include "CommandApiHandler.hpp"
#include "EventBus.hpp"             // Need full definition
#include "EventTypes.hpp"           // Need full definition
#include "StartBalancingCommand.hpp"// Need full definition
#include "StopCommand.hpp"          // Need full definition
#include "CalibrateCommandEvent.hpp"        // <<< ADDED
#include "EnableRecoveryCommandEvent.hpp"   // <<< ADDED
#include "DisableRecoveryCommandEvent.hpp"  // <<< ADDED
#include "EnableFallDetectCommandEvent.hpp"  // <<< ADDED
#include "DisableFallDetectCommandEvent.hpp" // <<< ADDED
#include "cJSON.h"
#include <memory>                   // For unique_ptr
#include <string>
#include "esp_http_server.h" // <<< ADDED for httpd_resp_* functions
#include "esp_log.h"         // <<< ADDED for logging

CommandApiHandler::CommandApiHandler(EventBus& eventBus) : m_eventBus(eventBus) {}

esp_err_t CommandApiHandler::handleRequest(httpd_req_t *req) {
    ESP_LOGD(TAG, "Received request for /api/command (POST)");
    char buf[128]; // Slightly larger buffer just in case
    esp_err_t ret = ESP_FAIL;
    size_t content_len = req->content_len;

    if (content_len == 0 || content_len >= sizeof(buf)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, content_len == 0 ? "Empty body" : "Payload too large or invalid");
        return ESP_FAIL;
    }

    int recv_ret = httpd_req_recv(req, buf, content_len);
    if (recv_ret <= 0) {
        if (recv_ret == HTTPD_SOCK_ERR_TIMEOUT) httpd_resp_send_408(req);
        else httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Receive error");
        return ESP_FAIL;
    }
    buf[recv_ret] = '\0';

    auto cjson_deleter = [](cJSON* ptr){ if(ptr) cJSON_Delete(ptr); };
    std::unique_ptr<cJSON, decltype(cjson_deleter)> root_ptr(cJSON_Parse(buf));
    cJSON* root = root_ptr.get();
    if (!root) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    cJSON *cmd_item = cJSON_GetObjectItemCaseSensitive(root, "command");
    if (!cmd_item || !cJSON_IsString(cmd_item) || !cmd_item->valuestring) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing/invalid 'command' field");
        return ESP_FAIL;
    }
    std::string command_str = cmd_item->valuestring;
    std::string message = "Unknown command";

    if (command_str == "start") {
        ESP_LOGI(TAG, "Publishing START_COMMAND_RECEIVED");
        StartBalancingCommand cmd_event;
        m_eventBus.publish(cmd_event);
        message = "Start command sent";
        ret = ESP_OK;
    } else if (command_str == "stop") {
        ESP_LOGI(TAG, "Publishing STOP_COMMAND_RECEIVED");
        StopCommand cmd_event;
        m_eventBus.publish(cmd_event);
        message = "Stop command sent";
        ret = ESP_OK;
    } else if (command_str == "calibrate") { // <<< ADDED
        ESP_LOGI(TAG, "Publishing CALIBRATE_COMMAND_RECEIVED");
        CalibrateCommandEvent cmd_event;
        m_eventBus.publish(cmd_event);
        message = "Calibrate command sent";
        ret = ESP_OK;
    } else if (command_str == "enable_recovery") { // <<< ADDED
        ESP_LOGI(TAG, "Publishing ENABLE_RECOVERY_COMMAND_RECEIVED");
        EnableRecoveryCommandEvent cmd_event;
        m_eventBus.publish(cmd_event);
        message = "Enable recovery command sent";
        ret = ESP_OK;
     } else if (command_str == "disable_recovery") { // <<< ADDED
        ESP_LOGI(TAG, "Publishing DISABLE_RECOVERY_COMMAND_RECEIVED");
        DisableRecoveryCommandEvent cmd_event;
        m_eventBus.publish(cmd_event);
        message = "Disable recovery command sent";
        ret = ESP_OK;
    } else if (command_str == "enable_fall_detect") {
        ESP_LOGI(TAG, "Publishing ENABLE_FALL_DETECT_COMMAND_RECEIVED");
        EnableFallDetectCommandEvent cmd_event;
        m_eventBus.publish(cmd_event);
        message = "Enable fall detect command sent";
        ret = ESP_OK;
     } else if (command_str == "disable_fall_detect") {
        ESP_LOGI(TAG, "Publishing DISABLE_FALL_DETECT_COMMAND_RECEIVED");
        DisableFallDetectCommandEvent cmd_event;
        m_eventBus.publish(cmd_event);
        message = "Disable fall detect command sent";
        ret = ESP_OK;
    // <<< END Added >>>
    } else {
        ESP_LOGW(TAG, "Unknown command received: %s", command_str.c_str());
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Unknown command");
        return ESP_FAIL; // Error already sent
    }

    // Send success response if command was known
    if(ret == ESP_OK) {
        // Simple JSON success response
        char resp_buf[128];
        snprintf(resp_buf, sizeof(resp_buf), "{\"status\":\"success\", \"message\":\"%s\"}", message.c_str());
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, resp_buf);
    }

    return ret;
}