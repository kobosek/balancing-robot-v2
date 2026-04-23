// main/web/CommandApiHandler.cpp
#include "CommandApiHandler.hpp"
#include "EventBus.hpp"             // Need full definition
#include "EventTypes.hpp"           // Need full definition
#include "UI_StartBalancing.hpp"// Need full definition
#include "UI_Stop.hpp"          // Need full definition
#include "UI_CalibrateImu.hpp"        // <<< ADDED
#include "UI_EnableAutoBalancing.hpp"
#include "UI_DisableAutoBalancing.hpp"
#include "UI_EnableFallDetection.hpp"  // <<< ADDED
#include "UI_DisableFallDetection.hpp" // <<< ADDED
#include "UI_StartPidTuning.hpp"
#include "UI_CancelPidTuning.hpp"
#include "UI_SavePidTuning.hpp"
#include "UI_DiscardPidTuning.hpp"
#include "UI_StartGuidedCalibration.hpp"
#include "UI_CancelGuidedCalibration.hpp"
#include "PidTuningTypes.hpp"
#include "BaseEvent.hpp"           // Need for BaseEvent
#include "cJSON.h"
#include <memory>                   // For unique_ptr
#include <string>
#include "esp_http_server.h" // <<< ADDED for httpd_resp_* functions
#include "esp_log.h"         // <<< ADDED for logging

CommandApiHandler::CommandApiHandler(EventBus& eventBus) : m_eventBus(eventBus) {
    ESP_LOGI(TAG, "CommandApiHandler constructed.");
}

// EventHandler implementation
void CommandApiHandler::handleEvent(const BaseEvent& event) {
    // Currently this handler doesn't need to process any events
    // Just log unhandled events at verbose level
    ESP_LOGV(TAG, "%s: Received unhandled event type %d", 
             getHandlerName().c_str(), static_cast<int>(event.type));
}

esp_err_t CommandApiHandler::handleRequest(httpd_req_t *req) {
    ESP_LOGD(TAG, "Received request for /api/command (POST)");
    char buf[256]; // Allows command plus optional tuning target
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
        UI_StartBalancing cmd_event;
        m_eventBus.publish(cmd_event);
        message = "Start command sent";
        ret = ESP_OK;
    } else if (command_str == "stop") {
        ESP_LOGI(TAG, "Publishing STOP_COMMAND_RECEIVED");
        UI_Stop cmd_event;
        m_eventBus.publish(cmd_event);
        message = "Stop command sent";
        ret = ESP_OK;
    } else if (command_str == "calibrate") { // <<< ADDED
        ESP_LOGI(TAG, "Publishing CALIBRATE_COMMAND_RECEIVED");
        UI_CalibrateImu cmd_event;
        m_eventBus.publish(cmd_event);
        message = "Calibrate command sent";
        ret = ESP_OK;
    } else if (command_str == "enable_auto_balancing") {
        ESP_LOGI(TAG, "Publishing ENABLE_AUTO_BALANCING_COMMAND_RECEIVED");
        UI_EnableAutoBalancing cmd_event;
        m_eventBus.publish(cmd_event);
        message = "Enable auto balancing command sent";
        ret = ESP_OK;
     } else if (command_str == "disable_auto_balancing") {
        ESP_LOGI(TAG, "Publishing DISABLE_AUTO_BALANCING_COMMAND_RECEIVED");
        UI_DisableAutoBalancing cmd_event;
        m_eventBus.publish(cmd_event);
        message = "Disable auto balancing command sent";
        ret = ESP_OK;
    } else if (command_str == "enable_fall_detect") {
        ESP_LOGI(TAG, "Publishing ENABLE_FALL_DETECT_COMMAND_RECEIVED");
        UI_EnableFallDetection cmd_event;
        m_eventBus.publish(cmd_event);
        message = "Enable fall detect command sent";
        ret = ESP_OK;
    } else if (command_str == "disable_fall_detect") {
        ESP_LOGI(TAG, "Publishing DISABLE_FALL_DETECT_COMMAND_RECEIVED");
        UI_DisableFallDetection cmd_event;
        m_eventBus.publish(cmd_event);
        message = "Disable fall detect command sent";
        ret = ESP_OK;
    } else if (command_str == "start_pid_tuning") {
        PidTuningTarget target = PidTuningTarget::MOTOR_SPEED_LEFT;
        cJSON *target_item = cJSON_GetObjectItemCaseSensitive(root, "target");
        if (target_item && cJSON_IsString(target_item) && target_item->valuestring) {
            std::string target_str = target_item->valuestring;
            if (target_str == "motor_speed_left" || target_str == "speed_left" || target_str == "pid_speed_left") {
                target = PidTuningTarget::MOTOR_SPEED_LEFT;
            } else if (target_str == "motor_speed_right" || target_str == "speed_right" || target_str == "pid_speed_right") {
                target = PidTuningTarget::MOTOR_SPEED_RIGHT;
            } else {
                httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Unknown PID tuning target");
                return ESP_FAIL;
            }
        }
        ESP_LOGI(TAG, "Publishing START_PID_TUNING_COMMAND_RECEIVED");
        UI_StartPidTuning cmd_event(target);
        m_eventBus.publish(cmd_event);
        message = "PID tuning start command sent";
        ret = ESP_OK;
    } else if (command_str == "cancel_pid_tuning") {
        ESP_LOGI(TAG, "Publishing CANCEL_PID_TUNING_COMMAND_RECEIVED");
        UI_CancelPidTuning cmd_event;
        m_eventBus.publish(cmd_event);
        message = "PID tuning cancel command sent";
        ret = ESP_OK;
    } else if (command_str == "save_pid_tuning") {
        ESP_LOGI(TAG, "Publishing SAVE_PID_TUNING_COMMAND_RECEIVED");
        UI_SavePidTuning cmd_event;
        m_eventBus.publish(cmd_event);
        message = "PID tuning save command sent";
        ret = ESP_OK;
    } else if (command_str == "discard_pid_tuning") {
        ESP_LOGI(TAG, "Publishing DISCARD_PID_TUNING_COMMAND_RECEIVED");
        UI_DiscardPidTuning cmd_event;
        m_eventBus.publish(cmd_event);
        message = "PID tuning discard command sent";
        ret = ESP_OK;
    } else if (command_str == "start_guided_calibration") {
        ESP_LOGI(TAG, "Publishing START_GUIDED_CALIBRATION_COMMAND_RECEIVED");
        UI_StartGuidedCalibration cmd_event;
        m_eventBus.publish(cmd_event);
        message = "Guided calibration start command sent";
        ret = ESP_OK;
    } else if (command_str == "cancel_guided_calibration") {
        ESP_LOGI(TAG, "Publishing CANCEL_GUIDED_CALIBRATION_COMMAND_RECEIVED");
        UI_CancelGuidedCalibration cmd_event;
        m_eventBus.publish(cmd_event);
        message = "Guided calibration cancel command sent";
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
