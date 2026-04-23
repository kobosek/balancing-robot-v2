// main/web/CommandApiHandler.cpp
#include "CommandApiHandler.hpp"
#include "EventBus.hpp"             // Need full definition
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
#include <array>
#include <string>
#include "esp_http_server.h" // <<< ADDED for httpd_resp_* functions
#include "esp_log.h"         // <<< ADDED for logging

CommandApiHandler::CommandApiHandler(EventBus& eventBus) : m_eventBus(eventBus) {
    ESP_LOGI(TAG, "CommandApiHandler constructed.");
}

// EventHandler implementation
void CommandApiHandler::handleEvent(const BaseEvent& event) {
    ESP_LOGV(TAG, "%s: Received unhandled event '%s'",
             getHandlerName().c_str(), event.eventName());
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
    std::string message;

    const auto publishCommand = [&](const char* logName, auto&& event, const char* successMessage) {
        ESP_LOGI(TAG, "Publishing %s", logName);
        m_eventBus.publish(event);
        message = successMessage;
        ret = ESP_OK;
    };

    struct SimpleCommandRoute {
        const char* command;
        const char* logName;
        const char* successMessage;
        void (*publish)(EventBus&);
    };

    const auto simpleRoutes = std::array<SimpleCommandRoute, 10>{{
        {"start", "START_COMMAND_RECEIVED", "Start command sent", [](EventBus& bus) { UI_StartBalancing event; bus.publish(event); }},
        {"stop", "STOP_COMMAND_RECEIVED", "Stop command sent", [](EventBus& bus) { UI_Stop event; bus.publish(event); }},
        {"calibrate", "CALIBRATE_COMMAND_RECEIVED", "Calibrate command sent", [](EventBus& bus) { UI_CalibrateImu event; bus.publish(event); }},
        {"enable_auto_balancing", "ENABLE_AUTO_BALANCING_COMMAND_RECEIVED", "Enable auto balancing command sent", [](EventBus& bus) { UI_EnableAutoBalancing event; bus.publish(event); }},
        {"disable_auto_balancing", "DISABLE_AUTO_BALANCING_COMMAND_RECEIVED", "Disable auto balancing command sent", [](EventBus& bus) { UI_DisableAutoBalancing event; bus.publish(event); }},
        {"enable_fall_detect", "ENABLE_FALL_DETECT_COMMAND_RECEIVED", "Enable fall detect command sent", [](EventBus& bus) { UI_EnableFallDetection event; bus.publish(event); }},
        {"disable_fall_detect", "DISABLE_FALL_DETECT_COMMAND_RECEIVED", "Disable fall detect command sent", [](EventBus& bus) { UI_DisableFallDetection event; bus.publish(event); }},
        {"cancel_pid_tuning", "CANCEL_PID_TUNING_COMMAND_RECEIVED", "PID tuning cancel command sent", [](EventBus& bus) { UI_CancelPidTuning event; bus.publish(event); }},
        {"save_pid_tuning", "SAVE_PID_TUNING_COMMAND_RECEIVED", "PID tuning save command sent", [](EventBus& bus) { UI_SavePidTuning event; bus.publish(event); }},
        {"discard_pid_tuning", "DISCARD_PID_TUNING_COMMAND_RECEIVED", "PID tuning discard command sent", [](EventBus& bus) { UI_DiscardPidTuning event; bus.publish(event); }},
    }};

    for (const SimpleCommandRoute& route : simpleRoutes) {
        if (command_str == route.command) {
            ESP_LOGI(TAG, "Publishing %s", route.logName);
            route.publish(m_eventBus);
            message = route.successMessage;
            ret = ESP_OK;
            break;
        }
    }

    if (ret != ESP_OK && command_str == "start_pid_tuning") {
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
        UI_StartPidTuning cmd_event(target);
        publishCommand("START_PID_TUNING_COMMAND_RECEIVED", cmd_event, "PID tuning start command sent");
    } else if (ret != ESP_OK && command_str == "start_guided_calibration") {
        UI_StartGuidedCalibration cmd_event;
        publishCommand("START_GUIDED_CALIBRATION_COMMAND_RECEIVED", cmd_event, "Guided calibration start command sent");
    } else if (ret != ESP_OK && command_str == "cancel_guided_calibration") {
        UI_CancelGuidedCalibration cmd_event;
        publishCommand("CANCEL_GUIDED_CALIBRATION_COMMAND_RECEIVED", cmd_event, "Guided calibration cancel command sent");
    }

    if (ret != ESP_OK) {
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
