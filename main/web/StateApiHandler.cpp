// main/web/StateApiHandler.cpp
#include "StateApiHandler.hpp"
#include "StateManager.hpp" // Need full definition
#include "SystemState.hpp"  // Need full definition
#include "cJSON.h"
#include <memory>
#include <string>
#include "esp_log.h"
#include "esp_http_server.h"

StateApiHandler::StateApiHandler(StateManager& stateManager) : m_stateManager(stateManager) {}

// Helper function remains the same
static const char* stateToString(SystemState state) {
    switch(state) {
        case SystemState::INIT:             return "INITIALIZING";
        case SystemState::IDLE:             return "IDLE";
        case SystemState::BALANCING:        return "BALANCING";
        case SystemState::FALLEN:           return "FALLEN";
        case SystemState::SHUTDOWN:         return "SHUTDOWN";
        case SystemState::FATAL_ERROR:      return "ERROR";
        default:                            return "UNKNOWN";
    }
}

esp_err_t StateApiHandler::handleRequest(httpd_req_t *req) {
    ESP_LOGD(TAG, "Received request for /api/state (GET)");
    SystemState currentState = m_stateManager.getCurrentState();
    bool autoRecoveryEnabled = m_stateManager.isAutoRecoveryEnabled(); // <<< Get status
    bool fallDetectionEnabled = m_stateManager.isFallDetectionEnabled(); // <<< Get status
    const char* stateStr = stateToString(currentState);

    auto cjson_deleter = [](cJSON* ptr){ if(ptr) cJSON_Delete(ptr); };
    auto char_deleter = [](char* ptr){ if(ptr) free(ptr); };
    std::unique_ptr<cJSON, decltype(cjson_deleter)> root_ptr(cJSON_CreateObject());
    cJSON* root = root_ptr.get();
    if (!root) {
        httpd_resp_send_500(req); return ESP_FAIL;
    }

    cJSON_AddNumberToObject(root, "state_id", static_cast<int>(currentState));
    cJSON_AddStringToObject(root, "state_name", stateStr);
    cJSON_AddBoolToObject(root, "auto_recovery_enabled", autoRecoveryEnabled); // <<< ADDED
    cJSON_AddBoolToObject(root, "fall_detection_enabled", fallDetectionEnabled); // <<< ADDED
    
    std::unique_ptr<char, decltype(char_deleter)> json_str_ptr(cJSON_PrintUnformatted(root));
    char* json_string = json_str_ptr.get();
    if (!json_string) {
        httpd_resp_send_500(req); return ESP_FAIL;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_sendstr(req, json_string);
}