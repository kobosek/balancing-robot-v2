#include "TelemetryHandler.hpp"
#include "EventBus.hpp" // Include event bus for subscription
#include "CONFIG_FullConfigUpdate.hpp" // Include event definition
#include "BaseEvent.hpp"
#include "EventTypes.hpp"
#include "ConfigData.hpp" // Need full struct def for WebServerConfig
#include "cJSON.h"
#include <memory>
#include <vector>
#include "esp_check.h"
#include "esp_log.h"
#include <cstring>
#include <algorithm> // For std::max

// Constructor takes initial WebServerConfig
TelemetryHandler::TelemetryHandler(const WebServerConfig& initialWebConfig) :
    // m_configService removed
    m_telemetry_buffer_max_size(100) // Default before applying initial config
{
    applyConfig(initialWebConfig); // Apply initial config
    ESP_LOGI(TAG, "TelemetryHandler constructed.");
}

// Apply config values
void TelemetryHandler::applyConfig(const WebServerConfig& config) {
    m_telemetry_buffer_max_size = std::max((size_t)1, (size_t)config.telemetry_buffer_size); // Ensure at least 1
    ESP_LOGI(TAG, "Applied TelemetryHandler params: BufferSize=%zu", m_telemetry_buffer_max_size);
    // Trim buffer if it exceeds new max size
    std::lock_guard<std::mutex> lock(m_telemetryMutex);
    while (m_telemetryBuffer.size() > m_telemetry_buffer_max_size) {
        m_telemetryBuffer.pop_front();
    }
}

// EventHandler implementation
void TelemetryHandler::handleEvent(const BaseEvent& event) {
    switch (event.type) {
        case EventType::CONFIG_FULL_UPDATE:
            handleConfigUpdate(static_cast<const CONFIG_FullConfigUpdate&>(event));
            break;
            
        default:
            ESP_LOGV(TAG, "%s: Received unhandled event type %d", 
                     getHandlerName().c_str(), static_cast<int>(event.type));
            break;
    }
}

// Handle config update event
void TelemetryHandler::handleConfigUpdate(const CONFIG_FullConfigUpdate& event) {
    ESP_LOGD(TAG, "Handling config update event.");
    applyConfig(event.configData.web);
}

void TelemetryHandler::addTelemetrySnapshot(const TelemetryDataPoint& data) {
    std::lock_guard<std::mutex> lock(m_telemetryMutex);
    // Use the configured buffer size
    if (m_telemetryBuffer.size() >= m_telemetry_buffer_max_size) {
        m_telemetryBuffer.pop_front();
    }
    m_telemetryBuffer.push_back(data);
}

esp_err_t TelemetryHandler::handleRequest(httpd_req_t *req) {
    ESP_LOGV(TAG, "Received request for /data");
    esp_err_t final_ret = ESP_FAIL;

    cJSON *root = nullptr;
    char *json_string = nullptr;
    auto cjson_deleter = [](cJSON* ptr){ if(ptr) cJSON_Delete(ptr); };
    auto char_deleter = [](char* ptr){ if(ptr) free(ptr); };
    std::unique_ptr<cJSON, decltype(cjson_deleter)> root_ptr(nullptr, cjson_deleter);
    std::unique_ptr<char, decltype(char_deleter)> json_str_ptr(nullptr, char_deleter);

    std::vector<TelemetryDataPoint> dataToSend;
    // Note: Interval is no longer fetched from ConfigService here. If needed,
    // the interval should also be stored locally and updated via handleConfigUpdate.
    // For now, assuming the JS doesn't strictly *need* the interval from this specific response.

    do {
        {
            std::lock_guard<std::mutex> lock(m_telemetryMutex);
            dataToSend.assign(m_telemetryBuffer.begin(), m_telemetryBuffer.end());
            m_telemetryBuffer.clear();
        }

        // int intervalMs = m_configService.getMainLoopConfig().interval_ms; // REMOVED

        root_ptr.reset(cJSON_CreateObject()); root = root_ptr.get();
        if (!root) { ESP_LOGE(TAG, "Failed create root JSON"); final_ret = ESP_FAIL; break; }
        // if (!cJSON_AddNumberToObject(root, "interval_ms", intervalMs)) { ESP_LOGE(TAG, "Failed add interval"); final_ret = ESP_FAIL; break; } // REMOVED
        cJSON* dataArray = cJSON_AddArrayToObject(root, "data");
        if (!dataArray) { ESP_LOGE(TAG, "Failed create data array"); final_ret = ESP_FAIL; break; }

        for (const auto& point : dataToSend) {
            cJSON *pointArray = cJSON_CreateArray();
            if (!pointArray) { ESP_LOGW(TAG, "Failed create point array, skipping"); continue; }
            // Add data in the order defined in TelemetryDataPoint.hpp
            cJSON_AddItemToArray(pointArray, cJSON_CreateNumber(point.pitch_deg));             // 0
            cJSON_AddItemToArray(pointArray, cJSON_CreateNumber(point.speedLeft_dps));         // 1
            cJSON_AddItemToArray(pointArray, cJSON_CreateNumber(point.speedRight_dps));        // 2
            cJSON_AddItemToArray(pointArray, cJSON_CreateNumber(point.batteryVoltage));        // 3
            cJSON_AddItemToArray(pointArray, cJSON_CreateNumber(point.systemState));           // 4
            cJSON_AddItemToArray(pointArray, cJSON_CreateNumber(point.speedSetpointLeft_dps)); // 5
            cJSON_AddItemToArray(pointArray, cJSON_CreateNumber(point.speedSetpointRight_dps));// 6
            cJSON_AddItemToArray(pointArray, cJSON_CreateNumber(point.desiredAngle_deg));      // 7
            cJSON_AddItemToArray(pointArray, cJSON_CreateNumber(point.yawRate_dps));           // 8

            cJSON_AddItemToArray(dataArray, pointArray);
        }

        json_str_ptr.reset(cJSON_PrintUnformatted(root)); json_string = json_str_ptr.get();
        if (!json_string) { ESP_LOGE(TAG, "Failed print JSON"); final_ret = ESP_FAIL; break; }
        final_ret = ESP_OK;

    } while (false); // End of JSON creation block

    // --- Send Response or Error ---
    if (final_ret == ESP_OK) {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_hdr(req, "Cache-Control", "no-store");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        esp_err_t send_ret = httpd_resp_send(req, json_string, strlen(json_string));
        if (send_ret == ESP_OK) { ESP_LOGV(TAG, "Sent %zu telemetry points.", dataToSend.size()); final_ret = ESP_OK; }
        else { ESP_LOGE(TAG, "Failed to send telemetry JSON response (%s)", esp_err_to_name(send_ret)); final_ret = ESP_FAIL; }
    } else {
        ESP_LOGE(TAG, "Error creating JSON for /data");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "JSON processing failed");
    }
    return final_ret;
}