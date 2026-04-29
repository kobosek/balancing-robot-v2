#include "TelemetryHandler.hpp"
#include "EventBus.hpp" // Include event bus for subscription
#include "CONFIG_FullConfigUpdate.hpp" // Include event definition
#include "TELEMETRY_Snapshot.hpp"
#include "BaseEvent.hpp"
#include "ConfigData.hpp" // Need full struct def for WebServerConfig
#include "HttpResponseUtils.hpp"
#include "cJSON.h"
#include <memory>
#include <vector>
#include "esp_check.h"
#include "esp_log.h"
#include <cstring>
#include <algorithm> // For std::max
#include <cstdio>
#include <string>

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
    if (event.is<CONFIG_FullConfigUpdate>()) {
        handleConfigUpdate(event.as<CONFIG_FullConfigUpdate>());
    } else if (event.is<TELEMETRY_Snapshot>()) {
        addTelemetrySnapshot(event.as<TELEMETRY_Snapshot>().snapshot);
    } else {
        ESP_LOGV(TAG, "%s: Received unhandled event '%s'",
                 getHandlerName().c_str(), event.eventName());
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

    std::vector<TelemetryDataPoint> dataToSend;
    std::string json;
    // Note: Interval is no longer fetched from ConfigService here. If needed,
    // the interval should also be stored locally and updated via handleConfigUpdate.
    // For now, assuming the JS doesn't strictly *need* the interval from this specific response.

    do {
        {
            std::lock_guard<std::mutex> lock(m_telemetryMutex);
            dataToSend.assign(m_telemetryBuffer.begin(), m_telemetryBuffer.end());
            m_telemetryBuffer.clear();
        }

        json.reserve(12 + dataToSend.size() * 144);
        json = "{\"data\":[";

        char pointBuffer[256];
        bool format_failed = false;
        for (size_t i = 0; i < dataToSend.size(); ++i) {
            const auto& point = dataToSend[i];
            const int written = std::snprintf(
                pointBuffer,
                sizeof(pointBuffer),
                "%s[%.3f,%.3f,%.3f,%.3f,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f]",
                i == 0 ? "" : ",",
                static_cast<double>(point.pitch_deg),
                static_cast<double>(point.speedLeft_dps),
                static_cast<double>(point.speedRight_dps),
                static_cast<double>(point.batteryVoltage),
                point.systemState,
                static_cast<double>(point.speedSetpointLeft_dps),
                static_cast<double>(point.speedSetpointRight_dps),
                static_cast<double>(point.desiredAngle_deg),
                static_cast<double>(point.yawAngle_deg),
                static_cast<double>(point.targetYawAngle_deg),
                static_cast<double>(point.yawRate_dps),
                static_cast<double>(point.targetYawRate_dps));
            if (written < 0 || written >= static_cast<int>(sizeof(pointBuffer))) {
                ESP_LOGE(TAG, "Failed format telemetry point");
                format_failed = true;
                break;
            }
            json.append(pointBuffer, written);
        }
        if (format_failed) { final_ret = ESP_FAIL; break; }

        json += "]}";
        final_ret = ESP_OK;

    } while (false);

    // --- Send Response or Error ---
    if (final_ret == ESP_OK) {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_hdr(req, "Cache-Control", "no-store");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        esp_err_t send_ret = httpd_resp_send(req, json.c_str(), json.size());
        if (send_ret == ESP_OK) { ESP_LOGV(TAG, "Sent %zu telemetry points.", dataToSend.size()); final_ret = ESP_OK; }
        else { ESP_LOGE(TAG, "Failed to send telemetry JSON response (%s)", esp_err_to_name(send_ret)); final_ret = ESP_FAIL; }
    } else {
        ESP_LOGE(TAG, "Error creating JSON for /data");
        final_ret = sendHttpError(req, HTTPD_500_INTERNAL_SERVER_ERROR, "JSON processing failed");
    }
    return final_ret;
}
