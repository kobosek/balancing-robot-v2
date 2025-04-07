// main/web/TelemetryHandler.cpp
#include "TelemetryHandler.hpp"
#include "ConfigurationService.hpp" // Need full definition for configService access
#include "cJSON.h"
#include <memory> // For unique_ptr
#include <vector> // For std::vector usage
#include "esp_check.h" // Not strictly needed now, but keep for other checks
#include "esp_log.h"
#include <cstring> // For strlen

TelemetryHandler::TelemetryHandler(ConfigurationService& configService) :
    m_configService(configService)
{}

void TelemetryHandler::addTelemetrySnapshot(const TelemetryDataPoint& data) {
    std::lock_guard<std::mutex> lock(m_telemetryMutex);
    if (m_telemetryBuffer.size() >= TELEMETRY_BUFFER_MAX_SIZE) {
        m_telemetryBuffer.pop_front();
    }
    m_telemetryBuffer.push_back(data);
}

esp_err_t TelemetryHandler::handleRequest(httpd_req_t *req) {
    ESP_LOGV(TAG, "Received request for /data");
    esp_err_t final_ret = ESP_FAIL;

    // --- Declare variables needed outside the do-while ---
    cJSON *root = nullptr;
    char *json_string = nullptr;
    auto cjson_deleter = [](cJSON* ptr){ if(ptr) cJSON_Delete(ptr); };
    auto char_deleter = [](char* ptr){ if(ptr) free(ptr); };
    std::unique_ptr<cJSON, decltype(cjson_deleter)> root_ptr(nullptr, cjson_deleter);
    std::unique_ptr<char, decltype(char_deleter)> json_str_ptr(nullptr, char_deleter);

    // --- Declare dataToSend OUTSIDE the do-while block ---
    std::vector<TelemetryDataPoint> dataToSend;

    // Use a block to scope operations that might fail
    do {
        // Get Data Snapshot (Populate the vector declared outside)
        {
            std::lock_guard<std::mutex> lock(m_telemetryMutex);
            dataToSend.assign(m_telemetryBuffer.begin(), m_telemetryBuffer.end());
            m_telemetryBuffer.clear(); // Clear original buffer
        }

        // Get Interval
        int intervalMs = m_configService.getMainLoopConfig().interval_ms;

        // Create JSON Response
        root_ptr.reset(cJSON_CreateObject());
        root = root_ptr.get();
        if (!root) {
            ESP_LOGE(TAG, "Failed create root JSON");
            final_ret = ESP_FAIL;
            break;
        }

        if (!cJSON_AddNumberToObject(root, "interval_ms", intervalMs)) {
            ESP_LOGE(TAG, "Failed add interval");
            final_ret = ESP_FAIL;
            break;
        }

        cJSON* dataArray = cJSON_AddArrayToObject(root, "data");
        if (!dataArray) {
            ESP_LOGE(TAG, "Failed create data array");
            final_ret = ESP_FAIL;
            break;
        }

        for (const auto& point : dataToSend) { // Iterate over the copied data
            cJSON *pointArray = cJSON_CreateArray();
            if (!pointArray) {
                ESP_LOGW(TAG, "Failed create point array, skipping");
                continue;
            }
            cJSON_AddItemToArray(pointArray, cJSON_CreateNumber(point.pitch_deg));
            cJSON_AddItemToArray(pointArray, cJSON_CreateNumber(point.balanceSpeed_dps));
            cJSON_AddItemToArray(pointArray, cJSON_CreateNumber(point.speedLeft_dps));
            cJSON_AddItemToArray(pointArray, cJSON_CreateNumber(point.speedRight_dps));
            cJSON_AddItemToArray(pointArray, cJSON_CreateNumber(point.effortLeft));
            cJSON_AddItemToArray(pointArray, cJSON_CreateNumber(point.effortRight));
            cJSON_AddItemToArray(pointArray, cJSON_CreateNumber(point.targetAngVel_dps));
            cJSON_AddItemToArray(pointArray, cJSON_CreateNumber(point.batteryVoltage));
            cJSON_AddItemToArray(pointArray, cJSON_CreateNumber(point.systemState));
            cJSON_AddItemToArray(dataArray, pointArray);
        }

        // Convert JSON object to string
        json_str_ptr.reset(cJSON_PrintUnformatted(root));
        json_string = json_str_ptr.get();
        if (!json_string) {
            ESP_LOGE(TAG, "Failed print JSON");
            final_ret = ESP_FAIL;
            break;
        }

        // Mark as potentially successful if we reach here
        final_ret = ESP_OK;

    } while (false); // End of JSON creation block

    // --- Send Response or Error ---
    if (final_ret == ESP_OK) {
        // Send Response
        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_hdr(req, "Cache-Control", "no-store");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

        esp_err_t send_ret = httpd_resp_send(req, json_string, strlen(json_string));
        if (send_ret == ESP_OK) {
            // Now dataToSend is in scope for logging
            ESP_LOGV(TAG, "Sent %zu telemetry points.", dataToSend.size());
            final_ret = ESP_OK; // Confirm success
        } else {
            ESP_LOGE(TAG, "Failed to send telemetry JSON response (%s)", esp_err_to_name(send_ret));
            final_ret = ESP_FAIL; // Mark failure if send fails
        }
    } else {
        // Send Error Response (because JSON creation failed)
        ESP_LOGE(TAG, "Error occurred during JSON creation for /data");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "JSON processing failed");
        // final_ret remains ESP_FAIL
    }

    // Cleanup is handled by unique_ptrs going out of scope
    return final_ret;
}