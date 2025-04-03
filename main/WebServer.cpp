#include "include/WebServer.hpp"
#include "include/ComponentHandler.hpp"
#include "include/RuntimeConfig.hpp" // Needed for loop interval

#include "esp_spiffs.h"
#include "esp_vfs.h"
#include <string.h>
#include "cJSON.h" // Needed for JSON formatting
#include "freertos/FreeRTOS.h" // For pdMS_TO_TICKS
#include "freertos/task.h"     // For task delay/yield if needed

#define MAX_FILE_PATH_LENGTH 256
#define CHUNK_SIZE 1024

WebServer::WebServer(ComponentHandler& componentHandler, IRuntimeConfig& runtimeConfig)
    : server(nullptr), componentHandler(componentHandler), runtimeConfig(runtimeConfig) {} // Remove old telemetry members

esp_err_t WebServer::init(const IRuntimeConfig& /*config_param*/) { // Parameter unused now
    ESP_LOGI(TAG, "Initializing web server");

    // Initialize the telemetry mutex
    telemetryMutex = xSemaphoreCreateMutex();
    if (!telemetryMutex) {
        ESP_LOGE(TAG, "Failed to create telemetry mutex");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Telemetry mutex created");

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;

    esp_err_t ret = httpd_start(&server, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error starting server!");
        return ret;
    }

    ESP_LOGI(TAG, "Web server started successfully");

    httpd_uri_t data_uri = {
        .uri       = "/data",
        .method    = HTTP_GET,
        .handler   = data_get_handler,
        .user_ctx  = this
    };
    httpd_register_uri_handler(server, &data_uri);

    httpd_uri_t root_uri = {
        .uri       = "/",
        .method    = HTTP_GET,
        .handler   = static_get_handler,
        .user_ctx  = this
    };
    httpd_register_uri_handler(server, &root_uri);

    httpd_uri_t file_download = {
        .uri       = "/*",
        .method    = HTTP_GET,
        .handler   = static_get_handler,
        .user_ctx  = this
    };
    httpd_register_uri_handler(server, &file_download);
    
    httpd_uri_t get_config_uri = {
        .uri       = "/api/config",
        .method    = HTTP_GET,
        .handler   = get_config_handler,
        .user_ctx  = this
    };
    httpd_register_uri_handler(server, &get_config_uri);

    httpd_uri_t set_config_uri = {
        .uri       = "/api/config",
        .method    = HTTP_POST,
        .handler   = set_config_handler,
        .user_ctx  = this
    };
    httpd_register_uri_handler(server, &set_config_uri);

    ESP_LOGI(TAG, "All URI handlers registered");
    return ESP_OK;
}

// Implementation for adding telemetry data to the buffer
void WebServer::addTelemetryData(const TelemetryDataPoint& data) {
    if (xSemaphoreTake(telemetryMutex, pdMS_TO_TICKS(10)) == pdTRUE) { // Wait briefly
        if (telemetryBuffer.size() >= maxBufferSize) {
            // Remove the oldest element to make space if buffer is full
            telemetryBuffer.erase(telemetryBuffer.begin());
            ESP_LOGV(TAG, "Telemetry buffer full, removed oldest entry.");
        }
        telemetryBuffer.push_back(data);
        xSemaphoreGive(telemetryMutex);
        ESP_LOGV(TAG, "Added telemetry point. Buffer size: %d", telemetryBuffer.size());
    } else {
        ESP_LOGW(TAG, "Failed to acquire telemetry mutex for writing");
    }
}


esp_err_t WebServer::static_get_handler(httpd_req_t *req) {
    WebServer* server = static_cast<WebServer*>(req->user_ctx);
    return server->handle_static_get(req);
}

esp_err_t WebServer::data_get_handler(httpd_req_t *req) {
    WebServer* server = static_cast<WebServer*>(req->user_ctx);
    return server->handle_data_get(req);
}

esp_err_t WebServer::get_config_handler(httpd_req_t *req) {
    WebServer* server = static_cast<WebServer*>(req->user_ctx);
    return server->handle_get_config(req);
}

esp_err_t WebServer::set_config_handler(httpd_req_t *req) {
    WebServer* server = static_cast<WebServer*>(req->user_ctx);
    return server->handle_set_config(req);
}

esp_err_t WebServer::handle_static_get(httpd_req_t *req) {
    char filepath[MAX_FILE_PATH_LENGTH];
    
    if (strcmp(req->uri, "/") == 0) {
        strlcpy(filepath, "/spiffs/index.html", sizeof(filepath));
    } else {
        strlcpy(filepath, "/spiffs", sizeof(filepath));
        strlcat(filepath, req->uri, sizeof(filepath));
    }
    
    ESP_LOGD(TAG, "Serving file: %s", filepath);

    FILE* file = fopen(filepath, "r");
    if (file == NULL) {
        ESP_LOGE(TAG, "Failed to open file : %s", filepath);
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    set_content_type_from_file(req, filepath);

    char *chunk = (char *)malloc(CHUNK_SIZE);
    if (chunk == NULL) {
        fclose(file);
        ESP_LOGE(TAG, "Failed to allocate memory for file reading");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    size_t chunksize;
    do {
        chunksize = fread(chunk, 1, CHUNK_SIZE, file);
        if (chunksize > 0) {
            if (httpd_resp_send_chunk(req, chunk, chunksize) != ESP_OK) {
                fclose(file);
                free(chunk);
                ESP_LOGE(TAG, "File sending failed!");
                httpd_resp_sendstr_chunk(req, NULL);
                httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to send file");
                return ESP_FAIL;
            }
        }
    } while (chunksize > 0);

    fclose(file);
    free(chunk);
    httpd_resp_send_chunk(req, NULL, 0);
    ESP_LOGD(TAG, "File sent successfully: %s", filepath);
    return ESP_OK;
}

// Modified handler for /data endpoint to send batched data as JSON
esp_err_t WebServer::handle_data_get(httpd_req_t *req) {
    std::vector<TelemetryDataPoint> dataToSend;

    // Safely copy and clear the shared buffer
    if (xSemaphoreTake(telemetryMutex, pdMS_TO_TICKS(50)) == pdTRUE) { // Wait a bit longer here
        dataToSend = telemetryBuffer; // Copy constructor
        telemetryBuffer.clear();
        xSemaphoreGive(telemetryMutex);
    } else {
        ESP_LOGE(TAG, "Failed to acquire telemetry mutex for reading");
        httpd_resp_send_500(req); // Indicate server error
        return ESP_FAIL;
    }

    // Get the current loop interval from runtime config
    // Note: Assumes RuntimeConfig class has this getter implemented later
    int intervalMs = runtimeConfig.getMainLoopIntervalMs();

    // Create JSON response
    cJSON *root = cJSON_CreateObject();
    if (!root) {
        ESP_LOGE(TAG, "Failed to create root cJSON object");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    cJSON_AddNumberToObject(root, "interval_ms", intervalMs);

    cJSON *dataArray = cJSON_CreateArray();
    if (!dataArray) {
        ESP_LOGE(TAG, "Failed to create data cJSON array");
        cJSON_Delete(root);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    cJSON_AddItemToObject(root, "data", dataArray);

    for (const auto& point : dataToSend) {
        cJSON *pointArray = cJSON_CreateArray();
        if (!pointArray) {
            ESP_LOGW(TAG, "Failed to create point cJSON array, skipping point");
            continue; // Skip this point if allocation fails
        }
        cJSON_AddItemToArray(pointArray, cJSON_CreateNumber(point.pitch));
        cJSON_AddItemToArray(pointArray, cJSON_CreateNumber(point.desiredSpeed));
        cJSON_AddItemToArray(pointArray, cJSON_CreateNumber(point.currentSpeed));
        cJSON_AddItemToArray(pointArray, cJSON_CreateNumber(point.rmse));
        cJSON_AddItemToArray(dataArray, pointArray);
    }

    char *json_string = cJSON_PrintUnformatted(root); // Use unformatted for smaller size
    cJSON_Delete(root); // Clean up cJSON structure

    if (!json_string) {
        ESP_LOGE(TAG, "Failed to print cJSON to string");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    // Send the JSON response
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
    httpd_resp_set_hdr(req, "Pragma", "no-cache");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*"); // Keep CORS header if needed

    esp_err_t send_ret = httpd_resp_send(req, json_string, strlen(json_string));
    free(json_string); // Free the allocated string

    ESP_LOGV(TAG, "Sent %u telemetry points. Interval: %d ms", dataToSend.size(), intervalMs);
    return send_ret;
}


esp_err_t WebServer::handle_get_config(httpd_req_t *req) {
    std::string json = runtimeConfig.toJson();
   
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
    httpd_resp_set_hdr(req, "Pragma", "no-cache");
    httpd_resp_sendstr(req, json.c_str());
   
    ESP_LOGD(TAG, "Sent configuration data");
    return ESP_OK;
}

esp_err_t WebServer::handle_set_config(httpd_req_t *req) {
    char* content = nullptr;
    size_t content_len = req->content_len;

    content = static_cast<char*>(malloc(content_len + 1));
    if (content == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate memory for request content");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    int ret = httpd_req_recv(req, content, content_len);
    if (ret <= 0) {
        free(content);
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "Timeout occurred while receiving request");
            httpd_resp_send_408(req);
        }
        return ESP_FAIL;
    }
    content[content_len] = '\0';

    ESP_LOGD(TAG, "Received configuration update request");

    if (runtimeConfig.fromJson(std::string(content)) != ESP_OK) {
        free(content);
        ESP_LOGE(TAG, "Failed to parse configuration JSON");
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    free(content);

    if (runtimeConfig.save() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save configuration");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to save configuration");
        return ESP_FAIL;
    }

    componentHandler.notifyConfigUpdate();

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"status\":\"success\",\"message\":\"Configuration updated and applied successfully\"}");

    ESP_LOGI(TAG, "Configuration updated successfully");
    return ESP_OK;
}

void WebServer::set_content_type_from_file(httpd_req_t *req, const char *filename) {
    const char *dot = strrchr(filename, '.');
    if (dot && dot != filename) {
        const char *type = dot + 1;
        if (strcasecmp(type, "html") == 0) httpd_resp_set_type(req, "text/html");
        else if (strcasecmp(type, "css") == 0) httpd_resp_set_type(req, "text/css");
        else if (strcasecmp(type, "js") == 0) httpd_resp_set_type(req, "application/javascript");
        else if (strcasecmp(type, "png") == 0) httpd_resp_set_type(req, "image/png");
        else if (strcasecmp(type, "jpg") == 0) httpd_resp_set_type(req, "image/jpeg");
        else if (strcasecmp(type, "ico") == 0) httpd_resp_set_type(req, "image/x-icon");
        else httpd_resp_set_type(req, "text/plain");
    } else {
        httpd_resp_set_type(req, "text/plain");
    }
    ESP_LOGD(TAG, "Set content type for file: %s", filename);
}

esp_err_t WebServer::onConfigUpdate(const IRuntimeConfig&) {
    ESP_LOGI(TAG, "Configuration update received");
    // Implement any necessary updates for the WebServer here
    return ESP_OK;
}
