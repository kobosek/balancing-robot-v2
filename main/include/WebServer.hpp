#pragma once

#include "interfaces/IWebServer.hpp" // Includes TelemetryDataPoint struct definition

#include "esp_log.h"
#include "esp_http_server.h"
#include "freertos/semphr.h" // For SemaphoreHandle_t
#include <vector>            // For std::vector

class ComponentHandler;
class IRuntimeConfig; // Forward declaration

class WebServer : public IWebServer {
    public:
        WebServer(ComponentHandler&, IRuntimeConfig&);
        esp_err_t init(const IRuntimeConfig&) override;
        esp_err_t onConfigUpdate(const IRuntimeConfig&) override;

        // Method to add telemetry data to the buffer
        void addTelemetryData(const TelemetryDataPoint& data) override;

    private:
        static constexpr const char* TAG = "WebServer";
        httpd_handle_t server = nullptr; // Initialize to nullptr

        // Telemetry buffer and synchronization
        std::vector<TelemetryDataPoint> telemetryBuffer;
        SemaphoreHandle_t telemetryMutex = nullptr; // Initialize to nullptr
        static constexpr size_t maxBufferSize = 64; // Max points to buffer

        ComponentHandler& componentHandler;
        IRuntimeConfig& runtimeConfig; // Store reference to access config (e.g., loop interval)

        static esp_err_t static_get_handler(httpd_req_t*);
        static esp_err_t data_get_handler(httpd_req_t*);
        static esp_err_t get_config_handler(httpd_req_t*);
        static esp_err_t set_config_handler(httpd_req_t*);

        esp_err_t handle_static_get(httpd_req_t*);
        esp_err_t handle_data_get(httpd_req_t*);
        esp_err_t handle_get_config(httpd_req_t*);
        esp_err_t handle_set_config(httpd_req_t*);

        void set_content_type_from_file(httpd_req_t*, const char*);
};
