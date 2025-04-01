#pragma once

#include "interfaces/IWebServer.hpp"

#include "esp_log.h"
#include "esp_http_server.h"

class ComponentHandler;

class WebServer : public IWebServer {
    public:
        WebServer(ComponentHandler&, IRuntimeConfig&);
        esp_err_t init(const IRuntimeConfig&) override;
        esp_err_t onConfigUpdate(const IRuntimeConfig&) override;
        
        void update_telemetry(float, float, float, float);

    private:
        static constexpr const char* TAG = "WebServer";
        httpd_handle_t server;
        float m_angle;
        float m_desiredSpeed;
        float m_speed;
        float m_rmse;

        ComponentHandler& componentHandler;
        IRuntimeConfig& runtimeConfig;

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