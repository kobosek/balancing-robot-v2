// main/web/include/WebServer.hpp
#pragma once

// --- ADDED: Include the necessary ESP HTTP Server header ---
#include "esp_http_server.h"
// --- END ADDED ---

#include "esp_err.h"
#include "esp_log.h"
#include <string>
#include <memory> // For unique_ptr
#include "TelemetryDataPoint.hpp" // Include the separate definition

// Forward declare dependencies needed by constructor/members
class ConfigurationService;
class StateManager;
class EventBus;

// Forward declare handler classes
class StaticFileHandler;
class TelemetryHandler;
class ConfigApiHandler;
class CommandApiHandler; // For start/stop etc.
class StateApiHandler;

class WebServer {
public:
    // Constructor takes high-level dependencies needed by handlers
    WebServer(ConfigurationService& configService, StateManager& stateManager, EventBus& eventBus);
    ~WebServer(); // Need custom destructor to delete handlers

    esp_err_t init();

    // Expose method to add telemetry data (delegates to TelemetryHandler)
    void addTelemetrySnapshot(const TelemetryDataPoint& data);

private:
    static constexpr const char* TAG = "WebServer";
    httpd_handle_t server = nullptr;

    // Store references to dependencies needed by WS handler
    ConfigurationService& m_configService; // Needed by some handlers
    StateManager& m_stateManager;         // Needed by some handlers
    EventBus& m_eventBus;                 // Needed for WS handler

    // Own instances of the handlers
    std::unique_ptr<StaticFileHandler> m_staticFileHandler;
    std::unique_ptr<TelemetryHandler> m_telemetryHandler;
    std::unique_ptr<ConfigApiHandler> m_configApiHandler;
    std::unique_ptr<CommandApiHandler> m_commandApiHandler; // Keeps start/stop/etc
    std::unique_ptr<StateApiHandler> m_stateApiHandler;

    // Static callback functions (HTTP)
    static esp_err_t static_get_handler(httpd_req_t* req);
    static esp_err_t data_get_handler(httpd_req_t* req);
    static esp_err_t get_config_handler(httpd_req_t* req);
    static esp_err_t set_config_handler(httpd_req_t* req);
    static esp_err_t command_handler(httpd_req_t* req);
    static esp_err_t get_state_handler(httpd_req_t* req);

    // WebSocket Handling
    static esp_err_t websocket_handler(httpd_req_t *req);
    // Declaration now uses types defined in esp_http_server.h
    esp_err_t handleWebSocketFrame(httpd_req_t *req, httpd_ws_frame_t *ws_pkt);
};