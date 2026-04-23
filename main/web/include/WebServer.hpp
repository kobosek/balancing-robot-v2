#pragma once

#include "esp_http_server.h"
#include "esp_err.h"
#include "esp_log.h"
#include <string>
#include <memory>
#include "TelemetryDataPoint.hpp"
#include "ConfigData.hpp" // Include full definition
#include "EventHandler.hpp"

// Forward declare dependencies needed by constructor/members
class ConfigurationService; // Still needed for ConfigApiHandler
class StateManager;
class BalanceMonitor;
class BatteryService;
class PidTuningService;
class GuidedCalibrationService;
class OTAService;
class EventBus;
class BaseEvent;

// Forward declare handler classes
class StaticFileHandler;
class TelemetryHandler;
class ConfigApiHandler;
class CommandApiHandler;
class StateApiHandler;
class OTAApiHandler;

class WebServer : public EventHandler {
public:
    // Constructor takes high-level dependencies + WebServerConfig
    WebServer(ConfigurationService& configService,
              StateManager& stateManager,
              BalanceMonitor& balanceMonitor,
              BatteryService& batteryService,
              PidTuningService& pidTuningService,
              GuidedCalibrationService& guidedCalibrationService,
              OTAService& otaService,
              EventBus& eventBus,
              const WebServerConfig& initialWebConfig);
    ~WebServer();

    esp_err_t init();

    // Expose method to add telemetry data (delegates to TelemetryHandler)
    void addTelemetrySnapshot(const TelemetryDataPoint& data);

    // EventHandler interface implementation
    void handleEvent(const BaseEvent& event) override;
    std::string getHandlerName() const override { return TAG; }


private:
    static constexpr const char* TAG = "WebServer";
    httpd_handle_t server = nullptr;

    // Store references to dependencies needed by WS handler or specific handlers
    ConfigurationService& m_configService;
    StateManager& m_stateManager;
    BalanceMonitor& m_balanceMonitor;
    BatteryService& m_batteryService;
    PidTuningService& m_pidTuningService;
    GuidedCalibrationService& m_guidedCalibrationService;
    OTAService& m_otaService;
    EventBus& m_eventBus;

    // Own instances of the handlers
    std::unique_ptr<StaticFileHandler> m_staticFileHandler;
    std::unique_ptr<TelemetryHandler> m_telemetryHandler;
    std::unique_ptr<ConfigApiHandler> m_configApiHandler;
    std::unique_ptr<CommandApiHandler> m_commandApiHandler;
    std::unique_ptr<StateApiHandler> m_stateApiHandler;
    std::unique_ptr<OTAApiHandler> m_otaApiHandler;

    // Static callback functions (HTTP)
    static esp_err_t static_get_handler(httpd_req_t* req);
    static esp_err_t data_get_handler(httpd_req_t* req);
    static esp_err_t get_config_handler(httpd_req_t* req);
    static esp_err_t set_config_handler(httpd_req_t* req);
    static esp_err_t command_handler(httpd_req_t* req);
    static esp_err_t get_state_handler(httpd_req_t* req);
    static esp_err_t get_ota_status_handler(httpd_req_t* req);
    static esp_err_t ota_upload_handler(httpd_req_t* req);

    // WebSocket Handling
    static esp_err_t websocket_handler(httpd_req_t *req);
    esp_err_t handleWebSocketFrame(httpd_req_t *req, httpd_ws_frame_t *ws_pkt);
};
