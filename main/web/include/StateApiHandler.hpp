#pragma once

#include "esp_http_server.h"
#include "esp_err.h"
#include "esp_log.h"
#include "EventHandler.hpp"

// Forward declare dependencies
class StateManager;
class BatteryService;
class PidTuningService;
class GuidedCalibrationService;
class ConfigurationService;
class OTAService;

class StateApiHandler : public EventHandler {
public:
    StateApiHandler(StateManager& stateManager,
                    BatteryService& batteryService,
                    PidTuningService& pidTuningService,
                    GuidedCalibrationService& guidedCalibrationService,
                    ConfigurationService& configService,
                    OTAService& otaService);
    esp_err_t handleRequest(httpd_req_t *req);

    // EventHandler interface implementation
    void handleEvent(const BaseEvent& event) override;
    std::string getHandlerName() const override { return TAG; }

private:
    static constexpr const char* TAG = "StateApiHandler";
    StateManager& m_stateManager;
    BatteryService& m_batteryService;
    PidTuningService& m_pidTuningService;
    GuidedCalibrationService& m_guidedCalibrationService;
    ConfigurationService& m_configService;
    OTAService& m_otaService;
};
