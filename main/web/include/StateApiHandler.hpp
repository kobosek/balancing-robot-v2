#pragma once

#include "esp_http_server.h"
#include "esp_err.h"
#include "esp_log.h"
#include "EventHandler.hpp"

// Forward declare dependencies
class StateManager;
class BalanceMonitor;
class BatteryService;

class StateApiHandler : public EventHandler {
public:
    StateApiHandler(StateManager& stateManager, BalanceMonitor& balanceMonitor, BatteryService& batteryService);
    esp_err_t handleRequest(httpd_req_t *req);

    // EventHandler interface implementation
    void handleEvent(const BaseEvent& event) override;
    std::string getHandlerName() const override { return TAG; }

private:
    static constexpr const char* TAG = "StateApiHandler";
    StateManager& m_stateManager;
    BalanceMonitor& m_balanceMonitor;
    BatteryService& m_batteryService;
};
