#pragma once

#include "esp_http_server.h"
#include "esp_err.h"
#include "esp_log.h"
#include "EventHandler.hpp"

// Forward declare dependencies
class EventBus;

class CommandApiHandler : public EventHandler {
public:
    CommandApiHandler(EventBus& eventBus);
    esp_err_t handleRequest(httpd_req_t *req);

    // EventHandler interface implementation
    void handleEvent(const BaseEvent& event) override;
    std::string getHandlerName() const override { return TAG; }
    
private:
    static constexpr const char* TAG = "CommandApiHandler";
    EventBus& m_eventBus;
};