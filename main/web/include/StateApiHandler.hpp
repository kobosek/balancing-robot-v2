#pragma once

#include "esp_http_server.h"
#include "esp_err.h"
#include "esp_log.h"

// Forward declare dependencies
class StateManager;

class StateApiHandler {
public:
    StateApiHandler(StateManager& stateManager);
    esp_err_t handleRequest(httpd_req_t *req);

private:
    static constexpr const char* TAG = "StateApiHandler";
    StateManager& m_stateManager;
};