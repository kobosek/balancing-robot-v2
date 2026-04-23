#include "WebServer.hpp"

// Include handler headers
#include "StaticFileHandler.hpp"
#include "TelemetryHandler.hpp"
#include "ConfigApiHandler.hpp"
#include "CommandApiHandler.hpp"
#include "StateApiHandler.hpp"

// Include necessary services/events for handler instantiation/use
#include "ConfigurationService.hpp"
#include "StateManager.hpp"
#include "BalanceMonitor.hpp"
#include "EventBus.hpp"
#include "TelemetryDataPoint.hpp"
#include "UI_JoystickInput.hpp"
#include "ConfigData.hpp" // Needed for config struct
#include "EventTypes.hpp" // Needed for subscription
#include "CONFIG_FullConfigUpdate.hpp" // Needed for subscription
#include "TELEMETRY_Snapshot.hpp"

#include "esp_check.h"
#include "esp_http_server.h"
#include "cJSON.h"
#include <memory>
#include <cstring>
#include <algorithm>

// Constructor: Instantiate handlers, injecting dependencies
WebServer::WebServer(ConfigurationService& configService, StateManager& stateManager, BalanceMonitor& balanceMonitor, BatteryService& batteryService, EventBus& eventBus, const WebServerConfig& initialWebConfig)
    : server(nullptr),
      m_configService(configService),
      m_stateManager(stateManager),
      m_balanceMonitor(balanceMonitor),
      m_batteryService(batteryService),
      m_eventBus(eventBus)
{
    m_staticFileHandler = std::make_unique<StaticFileHandler>("/spiffs");
    m_telemetryHandler = std::make_unique<TelemetryHandler>(initialWebConfig);
    m_configApiHandler = std::make_unique<ConfigApiHandler>(m_configService);
    m_commandApiHandler = std::make_unique<CommandApiHandler>(m_eventBus);
    m_stateApiHandler = std::make_unique<StateApiHandler>(m_stateManager, m_balanceMonitor, m_batteryService);
    ESP_LOGI(TAG, "Webserver handlers created.");
}

// Destructor
WebServer::~WebServer() {
     ESP_LOGI(TAG, "Destroying WebServer and handlers.");
     if (server) { httpd_stop(server); server = nullptr; }
}

// addTelemetrySnapshot
void WebServer::addTelemetrySnapshot(const TelemetryDataPoint& data) {
    if (m_telemetryHandler) { m_telemetryHandler->addTelemetrySnapshot(data); }
    else { ESP_LOGE(TAG, "TelemetryHandler not initialized."); }
}

// EventHandler implementation
void WebServer::handleEvent(const BaseEvent& event) {
    // Central event handler that dispatches to specific handlers based on event type
    switch (event.type) {
        case EventType::CONFIG_FULL_UPDATE:
            // Forward config updates to handlers that need them
            if (m_telemetryHandler) {
                ESP_LOGV(TAG, "Forwarding CONFIG_FULL_UPDATE to TelemetryHandler");
                m_telemetryHandler->handleEvent(event);
            }
            if (m_configApiHandler) {
                ESP_LOGV(TAG, "Forwarding CONFIG_FULL_UPDATE to ConfigApiHandler");
                m_configApiHandler->handleEvent(event);
            }
            break;
        case EventType::TELEMETRY_SNAPSHOT:
            if(m_telemetryHandler) {
                ESP_LOGV(TAG, "Forwarding TELEMETRY_SNAPSHOT to TelemetryHandler");
                m_telemetryHandler->handleEvent(event);
            }
            break;
        default:
            ESP_LOGV(TAG, "%s: Received unhandled event type %d", 
                     getHandlerName().c_str(), static_cast<int>(event.type));
            break;
    }
}

// init: Register handlers, including WebSocket
esp_err_t WebServer::init() {
    ESP_LOGI(TAG, "Initializing web server routing...");
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;
    config.max_uri_handlers = 24;
    config.stack_size = 8192; // Increase stack size to prevent overflow in HTTP server task
    config.global_user_ctx = this; // Pass 'this' WebServer instance

    ESP_RETURN_ON_ERROR(httpd_start(&server, &config), TAG, "Error starting httpd server");
    ESP_LOGI(TAG, "HTTPD server started.");

    const httpd_uri_t root_uri = { "/", HTTP_GET, static_get_handler, nullptr };
    esp_err_t ret = httpd_register_uri_handler(server, &root_uri);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed register root URI");
    ESP_LOGI(TAG, "Registered handler for: / (GET)");

    const httpd_uri_t style_uri = { "/style.css", HTTP_GET, static_get_handler, nullptr };
    ret = httpd_register_uri_handler(server, &style_uri);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed register style URI");
    ESP_LOGI(TAG, "Registered handler for: /style.css (GET)");

    // Register all JS files explicitly
    const char* js_files[] = {
        "/js/main.js", "/js/api.js", "/js/configUI.js", "/js/constants.js",
        "/js/graph.js", "/js/joystick.js", "/js/state.js", "/js/telemetry.js",
        "/js/ui.js", "/js/websocket.js"
    };
    for (const char* js_file : js_files) {
        httpd_uri_t js_uri = { js_file, HTTP_GET, static_get_handler, nullptr };
        ret = httpd_register_uri_handler(server, &js_uri);
        if (ret != ESP_OK) { ESP_LOGE(TAG, "Failed register JS URI: %s (%s)", js_file, esp_err_to_name(ret)); }
        else { ESP_LOGI(TAG, "Registered handler for: %s (GET)", js_file); }
    }

    const httpd_uri_t data_uri = { "/data", HTTP_GET, data_get_handler, nullptr };
    ret = httpd_register_uri_handler(server, &data_uri);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed register data URI");
    ESP_LOGI(TAG, "Registered handler for: /data (GET)");

    const httpd_uri_t get_config_uri = { "/api/config", HTTP_GET, get_config_handler, nullptr };
    ret = httpd_register_uri_handler(server, &get_config_uri);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed register get_config URI");
    ESP_LOGI(TAG, "Registered handler for: /api/config (GET)");

    const httpd_uri_t set_config_uri = { "/api/config", HTTP_POST, set_config_handler, nullptr };
    ret = httpd_register_uri_handler(server, &set_config_uri);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed register set_config URI");
    ESP_LOGI(TAG, "Registered handler for: /api/config (POST)");

    const httpd_uri_t command_uri = { "/api/command", HTTP_POST, command_handler, nullptr };
    ret = httpd_register_uri_handler(server, &command_uri);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed register command URI");
    ESP_LOGI(TAG, "Registered handler for: /api/command (POST)");

    const httpd_uri_t get_state_uri = { "/api/state", HTTP_GET, get_state_handler, nullptr };
    ret = httpd_register_uri_handler(server, &get_state_uri);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed register get_state URI");
    ESP_LOGI(TAG, "Registered handler for: /api/state (GET)");

    // --- Register WebSocket URI Handler ---
    const httpd_uri_t ws_uri = { "/ws", HTTP_GET, websocket_handler, nullptr, true };
    ret = httpd_register_uri_handler(server, &ws_uri);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed register WebSocket URI");
    ESP_LOGI(TAG, "Registered handler for: /ws (WebSocket)");

    // --- Register FALLBACK/WILDCARD file handler LAST ---
    const httpd_uri_t file_uri = { "/*", HTTP_GET, static_get_handler, nullptr };
    ret = httpd_register_uri_handler(server, &file_uri);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed register fallback file URI");
    ESP_LOGI(TAG, "Registered handler for: /* (GET) - Fallback");

    ESP_LOGI(TAG, "All URI handlers registration attempted.");
    return ESP_OK;
}


// --- Static HTTP Handler Implementations ---
esp_err_t WebServer::static_get_handler(httpd_req_t *req) {
    httpd_handle_t server_handle = req->handle; WebServer* instance = static_cast<WebServer*>(httpd_get_global_user_ctx(server_handle));
    if (!instance || !instance->m_staticFileHandler) { ESP_LOGE(TAG, "Static file handler ctx invalid!"); httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Ctx error"); return ESP_FAIL; }
    return instance->m_staticFileHandler->handleRequest(req);
}
esp_err_t WebServer::data_get_handler(httpd_req_t *req) {
    httpd_handle_t server_handle = req->handle; WebServer* instance = static_cast<WebServer*>(httpd_get_global_user_ctx(server_handle));
    if (!instance || !instance->m_telemetryHandler) { ESP_LOGE(TAG, "Telemetry handler ctx invalid!"); httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Ctx error"); return ESP_FAIL; }
    return instance->m_telemetryHandler->handleRequest(req);
}
esp_err_t WebServer::get_config_handler(httpd_req_t *req) {
     httpd_handle_t server_handle = req->handle; WebServer* instance = static_cast<WebServer*>(httpd_get_global_user_ctx(server_handle));
     if (!instance || !instance->m_configApiHandler) { ESP_LOGE(TAG, "Config API handler ctx invalid!"); httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Ctx error"); return ESP_FAIL; }
    return instance->m_configApiHandler->handleGetRequest(req);
}
esp_err_t WebServer::set_config_handler(httpd_req_t *req) {
     httpd_handle_t server_handle = req->handle; WebServer* instance = static_cast<WebServer*>(httpd_get_global_user_ctx(server_handle));
     if (!instance || !instance->m_configApiHandler) { ESP_LOGE(TAG, "Config API handler ctx invalid!"); httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Ctx error"); return ESP_FAIL; }
    return instance->m_configApiHandler->handlePostRequest(req);
}
esp_err_t WebServer::command_handler(httpd_req_t *req) {
     httpd_handle_t server_handle = req->handle; WebServer* instance = static_cast<WebServer*>(httpd_get_global_user_ctx(server_handle));
     if (!instance || !instance->m_commandApiHandler) { ESP_LOGE(TAG, "Command API handler ctx invalid!"); httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Ctx error"); return ESP_FAIL; }
    return instance->m_commandApiHandler->handleRequest(req);
}
esp_err_t WebServer::get_state_handler(httpd_req_t *req) {
     httpd_handle_t server_handle = req->handle; WebServer* instance = static_cast<WebServer*>(httpd_get_global_user_ctx(server_handle));
     if (!instance || !instance->m_stateApiHandler) { ESP_LOGE(TAG, "State API handler ctx invalid!"); httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Ctx error"); return ESP_FAIL; }
    return instance->m_stateApiHandler->handleRequest(req);
}

// --- WebSocket Static Handler ---
esp_err_t WebServer::websocket_handler(httpd_req_t *req) {
    // Handle initial GET upgrade request
    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "WebSocket client connecting (fd=%d)...", httpd_req_to_sockfd(req));
        return ESP_OK;
    }

    // Handle WebSocket frames
    httpd_ws_frame_t ws_pkt;
    uint8_t *buf = nullptr;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_TEXT; // Expect text

    // Receive frame header to get length
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK) {
        if (ret == ESP_ERR_HTTPD_INVALID_REQ || ret == ESP_FAIL) {
             ESP_LOGI(TAG, "WebSocket client disconnected (fd=%d).", httpd_req_to_sockfd(req));
        } else {
             ESP_LOGE(TAG, "httpd_ws_recv_frame (len) failed: %s", esp_err_to_name(ret));
        }
        return ret;
    }
    ESP_LOGV(TAG, "WS frame len is %d", ws_pkt.len);

    // Receive payload if available
    if (ws_pkt.len > 0) {
        buf = (uint8_t*)calloc(1, ws_pkt.len + 1);
        if (!buf) { ESP_LOGE(TAG, "Failed calloc for WS payload"); return ESP_ERR_NO_MEM; }
        ws_pkt.payload = buf;
        ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
        if (ret != ESP_OK) { ESP_LOGE(TAG, "httpd_ws_recv_frame (payload) failed: %s", esp_err_to_name(ret)); free(buf); return ret; }
        buf[ws_pkt.len] = '\0';
        ESP_LOGV(TAG, "WS[%d]: RX Pkt type=%d, len=%d, data=%s", httpd_req_to_sockfd(req), ws_pkt.type, ws_pkt.len, (char*)ws_pkt.payload);
    } else {
         ESP_LOGV(TAG, "WS[%d]: RX Empty frame type=%d", httpd_req_to_sockfd(req), ws_pkt.type);
         return ESP_OK;
    }

    // Process the received frame using the member function
    WebServer* instance = static_cast<WebServer*>(httpd_get_global_user_ctx(req->handle));
    if (!instance) { ESP_LOGE(TAG, "WS handler context invalid!"); free(buf); return ESP_FAIL; }
    ret = instance->handleWebSocketFrame(req, &ws_pkt);

    free(buf); // Free payload buffer
    return ret;
}


// --- WebSocket Member Function Frame Handler ---
esp_err_t WebServer::handleWebSocketFrame(httpd_req_t *req, httpd_ws_frame_t *ws_pkt) {
    // We only process TEXT frames containing our JSON commands
    if (ws_pkt->type != HTTPD_WS_TYPE_TEXT || ws_pkt->len == 0 || ws_pkt->payload == nullptr) {
        return ESP_OK;
    }

    esp_err_t ret = ESP_FAIL; // Default to fail

    // Use unique_ptr for RAII cleanup of cJSON object
    auto cjson_deleter = [](cJSON* ptr){ if(ptr) cJSON_Delete(ptr); };
    std::unique_ptr<cJSON, decltype(cjson_deleter)> root_ptr(cJSON_Parse((const char*)ws_pkt->payload));
    cJSON* root = root_ptr.get();

    if (!root) { ESP_LOGE(TAG, "WS: Failed to parse JSON: %s", (const char*)ws_pkt->payload); return ESP_FAIL; }

    cJSON *type_item = cJSON_GetObjectItemCaseSensitive(root, "type");
    if (!type_item || !cJSON_IsString(type_item) || !type_item->valuestring) { ESP_LOGE(TAG, "WS: Missing/invalid 'type' field"); return ESP_FAIL; }
    std::string type_str = type_item->valuestring;

    // --- Handle "joystick" type ---
    if (type_str == "joystick") {
        cJSON *x_item = cJSON_GetObjectItemCaseSensitive(root, "x");
        cJSON *y_item = cJSON_GetObjectItemCaseSensitive(root, "y");

        if (!x_item || !cJSON_IsNumber(x_item) || !y_item || !cJSON_IsNumber(y_item)) {
            ESP_LOGE(TAG, "WS: Missing/invalid 'x' or 'y' for type 'joystick'");
            return ESP_FAIL; // Bad request format
        }

        float joystick_x = x_item->valuedouble;
        float joystick_y = y_item->valuedouble;

        // Clamp received values to expected range [-1.0, 1.0]
        joystick_x = std::max(-1.0f, std::min(1.0f, joystick_x));
        joystick_y = std::max(-1.0f, std::min(1.0f, joystick_y));

        ESP_LOGV(TAG, "WS: Publishing JOYSTICK_INPUT_RECEIVED: X=%.3f, Y=%.3f", joystick_x, joystick_y);
        UI_JoystickInput js_event(joystick_x, joystick_y);
        m_eventBus.publish(js_event); // Use the member variable

        ret = ESP_OK; // Successfully processed

    } else if (type_str == "ping") {
        // Example: Respond to a custom ping from client
        ESP_LOGD(TAG, "WS: Received ping, sending pong.");
        httpd_ws_frame_t pong_pkt; memset(&pong_pkt, 0, sizeof(httpd_ws_frame_t));
        const char *pong_payload = "{\"type\":\"pong\"}";
        pong_pkt.payload = (uint8_t*)pong_payload;
        pong_pkt.len = strlen(pong_payload);
        pong_pkt.type = HTTPD_WS_TYPE_TEXT;
        ret = httpd_ws_send_frame(req, &pong_pkt); // Send pong back
        if (ret != ESP_OK) { ESP_LOGE(TAG, "httpd_ws_send_frame (pong) failed: %d", ret); }
        // Even if send fails, we processed the ping, so maybe return OK? Depends on desired behavior.
        ret = ESP_OK;

    } else {
        ESP_LOGW(TAG, "WS: Unknown message type received: %s", type_str.c_str());
        ret = ESP_FAIL; // Treat unknown type as an error
    }

    return ret;
}
