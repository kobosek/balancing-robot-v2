// main/core/include/ComponentHandler.hpp
#pragma once

#include <vector>
#include <memory>
#include "esp_err.h"
#include "esp_log.h"

// Forward Declarations
class ConfigurationService;
class StateManager;
class EventBus;
class BalancingAlgorithm;
class WebServer;
class WiFiManager;
class IMUService;
class OrientationEstimator;
class EncoderService;
class MotorService;
class FallDetector;
class BatteryService;
class CommandProcessor;     // <<<--- Already Forward Declared

class ComponentHandler {
public:
    ComponentHandler(ConfigurationService& config, StateManager& stateMgr, EventBus& bus);
    ~ComponentHandler() = default;

    esp_err_t init();

    // Getters (ensure CommandProcessor is available)
    WiFiManager& getWifiManager() { return *m_wifiManager; }
    WebServer& getWebServer() { return *m_webServer; }
    IMUService& getIMUService() { return *m_imuService; }
    OrientationEstimator& getOrientationEstimator() { return *m_orientationEstimator; }
    EncoderService& getEncoderService() { return *m_encoderService; }
    MotorService& getMotorService() { return *m_motorService; }
    BalancingAlgorithm& getBalancingAlgorithm() { return *m_balancingAlgorithm; }
    FallDetector& getFallDetector() { return *m_fallDetector; }
    BatteryService& getBatteryService() { return *m_batteryService; }
    CommandProcessor& getCommandProcessor() { return *m_commandProcessor; } // <<<--- Already Added


private:
    static constexpr const char* TAG = "ComponentHandler";

    ConfigurationService& m_configService;
    StateManager& m_stateManager;
    EventBus& m_eventBus;

    // Managed components
    std::shared_ptr<WiFiManager> m_wifiManager;
    std::shared_ptr<WebServer> m_webServer;
    std::shared_ptr<IMUService> m_imuService;
    std::shared_ptr<OrientationEstimator> m_orientationEstimator;
    std::shared_ptr<EncoderService> m_encoderService;
    std::shared_ptr<MotorService> m_motorService;
    std::shared_ptr<BalancingAlgorithm> m_balancingAlgorithm;
    std::shared_ptr<FallDetector> m_fallDetector;
    std::shared_ptr<BatteryService> m_batteryService;
    std::shared_ptr<CommandProcessor> m_commandProcessor; // <<<--- Already Added
};