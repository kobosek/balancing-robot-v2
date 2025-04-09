#include "ComponentHandler.hpp"

// Include ALL necessary service/component headers
#include "ConfigurationService.hpp"
#include "StateManager.hpp"
#include "EventBus.hpp"
#include "WebServer.hpp"
#include "WiFiManager.hpp"
#include "IMUService.hpp"
#include "OrientationEstimator.hpp"
#include "EncoderService.hpp"
#include "MotorService.hpp"
#include "BalancingAlgorithm.hpp"
#include "FallDetector.hpp"
#include "BatteryService.hpp"
#include "CommandProcessor.hpp" // Already Included

#include "esp_log.h"
#include "esp_check.h"

ComponentHandler::ComponentHandler(ConfigurationService& config, EventBus& bus) : // Removed StateManager
    m_configService(config),
    m_eventBus(bus)
{}


esp_err_t ComponentHandler::init(StateManager& stateMgr) { // Added StateManager parameter
    ESP_LOGI(TAG, "Initializing Components via ComponentHandler");
    esp_err_t ret = ESP_OK;

    m_wifiManager = std::make_shared<WiFiManager>();
    ret = m_wifiManager->init(m_configService);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to initialize WiFiManager");

    m_webServer = std::make_shared<WebServer>(m_configService, stateMgr, m_eventBus); // Use passed-in stateMgr
    ret = m_webServer->init();
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to initialize WebServer");

    m_orientationEstimator = std::make_shared<OrientationEstimator>(m_configService.getConfigData().imu);
    m_orientationEstimator->reset();
    m_imuService = std::make_shared<IMUService>(m_configService.getConfigData().imu, m_eventBus, *m_orientationEstimator);
    ret = m_imuService->init();
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to initialize IMUService");
    m_encoderService = std::make_shared<EncoderService>(m_configService.getConfigData().encoder);
    ret = m_encoderService->init();
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to initialize EncoderService");
    m_batteryService = std::make_shared<BatteryService>(m_configService.getConfigData().battery, m_eventBus);
    ret = m_batteryService->init();
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to initialize BatteryService");
    m_batteryService->start();

    m_motorService = std::make_shared<MotorService>(m_configService.getConfigData().motor, m_eventBus);
    ret = m_motorService->init();
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to initialize MotorService");

    m_commandProcessor = std::make_shared<CommandProcessor>(m_eventBus, m_configService);
    ret = m_commandProcessor->init(); // Init loads config, sets up timer etc.
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to initialize CommandProcessor");
    // <<< END MODIFICATION >>>

    m_balancingAlgorithm = std::make_shared<BalancingAlgorithm>(m_configService, m_eventBus);
    ret = m_balancingAlgorithm->init();
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to initialize BalancingAlgorithm");

    m_fallDetector = std::make_shared<FallDetector>(m_eventBus);
    m_fallDetector->reset();

    ESP_LOGI(TAG, "ComponentHandler initialization complete");
    return ESP_OK;
}