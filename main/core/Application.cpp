#include "Application.hpp"

#include "EventBus.hpp"
#include "SPIFFSStorageService.hpp"
#include "JsonConfigParser.hpp"
#include "ConfigurationService.hpp"
#include "ComponentHandler.hpp"
#include "StateManager.hpp"
#include "RobotController.hpp"
#include "SystemState.hpp"
#include "esp_log.h"
#include "esp_check.h"

// Task implementations
#include "IMUTasks.hpp"
#include "BatteryTask.hpp"
#include "ControlTask.hpp"

Application::Application() 
{
    ESP_LOGI(TAG, "Creating Application instance");
}

Application::~Application() 
{
    // Task objects will clean up their own resources in their destructors
    ESP_LOGI(TAG, "Application instance destroyed");
}

esp_err_t Application::init() 
{
    ESP_LOGI(TAG, "Initializing Application");
    
    // Get EventBus singleton
    m_eventBus = &EventBus::getInstance();
    ESP_LOGI(TAG, "EventBus accessed");
    
    // Initialize storage and configuration
    m_storageService = std::make_unique<SPIFFSStorageService>();
    ESP_RETURN_ON_ERROR(m_storageService->init(), TAG, "Storage init failed");
    ESP_LOGI(TAG, "Storage initialized");
    
    m_configParser = std::make_unique<JsonConfigParser>();
    ESP_LOGI(TAG, "Parser initialized");
    
    m_configService = std::make_unique<ConfigurationService>(
        *m_storageService, *m_configParser, *m_eventBus);
    ESP_RETURN_ON_ERROR(m_configService->init(), TAG, "Config service init failed");
    ESP_LOGI(TAG, "ConfigService initialized");
    
    m_componentHandler = std::make_unique<ComponentHandler>(*m_configService, *m_eventBus);
    ESP_RETURN_ON_ERROR(m_componentHandler->initComponents(), TAG, "Component init failed");
    ESP_LOGI(TAG, "ComponentHandler components initialized");

    m_stateManager = std::make_unique<StateManager>(*m_eventBus);
    ESP_LOGI(TAG, "StateManager created");
    
    ESP_RETURN_ON_ERROR(m_stateManager->init(), TAG, "StateManager init failed");
    ESP_LOGI(TAG, "StateManager initialized (Subscriptions Active)");
    
    // Register StateManager with ComponentHandler's components that need it
    ESP_RETURN_ON_ERROR(m_componentHandler->registerStateManager(*m_stateManager), 
                      TAG, "StateManager registration failed");
    ESP_LOGI(TAG, "StateManager registered with components");
    

    m_robotController = std::make_unique<RobotController>(
        m_componentHandler->getOrientationEstimator(),
        m_componentHandler->getEncoderService(),
        m_componentHandler->getMotorService(),
        m_componentHandler->getBalancingAlgorithm(),
        *m_stateManager,
        m_componentHandler->getFallDetector(),
        m_componentHandler->getWebServer(),
        m_componentHandler->getBatteryService(),
        m_componentHandler->getCommandProcessor()
    );
    ESP_LOGI(TAG, "RobotController created");
    
    ESP_RETURN_ON_ERROR(m_robotController->init(*m_eventBus), TAG, "RobotController init failed");
    ESP_LOGI(TAG, "RobotController initialized (Subscriptions Active)");
    
    ESP_LOGI(TAG, "Application initialization complete");
    return ESP_OK;
}

void Application::run() 
{
    ESP_LOGI(TAG, "Running Application");
    
    // Create and start all application tasks
    ESP_ERROR_CHECK_WITHOUT_ABORT(createAndStartTasks());
    
    // Set initial state after everything is ready
    m_stateManager->setState(SystemState::IDLE);
    ESP_LOGI(TAG, "System State set to IDLE");
    
    ESP_LOGI(TAG, "Application running");
}

esp_err_t Application::createAndStartTasks()
{
    ESP_LOGI(TAG, "Creating and starting application tasks");
    
    // Get control loop interval from configuration
    int intervalMs = m_configService->getMainLoopConfig().interval_ms;
    if (intervalMs <= 0 || intervalMs > 100) {
        ESP_LOGW(TAG, "Invalid loop interval %dms from config, defaulting to 10ms", intervalMs);
        intervalMs = 10;
    }
    ESP_LOGI(TAG, "Control Loop Interval: %d ms", intervalMs);
    
    // Create all task objects
    m_controlTask = std::make_unique<ControlTask>(*m_robotController, intervalMs);
    m_imuFifoTask = std::make_unique<IMUFifoTask>(m_componentHandler->getIMUService());
    m_imuWatchdogTask = std::make_unique<IMUWatchdogTask>(m_componentHandler->getIMUService());
    m_batteryMonitorTask = std::make_unique<BatteryMonitorTask>(m_componentHandler->getBatteryService());
    
    // Start control task on Core 1 with highest priority
    if (!m_controlTask->start(configMAX_PRIORITIES - 1, 1, 4096)) {
        ESP_LOGE(TAG, "Failed to start Control task!");
        m_stateManager->setState(SystemState::FATAL_ERROR);
        return ESP_FAIL;
    }
    
    // Start IMU FIFO task on Core 0 with high priority
    if (!m_imuFifoTask->start(configMAX_PRIORITIES - 2, 0, 8192)) {
        ESP_LOGE(TAG, "Failed to start IMU FIFO task!");
        m_stateManager->setState(SystemState::FATAL_ERROR);
        return ESP_FAIL;
    }
    
    // Start IMU Watchdog task on Core 1 with medium-high priority
    if (!m_imuWatchdogTask->start(5, 1, 8192)) {
        ESP_LOGE(TAG, "Failed to start IMU Watchdog task!");
        m_stateManager->setState(SystemState::FATAL_ERROR);
        return ESP_FAIL;
    }
    
    // Start Battery Monitor task on Core 0 with medium priority
    if (!m_batteryMonitorTask->start(5, 0, 3072)) {
        ESP_LOGE(TAG, "Failed to start Battery Monitor task!");
        m_stateManager->setState(SystemState::FATAL_ERROR);
        return ESP_FAIL;
    }
    
    // Here we would create and start other tasks as needed
    
    ESP_LOGI(TAG, "All application tasks started successfully");
    return ESP_OK;
}
