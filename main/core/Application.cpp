// ================================================
// File: main/core/Application.cpp
// ================================================
#include "Application.hpp"

#include "EventBus.hpp"
#include "SPIFFSStorageService.hpp"
#include "JsonConfigParser.hpp"
#include "ConfigurationService.hpp"
#include "StateManager.hpp"
#include "RobotController.hpp"
#include "SystemState.hpp"
#include "esp_log.h"
#include "esp_check.h"

// --- Include Headers for new/modified IMU components ---
#include "mpu6050.hpp"              // Driver
#include "IMUCalibrationService.hpp" // Calibration Service
#include "IMUHealthMonitor.hpp"     // Health Monitor
#include "IMUService.hpp"           // Refactored Service
#include "OrientationEstimator.hpp" // Estimator
// --- Include Task headers ---
#include "IMUTasks.hpp"           // Includes both FIFO and Health tasks now
#include "BatteryTask.hpp"
#include "ControlTask.hpp"

// Other required component headers
#include "EncoderService.hpp"
#include "MotorService.hpp"
#include "BalancingAlgorithm.hpp"
#include "FallDetector.hpp"
#include "BatteryService.hpp"
#include "CommandProcessor.hpp"
#include "WiFiManager.hpp"
#include "WebServer.hpp"
#include "ConfigData.hpp" // Needed for config struct types

Application::Application()
{
    ESP_LOGI(TAG, "Creating Application instance");
}

Application::~Application()
{
    ESP_LOGI(TAG, "Application instance destroyed");
    // unique_ptrs handle cleanup automatically
}

esp_err_t Application::init()
{
    ESP_LOGI(TAG, "Initializing Application");
    esp_err_t ret;

    // --- Core Services ---
    m_eventBus = &EventBus::getInstance();
    ESP_LOGI(TAG, "EventBus accessed");

    m_storageService = std::make_unique<SPIFFSStorageService>();
    ret = m_storageService->init(); ESP_RETURN_ON_ERROR(ret, TAG, "Storage init failed");
    ESP_LOGI(TAG, "Storage initialized");

    m_configParser = std::make_unique<JsonConfigParser>();
    ESP_LOGI(TAG, "Parser initialized");

    m_configService = std::make_unique<ConfigurationService>(*m_storageService, *m_configParser, *m_eventBus);
    ret = m_configService->init(); ESP_RETURN_ON_ERROR(ret, TAG, "Config service init failed");
    ESP_LOGI(TAG, "ConfigService initialized");

    // --- Pre-fetch necessary initial config structs ---
    WiFiConfig wifiConf = m_configService->getWiFiConfig();
    MPU6050Config imuConf = m_configService->getMpu6050Config();
    SystemBehaviorConfig behaviorConf = m_configService->getSystemBehaviorConfig();
    ControlConfig controlConf = m_configService->getControlConfig();
    EncoderConfig encoderConf = m_configService->getEncoderConfig();
    MotorConfig motorConf = m_configService->getMotorConfig();
    BatteryConfig batteryConf = m_configService->getBatteryConfig();
    RobotDimensionsConfig dimensionConf = m_configService->getRobotDimensionsConfig();
    WebServerConfig webConf = m_configService->getWebServerConfig();
    PIDConfig anglePidConf = m_configService->getPidAngleConfig();
    PIDConfig speedLeftPidConf = m_configService->getPidSpeedLeftConfig();
    PIDConfig speedRightPidConf = m_configService->getPidSpeedRightConfig();
    PIDConfig yawRatePidConf = m_configService->getPidYawRateConfig();

    // --- State Manager ---
    m_stateManager = std::make_unique<StateManager>(*m_eventBus, behaviorConf); // Pass behavior config
    ret = m_stateManager->init(); ESP_RETURN_ON_ERROR(ret, TAG, "StateManager init failed");
    ESP_LOGI(TAG, "StateManager initialized");

    // --- Instantiate Non-IMU Components ---
    m_wifiManager = std::make_unique<WiFiManager>();
    ret = m_wifiManager->init(wifiConf); ESP_RETURN_ON_ERROR(ret, TAG, "WiFiManager init failed"); // Pass WiFi config

    m_orientationEstimator = std::make_unique<OrientationEstimator>();
    // Estimator init happens in IMUService init

    m_encoderService = std::make_unique<EncoderService>(encoderConf); // Pass encoder config
    ret = m_encoderService->init(); ESP_RETURN_ON_ERROR(ret, TAG, "EncoderService init failed");

    m_batteryService = std::make_unique<BatteryService>(batteryConf, behaviorConf, *m_eventBus); // Pass battery and relevant behavior config
    ret = m_batteryService->init(); ESP_RETURN_ON_ERROR(ret, TAG, "BatteryService init failed");

    m_motorService = std::make_unique<MotorService>(motorConf, *m_eventBus); // Pass motor config
    ret = m_motorService->init(); ESP_RETURN_ON_ERROR(ret, TAG, "MotorService init failed");

    m_commandProcessor = std::make_unique<CommandProcessor>(*m_eventBus, controlConf, behaviorConf); // Pass control and behavior config
    ret = m_commandProcessor->init(); ESP_RETURN_ON_ERROR(ret, TAG, "CommandProcessor init failed");

    m_balancingAlgorithm = std::make_unique<BalancingAlgorithm>(
        *m_eventBus,
        anglePidConf, speedLeftPidConf, speedRightPidConf, yawRatePidConf,
        encoderConf, dimensionConf
    );
    ret = m_balancingAlgorithm->init(); ESP_RETURN_ON_ERROR(ret, TAG, "BalancingAlgorithm init failed");

    m_fallDetector = std::make_unique<FallDetector>(*m_eventBus, behaviorConf); // Pass behavior config
    // FallDetector constructor now loads config and resets

    // WebServer needs ConfigService mainly to pass to handlers that NEED it (ConfigApiHandler)
    m_webServer = std::make_unique<WebServer>(*m_configService, *m_stateManager, *m_eventBus, webConf); // Pass web config
    ret = m_webServer->init(); ESP_RETURN_ON_ERROR(ret, TAG, "WebServer init failed");

    // --- Instantiate IMU Components ---
    m_mpuDriver = std::make_unique<MPU6050Driver>();
    // Driver init happens within IMUService init

    m_imuCalibrationService = std::make_unique<IMUCalibrationService>(*m_mpuDriver, imuConf, *m_eventBus); // Pass IMU config
    // Calibration service init happens within IMUService init

    m_imuHealthMonitor = std::make_unique<IMUHealthMonitor>(*m_mpuDriver, *m_eventBus, behaviorConf); // Pass behavior config
    // Health monitor constructor now loads config

    m_imuService = std::make_unique<IMUService>(
        *m_mpuDriver,
        *m_imuCalibrationService,
        *m_imuHealthMonitor,
        *m_orientationEstimator,
        imuConf, // Pass IMU config struct
        *m_eventBus
    );
    ret = m_imuService->init(); ESP_RETURN_ON_ERROR(ret, TAG, "IMUService init failed");
    ESP_LOGI(TAG, "IMU Subsystem Initialized");

    // --- Instantiate Robot Controller (Facade) ---
    m_robotController = std::make_unique<RobotController>(
        *m_orientationEstimator,
        *m_encoderService,
        *m_motorService,
        *m_balancingAlgorithm,
        *m_stateManager,
        *m_fallDetector,
        *m_webServer,
        *m_batteryService,
        *m_commandProcessor
    );
    ret = m_robotController->init(*m_eventBus); ESP_RETURN_ON_ERROR(ret, TAG, "RobotController init failed");
    ESP_LOGI(TAG, "RobotController initialized");

    // --- Establish Event Subscriptions ---
    ESP_LOGI(TAG, "Establishing event subscriptions...");
    m_configService->subscribeToEvents(*m_eventBus); // Config service listens for offset updates
    // Components that need config updates subscribe themselves via their subscribeToEvents method
    m_stateManager->subscribeToEvents(*m_eventBus);
    m_batteryService->subscribeToEvents(*m_eventBus);
    m_imuHealthMonitor->subscribeToEvents(*m_eventBus);
    m_commandProcessor->subscribeToEvents(*m_eventBus);
    m_balancingAlgorithm->subscribeToEvents(*m_eventBus);
    m_fallDetector->subscribeToEvents(*m_eventBus); // Fall detector needs to subscribe now
    m_motorService->subscribeToEvents(*m_eventBus);
    m_imuService->subscribeToEvents(*m_eventBus);
    m_imuCalibrationService->subscribeToEvents(*m_eventBus);
    // m_imuHealthMonitor->subscribeToEvents(*m_eventBus); // Already subscribed
    m_robotController->subscribeToEvents(*m_eventBus);
    m_webServer->subscribeToEvents(*m_eventBus); // Webserver/handlers subscribe for config updates

    ESP_LOGI(TAG, "Component event subscriptions established.");

    ESP_LOGI(TAG, "Application initialization complete");
    return ESP_OK;
}


void Application::run()
{
    ESP_LOGI(TAG, "Running Application");

    // Use config fetched during init for task creation params
    int intervalMs = m_configService->getMainLoopConfig().interval_ms; // Get interval here
    int batteryIntervalMs = m_configService->getSystemBehaviorConfig().battery_read_interval_ms; // Get interval here

    esp_err_t task_ret = createAndStartTasks(intervalMs, batteryIntervalMs); // Pass intervals
    if (task_ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to start required tasks. Entering FATAL_ERROR state.");
         m_stateManager->setState(SystemState::FATAL_ERROR);
         return;
    }

    m_stateManager->setState(SystemState::IDLE);
    ESP_LOGI(TAG, "System State set to IDLE");

    ESP_LOGI(TAG, "Application running");
}

esp_err_t Application::createAndStartTasks(int intervalMs, int batteryIntervalMs) // Keep parameters here
{
    ESP_LOGI(TAG, "Creating and starting application tasks");

    // Validate intervals passed as parameters
    if (intervalMs <= 0 || intervalMs > 1000) {
        ESP_LOGW(TAG, "Invalid loop interval %dms provided, defaulting to 10ms", intervalMs);
        intervalMs = 10;
    }
    if (batteryIntervalMs <= 0 || batteryIntervalMs > 60000) {
        ESP_LOGW(TAG, "Invalid battery interval %dms provided, defaulting to 5000ms", batteryIntervalMs);
        batteryIntervalMs = 5000;
    }
    ESP_LOGI(TAG, "Control Loop Interval: %d ms, Battery Read Interval: %d ms", intervalMs, batteryIntervalMs);


    // --- Create Task Objects ---
    ESP_RETURN_ON_FALSE(m_robotController != nullptr, ESP_FAIL, TAG, "RobotController is null");
    m_controlTask = std::make_unique<ControlTask>(*m_robotController, intervalMs);

    ESP_RETURN_ON_FALSE(m_imuService != nullptr, ESP_FAIL, TAG, "IMUService is null");
    m_imuFifoTask = std::make_unique<IMUFifoTask>(*m_imuService);

    ESP_RETURN_ON_FALSE(m_imuHealthMonitor != nullptr, ESP_FAIL, TAG, "IMUHealthMonitor is null");
    m_imuHealthMonitorTask = std::make_unique<IMUHealthMonitorTask>(*m_imuHealthMonitor);

    ESP_RETURN_ON_FALSE(m_batteryService != nullptr, ESP_FAIL, TAG, "BatteryService is null");
    m_batteryMonitorTask = std::make_unique<BatteryMonitorTask>(*m_batteryService, batteryIntervalMs);

    ESP_RETURN_ON_FALSE(m_imuCalibrationService != nullptr, ESP_FAIL, TAG, "IMUCalibrationService is null");
    m_imuCalibrationTask = std::make_unique<IMUCalibrationTask>(*m_imuCalibrationService, *m_eventBus);

    // --- Start Tasks (Using hardcoded params for now, move to config later if needed) ---
    if (!m_controlTask->start(configMAX_PRIORITIES - 1, 1, 4096)) { ESP_LOGE(TAG, "Failed to start Control task!"); return ESP_FAIL; }
    if (!m_imuFifoTask->start(configMAX_PRIORITIES - 2, 0, 6144)) { ESP_LOGE(TAG, "Failed to start IMU FIFO task!"); return ESP_FAIL; }
    if (!m_imuHealthMonitorTask->start(5, 1, 4096)) { ESP_LOGE(TAG, "Failed to start IMU Health Monitor task!"); return ESP_FAIL; }
    if (!m_batteryMonitorTask->start(5, 0, 3072)) { ESP_LOGE(TAG, "Failed to start Battery Monitor task!"); return ESP_FAIL; }
    if (!m_imuCalibrationTask->start(5, 1, 4096)) { ESP_LOGE(TAG, "Failed to start IMU Calibration task!"); return ESP_FAIL; }

    ESP_LOGI(TAG, "All application tasks started successfully");
    return ESP_OK;
}