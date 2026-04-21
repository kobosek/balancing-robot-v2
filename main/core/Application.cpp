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
#include "IMUHealthMonitor.hpp"     // Health Monitor
#include "IMUService.hpp"           // Refactored Service
#include "OrientationEstimator.hpp" // Estimator
#include "FIFOProcessor.hpp"        // FIFO processing
// --- Include Task headers ---
#include "FIFOTask.hpp"           // FIFO processing task 
#include "HealthMonitorTask.hpp"  // Health monitoring task
#include "BatteryTask.hpp"
#include "ControlTask.hpp"

// Other required component headers
#include "EncoderService.hpp"
#include "MotorService.hpp"
#include "BalancingAlgorithm.hpp"
#include "BalanceMonitor.hpp"
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

    m_configService = std::make_shared<ConfigurationService>(*m_storageService, *m_configParser, *m_eventBus);
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
    m_stateManager = std::make_shared<StateManager>(*m_eventBus, behaviorConf); // Pass behavior config
    ret = m_stateManager->init(); ESP_RETURN_ON_ERROR(ret, TAG, "StateManager init failed");
    ESP_LOGI(TAG, "StateManager initialized");

    // --- Instantiate Non-IMU Components ---
    m_wifiManager = std::make_unique<WiFiManager>();
    ret = m_wifiManager->init(wifiConf); ESP_RETURN_ON_ERROR(ret, TAG, "WiFiManager init failed"); // Pass WiFi config

    m_orientationEstimator = std::make_shared<OrientationEstimator>();
    // Estimator init happens in IMUService init

    m_encoderService = std::make_unique<EncoderService>(encoderConf); // Pass encoder config
    ret = m_encoderService->init(); ESP_RETURN_ON_ERROR(ret, TAG, "EncoderService init failed");

    m_batteryService = std::make_shared<BatteryService>(batteryConf, behaviorConf, *m_eventBus); // Pass battery and relevant behavior config
    ret = m_batteryService->init(); ESP_RETURN_ON_ERROR(ret, TAG, "BatteryService init failed");

    m_motorService = std::make_shared<MotorService>(motorConf, *m_eventBus); // Pass motor config
    ret = m_motorService->init(); ESP_RETURN_ON_ERROR(ret, TAG, "MotorService init failed");

    m_commandProcessor = std::make_shared<CommandProcessor>(*m_eventBus, controlConf, behaviorConf); // Pass control and behavior config
    ret = m_commandProcessor->init(); ESP_RETURN_ON_ERROR(ret, TAG, "CommandProcessor init failed");

    m_balancingAlgorithm = std::make_shared<BalancingAlgorithm>(
        *m_eventBus,
        anglePidConf, speedLeftPidConf, speedRightPidConf, yawRatePidConf,
        encoderConf, dimensionConf
    );
    ret = m_balancingAlgorithm->init(); ESP_RETURN_ON_ERROR(ret, TAG, "BalancingAlgorithm init failed");

    m_balanceMonitor = std::make_shared<BalanceMonitor>(*m_eventBus, behaviorConf);

    // WebServer needs ConfigService mainly to pass to handlers that NEED it (ConfigApiHandler)
    m_webServer = std::make_shared<WebServer>(*m_configService, *m_stateManager, *m_balanceMonitor, *m_eventBus, webConf);
    ret = m_webServer->init(); ESP_RETURN_ON_ERROR(ret, TAG, "WebServer init failed");

    // Create IMUService with encapsulated components
    m_imuService = std::make_shared<IMUService>(
        m_orientationEstimator,
        imuConf,         // Pass IMU config struct
        behaviorConf,    // Pass behavior config
        *m_eventBus
    );
    ret = m_imuService->init(); ESP_RETURN_ON_ERROR(ret, TAG, "IMUService init failed");
    ESP_LOGI(TAG, "IMU Subsystem Initialized");

    // --- Instantiate Robot Controller (Facade) ---
    m_robotController = std::make_shared<RobotController>(
        m_orientationEstimator,
        *m_encoderService,
        *m_motorService,
        *m_balancingAlgorithm,
        *m_stateManager,
        *m_batteryService,
        *m_commandProcessor
    );
    ret = m_robotController->init(*m_eventBus); ESP_RETURN_ON_ERROR(ret, TAG, "RobotController init failed");
    ESP_LOGI(TAG, "RobotController initialized");

    m_eventBus->subscribe(m_commandProcessor, {
        EventType::SYSTEM_STATE_CHANGED,
        EventType::UI_JOYSTICK_INPUT,
        EventType::CONFIG_FULL_UPDATE
    });
    
    // BalancingAlgorithm subscriptions - now using EventHandler
    m_eventBus->subscribe(m_balancingAlgorithm, {
        EventType::CONFIG_FULL_UPDATE,
        EventType::CONFIG_PID_UPDATE
    });
    
    // BalanceMonitor subscriptions
    m_eventBus->subscribe(m_balanceMonitor, {
        EventType::IMU_ORIENTATION_DATA,
        EventType::SYSTEM_STATE_CHANGED,
        EventType::CONFIG_FULL_UPDATE,
        EventType::UI_ENABLE_FALL_RECOVERY,
        EventType::UI_DISABLE_FALL_RECOVERY,
        EventType::UI_ENABLE_FALL_DETECTION,
        EventType::UI_DISABLE_FALL_DETECTION
    });

    // MotorService subscriptions - now using EventHandler
    m_eventBus->subscribe(m_motorService, {
        EventType::SYSTEM_STATE_CHANGED
    });
    
    // BatteryService subscriptions - now using EventHandler
    m_eventBus->subscribe(m_batteryService, {
        EventType::CONFIG_FULL_UPDATE
    });
    
    // StateManager subscriptions - now using EventHandler
    m_eventBus->subscribe(m_stateManager, {
        EventType::BALANCE_FALL_DETECTED,
        EventType::BALANCE_RECOVERY_DETECTED,
        EventType::UI_START_BALANCING,
        EventType::UI_STOP,
        EventType::BATTERY_STATUS_UPDATE,

        EventType::UI_CALIBRATE_IMU,
        EventType::IMU_CALIBRATION_COMPLETED,
        EventType::IMU_CALIBRATION_REJECTED,
        EventType::IMU_RECOVERY_SUCCEEDED,
        EventType::IMU_RECOVERY_FAILED,
        EventType::CONFIG_FULL_UPDATE
    });
    
    // ConfigurationService subscriptions - now using EventHandler
    m_eventBus->subscribe(m_configService, {
        EventType::CONFIG_GYRO_OFFSETS_UPDATE
    });
    
    // IMUService subscriptions - now using EventHandler
    m_eventBus->subscribe(m_imuService, {
        EventType::CONFIG_FULL_UPDATE,
        EventType::CONFIG_IMU_UPDATE,
        EventType::IMU_COMMUNICATION_ERROR,
        EventType::IMU_CALIBRATION_REQUEST,
        EventType::SYSTEM_STATE_CHANGED,
    });
    
    // RobotController subscriptions - now using EventHandler
    m_eventBus->subscribe(m_robotController, {
        EventType::MOTION_TARGET_SET
    });
    
    // WebServer subscriptions - now using EventHandler
    m_eventBus->subscribe(m_webServer, {
        EventType::CONFIG_FULL_UPDATE,
        EventType::TELEMETRY_SNAPSHOT
    });

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

    ESP_RETURN_ON_FALSE(m_batteryService != nullptr, ESP_FAIL, TAG, "BatteryService is null");
    m_batteryMonitorTask = std::make_unique<BatteryMonitorTask>(*m_batteryService, batteryIntervalMs);

    // --- Start Tasks (Using hardcoded params for now, move to config later if needed) ---
    if (!m_controlTask->start(configMAX_PRIORITIES - 1, 1, 4096)) { ESP_LOGE(TAG, "Failed to start Control task!"); return ESP_FAIL; }
    if (!m_batteryMonitorTask->start(5, 0, 3072)) { ESP_LOGE(TAG, "Failed to start Battery Monitor task!"); return ESP_FAIL; }
    
    // Start the IMU-related tasks from the IMUService
    ESP_RETURN_ON_FALSE(m_imuService != nullptr, ESP_FAIL, TAG, "IMUService is null");
    if (!m_imuService->startTasks()) { ESP_LOGE(TAG, "Failed to start IMU tasks!"); return ESP_FAIL; }
    // IMUCalibration now handled directly by IMUService, no separate task needed

    ESP_LOGI(TAG, "All application tasks started successfully");
    return ESP_OK;
}