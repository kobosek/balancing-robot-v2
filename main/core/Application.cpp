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

// Other required component headers (assuming ComponentHandler isn't managing IMU parts anymore)
#include "EncoderService.hpp"
#include "MotorService.hpp"
#include "BalancingAlgorithm.hpp"
#include "FallDetector.hpp"
#include "BatteryService.hpp"
#include "CommandProcessor.hpp"
#include "WiFiManager.hpp"
#include "WebServer.hpp"

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

    // --- State Manager ---
    m_stateManager = std::make_unique<StateManager>(*m_eventBus);
    ret = m_stateManager->init(); ESP_RETURN_ON_ERROR(ret, TAG, "StateManager init failed");
    ESP_LOGI(TAG, "StateManager initialized"); // Removed "(Subscriptions Active)" from log

    // --- Instantiate Non-IMU Components ---
    // (Assuming these are still valid and needed)
    m_wifiManager = std::make_unique<WiFiManager>();
    ret = m_wifiManager->init(*m_configService); ESP_RETURN_ON_ERROR(ret, TAG, "WiFiManager init failed");

    m_orientationEstimator = std::make_unique<OrientationEstimator>(); // Constructor is simpler now
    // Estimator init happens in IMUService now

    m_encoderService = std::make_unique<EncoderService>(m_configService->getEncoderConfig());
    ret = m_encoderService->init(); ESP_RETURN_ON_ERROR(ret, TAG, "EncoderService init failed");

    m_batteryService = std::make_unique<BatteryService>(m_configService->getBatteryConfig(), *m_eventBus);
    ret = m_batteryService->init(); ESP_RETURN_ON_ERROR(ret, TAG, "BatteryService init failed");

    m_motorService = std::make_unique<MotorService>(m_configService->getMotorConfig(), *m_eventBus);
    ret = m_motorService->init(); ESP_RETURN_ON_ERROR(ret, TAG, "MotorService init failed");

    m_commandProcessor = std::make_unique<CommandProcessor>(*m_eventBus, *m_configService);
    ret = m_commandProcessor->init(); ESP_RETURN_ON_ERROR(ret, TAG, "CommandProcessor init failed");

    m_balancingAlgorithm = std::make_unique<BalancingAlgorithm>(*m_configService, *m_eventBus);
    ret = m_balancingAlgorithm->init(); ESP_RETURN_ON_ERROR(ret, TAG, "BalancingAlgorithm init failed");

    m_fallDetector = std::make_unique<FallDetector>(*m_eventBus); // Assuming simple constructor
    m_fallDetector->reset();

    m_webServer = std::make_unique<WebServer>(*m_configService, *m_stateManager, *m_eventBus);
    ret = m_webServer->init(); ESP_RETURN_ON_ERROR(ret, TAG, "WebServer init failed");


    // --- Instantiate IMU Components ---
    m_mpuDriver = std::make_unique<MPU6050Driver>();
    // Driver init (I2C setup) happens within IMUService init

    m_imuCalibrationService = std::make_unique<IMUCalibrationService>(
        *m_mpuDriver, m_configService->getMpu6050Config(), *m_eventBus);
    // Calibration service init happens within IMUService init

    m_imuHealthMonitor = std::make_unique<IMUHealthMonitor>(*m_mpuDriver, *m_eventBus);
    // Health monitor task started in createAndStartTasks

    m_imuService = std::make_unique<IMUService>(
        *m_mpuDriver,
        *m_imuCalibrationService,
        *m_imuHealthMonitor,
        *m_orientationEstimator, // Pass the estimator instance
        m_configService->getMpu6050Config(),
        *m_eventBus
    );
    ret = m_imuService->init(); ESP_RETURN_ON_ERROR(ret, TAG, "IMUService init failed");
    ESP_LOGI(TAG, "IMU Subsystem Initialized");


    // --- Instantiate Robot Controller (Facade) ---
    // Pass references to the required components
    m_robotController = std::make_unique<RobotController>(
        *m_orientationEstimator, // Use the single estimator instance
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
    ESP_LOGI(TAG, "RobotController initialized"); // Removed "(Subscriptions Active)" from log

    // Remove ComponentHandler or adapt if still used for other groups
    // m_componentHandler = std::make_unique<ComponentHandler>(*m_configService, *m_eventBus);
    // ESP_RETURN_ON_ERROR(m_componentHandler->initComponents(), TAG, "Component init failed");
    // ESP_RETURN_ON_ERROR(m_componentHandler->registerStateManager(*m_stateManager), TAG, "StateManager registration failed");

    // --- Establish Event Subscriptions ---
    // Call subscribeToEvents on components AFTER they are fully initialized
    ESP_LOGI(TAG, "Establishing event subscriptions...");
    m_stateManager->subscribeToEvents(*m_eventBus);
    m_commandProcessor->subscribeToEvents(*m_eventBus);
    m_balancingAlgorithm->subscribeToEvents(*m_eventBus);
    m_motorService->subscribeToEvents(*m_eventBus);
    m_imuService->subscribeToEvents(*m_eventBus); // Handles its own subscriptions
    m_imuCalibrationService->subscribeToEvents(*m_eventBus);
    m_robotController->subscribeToEvents(*m_eventBus); // RobotController listens for target commands
    // Add calls for any other components that need to subscribe (e.g., WebServer for internal events?)
    // m_webServer->subscribeToEvents(*m_eventBus); // If WebServer needs to listen to events
    ESP_LOGI(TAG, "Component event subscriptions established.");


    ESP_LOGI(TAG, "Application initialization complete");
    return ESP_OK;
}


void Application::run()
{
    ESP_LOGI(TAG, "Running Application");

    // Create and start all application tasks
    esp_err_t task_ret = createAndStartTasks();
    if (task_ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to start required tasks. Entering FATAL_ERROR state.");
         m_stateManager->setState(SystemState::FATAL_ERROR);
         // Stop here or let main loop handle sleep
         return;
    }

    // Set initial state after tasks are running
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

    // --- Create Task Objects ---
    // Control Task
    ESP_RETURN_ON_FALSE(m_robotController != nullptr, ESP_FAIL, TAG, "RobotController is null");
    m_controlTask = std::make_unique<ControlTask>(*m_robotController, intervalMs);

    // IMU Tasks
    ESP_RETURN_ON_FALSE(m_imuService != nullptr, ESP_FAIL, TAG, "IMUService is null");
    m_imuFifoTask = std::make_unique<IMUFifoTask>(*m_imuService); // Task for data pipeline

    ESP_RETURN_ON_FALSE(m_imuHealthMonitor != nullptr, ESP_FAIL, TAG, "IMUHealthMonitor is null");
    m_imuHealthMonitorTask = std::make_unique<IMUHealthMonitorTask>(*m_imuHealthMonitor); // Task for health checks

    // Battery Task
    ESP_RETURN_ON_FALSE(m_batteryService != nullptr, ESP_FAIL, TAG, "BatteryService is null");
    m_batteryMonitorTask = std::make_unique<BatteryMonitorTask>(*m_batteryService);


    // --- Start Tasks ---
    // Control task (High priority, Core 1)
    if (!m_controlTask->start(configMAX_PRIORITIES - 1, 1, 4096)) {
        ESP_LOGE(TAG, "Failed to start Control task!"); return ESP_FAIL;
    }

    // IMU FIFO task (High priority, Core 0 - for I2C/ISR interaction)
    if (!m_imuFifoTask->start(configMAX_PRIORITIES - 2, 0, 6144)) { // Increased stack slightly
        ESP_LOGE(TAG, "Failed to start IMU FIFO task!"); return ESP_FAIL;
    }

    // IMU Health Monitor task (Medium priority, Core 1 - less critical timing)
    if (!m_imuHealthMonitorTask->start(5, 1, 4096)) { // Renamed task
        ESP_LOGE(TAG, "Failed to start IMU Health Monitor task!"); return ESP_FAIL;
    }

    // Battery Monitor task (Medium priority, Core 0)
    if (!m_batteryMonitorTask->start(5, 0, 3072)) {
        ESP_LOGE(TAG, "Failed to start Battery Monitor task!"); return ESP_FAIL;
    }

    ESP_LOGI(TAG, "All application tasks started successfully");
    return ESP_OK;
}