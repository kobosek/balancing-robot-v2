#include "ApplicationContext.hpp"

#include "BalancingAlgorithm.hpp"
#include "BalanceMonitor.hpp"
#include "BatteryService.hpp"
#include "CommandProcessor.hpp"
#include "CommandApiHandler.hpp"
#include "ConfigApiHandler.hpp"
#include "ConfigData.hpp"
#include "ConfigurationService.hpp"
#include "ControlEventDispatcher.hpp"
#include "ControlModeExecutor.hpp"
#include "EncoderService.hpp"
#include "EventBus.hpp"
#include "GuidedCalibrationService.hpp"
#include "IMUService.hpp"
#include "JsonConfigParser.hpp"
#include "LogBufferService.hpp"
#include "MotorService.hpp"
#include "OTAService.hpp"
#include "OrientationEstimator.hpp"
#include "PidTuningService.hpp"
#include "RobotController.hpp"
#include "OTAApiHandler.hpp"
#include "LogsApiHandler.hpp"
#include "SPIFFSStorageService.hpp"
#include "StateApiHandler.hpp"
#include "StateManager.hpp"
#include "StaticFileHandler.hpp"
#include "TelemetryHandler.hpp"
#include "WebServer.hpp"
#include "WiFiManager.hpp"
#include "esp_check.h"
#include "esp_log.h"

ApplicationContext::ApplicationContext() = default;
ApplicationContext::~ApplicationContext() = default;

esp_err_t ApplicationContext::initialize()
{
    ESP_LOGI(TAG, "Initializing application context");

    esp_err_t ret = initializeCoreServices();
    ESP_RETURN_ON_ERROR(ret, TAG, "Core service initialization failed");

    ret = initializeSupportServices();
    ESP_RETURN_ON_ERROR(ret, TAG, "Support service initialization failed");

    ret = initializeControlSubsystem();
    ESP_RETURN_ON_ERROR(ret, TAG, "Control subsystem initialization failed");

    ret = initializeConnectivitySubsystem();
    ESP_RETURN_ON_ERROR(ret, TAG, "Connectivity subsystem initialization failed");

    return ESP_OK;
}

EventBus& ApplicationContext::eventBus() const
{
    return *m_eventBus;
}

ConfigurationService& ApplicationContext::configService() const
{
    return *m_configService;
}

StateManager& ApplicationContext::stateManager() const
{
    return *m_stateManager;
}

ControlEventDispatcher& ApplicationContext::controlEventDispatcher() const
{
    return *m_controlEventDispatcher;
}

RobotController& ApplicationContext::robotController() const
{
    return *m_robotController;
}

BatteryService& ApplicationContext::batteryService() const
{
    return *m_batteryService;
}

IMUService& ApplicationContext::imuService() const
{
    return *m_imuService;
}

esp_err_t ApplicationContext::initializeCoreServices()
{
    m_eventBus = &EventBus::getInstance();
    ESP_LOGI(TAG, "EventBus accessed");

    m_storageService = std::make_unique<SPIFFSStorageService>();
    ESP_RETURN_ON_FALSE(m_storageService != nullptr, ESP_ERR_NO_MEM, TAG, "Failed to allocate storage service");

    esp_err_t ret = m_storageService->init();
    ESP_RETURN_ON_ERROR(ret, TAG, "Storage init failed");
    ESP_LOGI(TAG, "Storage initialized");

    m_configParser = std::make_unique<JsonConfigParser>();
    ESP_RETURN_ON_FALSE(m_configParser != nullptr, ESP_ERR_NO_MEM, TAG, "Failed to allocate config parser");
    ESP_LOGI(TAG, "Parser initialized");

    m_configService = std::make_shared<ConfigurationService>(*m_storageService, *m_configParser, *m_eventBus);
    ESP_RETURN_ON_FALSE(m_configService != nullptr, ESP_ERR_NO_MEM, TAG, "Failed to allocate config service");

    ret = m_configService->init();
    ESP_RETURN_ON_ERROR(ret, TAG, "Config service init failed");
    ESP_LOGI(TAG, "ConfigService initialized");

    const WebServerConfig webConf = m_configService->getWebServerConfig();
    m_logBufferService = std::make_unique<LogBufferService>();
    ESP_RETURN_ON_FALSE(m_logBufferService != nullptr, ESP_ERR_NO_MEM, TAG, "Failed to allocate log buffer service");
    ret = m_logBufferService->init(
        webConf.web_logs_enabled,
        static_cast<size_t>(webConf.log_buffer_lines),
        static_cast<size_t>(webConf.log_line_max_length)
    );
    ESP_RETURN_ON_ERROR(ret, TAG, "LogBufferService init failed");
    ESP_LOGI(TAG, "LogBufferService initialized");

    const SystemBehaviorConfig behaviorConf = m_configService->getSystemBehaviorConfig();
    const BatteryConfig batteryConf = m_configService->getBatteryConfig();

    m_stateManager = std::make_shared<StateManager>(*m_eventBus, behaviorConf, batteryConf);
    ESP_RETURN_ON_FALSE(m_stateManager != nullptr, ESP_ERR_NO_MEM, TAG, "Failed to allocate state manager");

    ret = m_stateManager->init();
    ESP_RETURN_ON_ERROR(ret, TAG, "StateManager init failed");
    ESP_LOGI(TAG, "StateManager initialized");

    m_controlEventDispatcher = std::make_shared<ControlEventDispatcher>(*m_eventBus);
    ESP_RETURN_ON_FALSE(m_controlEventDispatcher != nullptr, ESP_ERR_NO_MEM, TAG, "Failed to allocate control event dispatcher");

    ret = m_controlEventDispatcher->init();
    ESP_RETURN_ON_ERROR(ret, TAG, "ControlEventDispatcher init failed");
    ESP_LOGI(TAG, "ControlEventDispatcher initialized");

    return ESP_OK;
}

esp_err_t ApplicationContext::initializeSupportServices()
{
    const WiFiConfig wifiConf = m_configService->getWiFiConfig();
    const EncoderConfig encoderConf = m_configService->getEncoderConfig();
    const MotorConfig motorConf = m_configService->getMotorConfig();
    const BatteryConfig batteryConf = m_configService->getBatteryConfig();
    const SystemBehaviorConfig behaviorConf = m_configService->getSystemBehaviorConfig();
    const MPU6050Config imuConf = m_configService->getMpu6050Config();

    m_wifiManager = std::make_unique<WiFiManager>();
    ESP_RETURN_ON_FALSE(m_wifiManager != nullptr, ESP_ERR_NO_MEM, TAG, "Failed to allocate WiFi manager");

    esp_err_t ret = m_wifiManager->init(wifiConf);
    ESP_RETURN_ON_ERROR(ret, TAG, "WiFiManager init failed");

    m_orientationEstimator = std::make_shared<OrientationEstimator>();
    ESP_RETURN_ON_FALSE(m_orientationEstimator != nullptr, ESP_ERR_NO_MEM, TAG, "Failed to allocate orientation estimator");

    m_encoderService = std::make_unique<EncoderService>(encoderConf);
    ESP_RETURN_ON_FALSE(m_encoderService != nullptr, ESP_ERR_NO_MEM, TAG, "Failed to allocate encoder service");

    ret = m_encoderService->init();
    ESP_RETURN_ON_ERROR(ret, TAG, "EncoderService init failed");

    m_batteryService = std::make_shared<BatteryService>(batteryConf, behaviorConf, *m_eventBus);
    ESP_RETURN_ON_FALSE(m_batteryService != nullptr, ESP_ERR_NO_MEM, TAG, "Failed to allocate battery service");

    ret = m_batteryService->init();
    ESP_RETURN_ON_ERROR(ret, TAG, "BatteryService init failed");

    m_motorService = std::make_shared<MotorService>(motorConf, *m_eventBus);
    ESP_RETURN_ON_FALSE(m_motorService != nullptr, ESP_ERR_NO_MEM, TAG, "Failed to allocate motor service");

    ret = m_motorService->init();
    ESP_RETURN_ON_ERROR(ret, TAG, "MotorService init failed");

    m_imuService = std::make_shared<IMUService>(m_orientationEstimator, imuConf, behaviorConf, *m_eventBus);
    ESP_RETURN_ON_FALSE(m_imuService != nullptr, ESP_ERR_NO_MEM, TAG, "Failed to allocate IMU service");

    ret = m_imuService->init();
    ESP_RETURN_ON_ERROR(ret, TAG, "IMUService init failed");
    ESP_LOGI(TAG, "IMU subsystem initialized");

    return ESP_OK;
}

esp_err_t ApplicationContext::initializeControlSubsystem()
{
    const ControlConfig controlConf = m_configService->getControlConfig();
    const SystemBehaviorConfig behaviorConf = m_configService->getSystemBehaviorConfig();
    const EncoderConfig encoderConf = m_configService->getEncoderConfig();
    const RobotDimensionsConfig dimensionConf = m_configService->getRobotDimensionsConfig();
    const PIDConfig anglePidConf = m_configService->getPidAngleConfig();
    const PIDConfig speedLeftPidConf = m_configService->getPidSpeedLeftConfig();
    const PIDConfig speedRightPidConf = m_configService->getPidSpeedRightConfig();
    const PIDConfig yawRatePidConf = m_configService->getPidYawRateConfig();
    const PidTuningConfig pidTuningConf = m_configService->getPidTuningConfig();
    const MotorConfig motorConf = m_configService->getMotorConfig();

    m_commandProcessor = std::make_shared<CommandProcessor>(*m_eventBus, controlConf, behaviorConf);
    ESP_RETURN_ON_FALSE(m_commandProcessor != nullptr, ESP_ERR_NO_MEM, TAG, "Failed to allocate command processor");

    esp_err_t ret = m_commandProcessor->init();
    ESP_RETURN_ON_ERROR(ret, TAG, "CommandProcessor init failed");

    m_balancingAlgorithm = std::make_shared<BalancingAlgorithm>(
        *m_eventBus,
        anglePidConf,
        speedLeftPidConf,
        speedRightPidConf,
        yawRatePidConf,
        controlConf,
        encoderConf,
        dimensionConf
    );
    ESP_RETURN_ON_FALSE(m_balancingAlgorithm != nullptr, ESP_ERR_NO_MEM, TAG, "Failed to allocate balancing algorithm");

    ret = m_balancingAlgorithm->init();
    ESP_RETURN_ON_ERROR(ret, TAG, "BalancingAlgorithm init failed");

    m_balanceMonitor = std::make_shared<BalanceMonitor>(*m_eventBus, behaviorConf);
    ESP_RETURN_ON_FALSE(m_balanceMonitor != nullptr, ESP_ERR_NO_MEM, TAG, "Failed to allocate balance monitor");

    m_pidTuningService = std::make_shared<PidTuningService>(*m_eventBus, *m_configService, *m_encoderService, pidTuningConf);
    ESP_RETURN_ON_FALSE(m_pidTuningService != nullptr, ESP_ERR_NO_MEM, TAG, "Failed to allocate PID tuning service");

    ret = m_pidTuningService->init();
    ESP_RETURN_ON_ERROR(ret, TAG, "PidTuningService init failed");

    m_guidedCalibrationService = std::make_shared<GuidedCalibrationService>(*m_eventBus, pidTuningConf, motorConf);
    ESP_RETURN_ON_FALSE(m_guidedCalibrationService != nullptr, ESP_ERR_NO_MEM, TAG, "Failed to allocate guided calibration service");

    ret = m_guidedCalibrationService->init();
    ESP_RETURN_ON_ERROR(ret, TAG, "GuidedCalibrationService init failed");

    m_controlModeExecutor = std::make_unique<ControlModeExecutor>(
        *m_balancingAlgorithm,
        *m_pidTuningService,
        *m_guidedCalibrationService
    );
    ESP_RETURN_ON_FALSE(m_controlModeExecutor != nullptr, ESP_ERR_NO_MEM, TAG, "Failed to allocate control mode executor");

    m_robotController = std::make_shared<RobotController>(
        m_orientationEstimator,
        *m_encoderService,
        *m_motorService,
        *m_batteryService,
        *m_controlModeExecutor,
        *m_controlEventDispatcher
    );
    ESP_RETURN_ON_FALSE(m_robotController != nullptr, ESP_ERR_NO_MEM, TAG, "Failed to allocate robot controller");
    ESP_LOGI(TAG, "RobotController initialized");

    return ESP_OK;
}

esp_err_t ApplicationContext::initializeConnectivitySubsystem()
{
    const WebServerConfig webConf = m_configService->getWebServerConfig();

    m_otaService = std::make_shared<OTAService>();
    ESP_RETURN_ON_FALSE(m_otaService != nullptr, ESP_ERR_NO_MEM, TAG, "Failed to allocate OTA service");

    esp_err_t ret = m_otaService->init();
    ESP_RETURN_ON_ERROR(ret, TAG, "OTAService init failed");

    auto staticFileHandler = std::make_unique<StaticFileHandler>("/spiffs");
    auto telemetryHandler = std::make_unique<TelemetryHandler>(webConf);
    auto configApiHandler = std::make_unique<ConfigApiHandler>(*m_configService);
    auto commandApiHandler = std::make_unique<CommandApiHandler>(*m_eventBus);
    auto stateApiHandler = std::make_unique<StateApiHandler>(
        *m_stateManager,
        *m_batteryService,
        *m_pidTuningService,
        *m_guidedCalibrationService,
        *m_configService,
        *m_otaService
    );
    auto otaApiHandler = std::make_unique<OTAApiHandler>(*m_otaService);
    auto logsApiHandler = std::make_unique<LogsApiHandler>(*m_logBufferService);

    m_webServer = std::make_shared<WebServer>(
        *m_eventBus,
        std::move(staticFileHandler),
        std::move(telemetryHandler),
        std::move(configApiHandler),
        std::move(commandApiHandler),
        std::move(stateApiHandler),
        std::move(otaApiHandler),
        std::move(logsApiHandler)
    );
    ESP_RETURN_ON_FALSE(m_webServer != nullptr, ESP_ERR_NO_MEM, TAG, "Failed to allocate web server");

    ret = m_webServer->init();
    ESP_RETURN_ON_ERROR(ret, TAG, "WebServer init failed");

    return ESP_OK;
}
