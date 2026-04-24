#pragma once

#include <memory>
#include "esp_err.h"

class EventBus;
class SPIFFSStorageService;
class JsonConfigParser;
class ConfigurationService;
class StateManager;
class RobotController;
class ControlModeExecutor;
class ControlEventDispatcher;
class WiFiManager;
class WebServer;
class IMUService;
class OrientationEstimator;
class EncoderService;
class MotorService;
class BalancingAlgorithm;
class BalanceMonitor;
class BatteryService;
class CommandProcessor;
class PidTuningService;
class OTAService;
class GuidedCalibrationService;

class ApplicationContext {
public:
    ApplicationContext();
    ~ApplicationContext();

    esp_err_t initialize();

    EventBus& eventBus() const;
    ConfigurationService& configService() const;
    StateManager& stateManager() const;
    ControlEventDispatcher& controlEventDispatcher() const;
    RobotController& robotController() const;
    BatteryService& batteryService() const;
    IMUService& imuService() const;

    std::shared_ptr<ConfigurationService> configServiceHandle() const { return m_configService; }
    std::shared_ptr<StateManager> stateManagerHandle() const { return m_stateManager; }
    std::shared_ptr<ControlEventDispatcher> controlEventDispatcherHandle() const { return m_controlEventDispatcher; }
    std::shared_ptr<RobotController> robotControllerHandle() const { return m_robotController; }
    std::shared_ptr<BalanceMonitor> balanceMonitorHandle() const { return m_balanceMonitor; }
    std::shared_ptr<BatteryService> batteryServiceHandle() const { return m_batteryService; }
    std::shared_ptr<MotorService> motorServiceHandle() const { return m_motorService; }
    std::shared_ptr<CommandProcessor> commandProcessorHandle() const { return m_commandProcessor; }
    std::shared_ptr<BalancingAlgorithm> balancingAlgorithmHandle() const { return m_balancingAlgorithm; }
    std::shared_ptr<PidTuningService> pidTuningServiceHandle() const { return m_pidTuningService; }
    std::shared_ptr<GuidedCalibrationService> guidedCalibrationServiceHandle() const { return m_guidedCalibrationService; }
    std::shared_ptr<OTAService> otaServiceHandle() const { return m_otaService; }
    std::shared_ptr<WebServer> webServerHandle() const { return m_webServer; }
    std::shared_ptr<IMUService> imuServiceHandle() const { return m_imuService; }

private:
    static constexpr const char* TAG = "AppContext";

    esp_err_t initializeCoreServices();
    esp_err_t initializeSupportServices();
    esp_err_t initializeControlSubsystem();
    esp_err_t initializeConnectivitySubsystem();

    EventBus* m_eventBus = nullptr;
    std::unique_ptr<SPIFFSStorageService> m_storageService;
    std::unique_ptr<JsonConfigParser> m_configParser;
    std::shared_ptr<ConfigurationService> m_configService;
    std::shared_ptr<StateManager> m_stateManager;
    std::shared_ptr<ControlEventDispatcher> m_controlEventDispatcher;
    std::unique_ptr<ControlModeExecutor> m_controlModeExecutor;
    std::shared_ptr<RobotController> m_robotController;
    std::unique_ptr<WiFiManager> m_wifiManager;
    std::shared_ptr<WebServer> m_webServer;

    std::shared_ptr<IMUService> m_imuService;
    std::shared_ptr<OrientationEstimator> m_orientationEstimator;

    std::unique_ptr<EncoderService> m_encoderService;
    std::shared_ptr<MotorService> m_motorService;
    std::shared_ptr<BalancingAlgorithm> m_balancingAlgorithm;
    std::shared_ptr<BalanceMonitor> m_balanceMonitor;
    std::shared_ptr<BatteryService> m_batteryService;
    std::shared_ptr<CommandProcessor> m_commandProcessor;
    std::shared_ptr<PidTuningService> m_pidTuningService;
    std::shared_ptr<OTAService> m_otaService;
    std::shared_ptr<GuidedCalibrationService> m_guidedCalibrationService;
};
