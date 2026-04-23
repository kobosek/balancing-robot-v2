#pragma once

#include <memory>
#include "freertos/FreeRTOS.h"
#include "esp_err.h"

// Forward declarations for core services
class EventBus;
class SPIFFSStorageService;
class JsonConfigParser;
class ConfigurationService;
class StateManager;
class RobotController;
class ControlEventDispatcher;
class WiFiManager;
class WebServer;

// Forward declarations for IMU components
class MPU6050Driver;
class IMUService;
class OrientationEstimator;

// Forward declarations for other components
class EncoderService;
class MotorService;
class BalancingAlgorithm;
class BalanceMonitor;
class BatteryService;
class CommandProcessor;
class PidTuningService;
class OTAService;
class GuidedCalibrationService;

// Task class forward declarations
class Task; // Base task class
class BatteryMonitorTask;
class ControlTask;

class Application {
public:
    Application();
    ~Application();

    esp_err_t init();
    void run();

private:
    static constexpr const char* TAG = "Application";

    // Core services
    EventBus* m_eventBus = nullptr; // Singleton reference
    std::unique_ptr<SPIFFSStorageService> m_storageService;
    std::unique_ptr<JsonConfigParser> m_configParser;
    std::shared_ptr<ConfigurationService> m_configService;
    std::shared_ptr<StateManager> m_stateManager;
    std::shared_ptr<ControlEventDispatcher> m_controlEventDispatcher;
    std::shared_ptr<RobotController> m_robotController;
    std::unique_ptr<WiFiManager> m_wifiManager;
    std::shared_ptr<WebServer> m_webServer;

    // IMU Components (using unique_ptr)
    std::unique_ptr<MPU6050Driver> m_mpuDriver;
    std::shared_ptr<IMUService> m_imuService;
    std::shared_ptr<OrientationEstimator> m_orientationEstimator;

    // Other Components (using unique_ptr)
    std::unique_ptr<EncoderService> m_encoderService;
    std::shared_ptr<MotorService> m_motorService;
    std::shared_ptr<BalancingAlgorithm> m_balancingAlgorithm;
    std::shared_ptr<BalanceMonitor> m_balanceMonitor;
    std::shared_ptr<BatteryService> m_batteryService;
    std::shared_ptr<CommandProcessor> m_commandProcessor;
    std::shared_ptr<PidTuningService> m_pidTuningService;
    std::shared_ptr<OTAService> m_otaService;
    std::shared_ptr<GuidedCalibrationService> m_guidedCalibrationService;

    // Application tasks (using unique_ptr)
    std::unique_ptr<ControlTask> m_controlTask;
    std::unique_ptr<BatteryMonitorTask> m_batteryMonitorTask;

    esp_err_t createAndStartTasks(int intervalMs, int batteryIntervalMs);
};
