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
class WiFiManager;
class WebServer;

// Forward declarations for IMU components
class MPU6050Driver;
class IMUCalibrationService;
class IMUHealthMonitor;
class IMUService;
class OrientationEstimator;

// Forward declarations for other components
class EncoderService;
class MotorService;
class BalancingAlgorithm;
class FallDetector;
class BatteryService;
class CommandProcessor;

// Task class forward declarations
class Task; // Base task class
class IMUFifoTask;
class IMUHealthMonitorTask; // Renamed task
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
    std::unique_ptr<ConfigurationService> m_configService;
    std::unique_ptr<StateManager> m_stateManager;
    std::unique_ptr<RobotController> m_robotController;
    std::unique_ptr<WiFiManager> m_wifiManager;
    std::unique_ptr<WebServer> m_webServer;

    // IMU Components (using unique_ptr)
    std::unique_ptr<MPU6050Driver> m_mpuDriver;
    std::unique_ptr<IMUCalibrationService> m_imuCalibrationService;
    std::unique_ptr<IMUHealthMonitor> m_imuHealthMonitor;
    std::unique_ptr<IMUService> m_imuService;
    std::unique_ptr<OrientationEstimator> m_orientationEstimator;

    // Other Components (using unique_ptr)
    std::unique_ptr<EncoderService> m_encoderService;
    std::unique_ptr<MotorService> m_motorService;
    std::unique_ptr<BalancingAlgorithm> m_balancingAlgorithm;
    std::unique_ptr<FallDetector> m_fallDetector;
    std::unique_ptr<BatteryService> m_batteryService;
    std::unique_ptr<CommandProcessor> m_commandProcessor;

    // Application tasks (using unique_ptr)
    std::unique_ptr<ControlTask> m_controlTask;
    std::unique_ptr<IMUFifoTask> m_imuFifoTask;
    std::unique_ptr<IMUHealthMonitorTask> m_imuHealthMonitorTask;
    std::unique_ptr<BatteryMonitorTask> m_batteryMonitorTask;


    esp_err_t createAndStartTasks(int intervalMs, int batteryIntervalMs);
};