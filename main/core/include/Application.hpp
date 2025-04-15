#pragma once

#include <memory>
#include "freertos/FreeRTOS.h"
#include "esp_err.h"

// Forward declarations
class EventBus;
class SPIFFSStorageService;
class JsonConfigParser;
class ConfigurationService;
class ComponentHandler;
class StateManager;
class RobotController;

// Task class forward declarations
class Task;
class IMUFifoTask;
class IMUWatchdogTask;
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
    std::unique_ptr<ComponentHandler> m_componentHandler;
    std::unique_ptr<StateManager> m_stateManager;
    std::unique_ptr<RobotController> m_robotController;
    
    // Application tasks
    std::unique_ptr<ControlTask> m_controlTask;
    std::unique_ptr<IMUFifoTask> m_imuFifoTask;
    std::unique_ptr<IMUWatchdogTask> m_imuWatchdogTask;
    std::unique_ptr<BatteryMonitorTask> m_batteryMonitorTask;
    
    // Task management methods
    esp_err_t createAndStartTasks();
};
