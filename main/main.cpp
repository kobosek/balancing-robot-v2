#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include <cmath>
#include <string>
#include <memory>

#include "EventBus.hpp"
#include "SPIFFSStorageService.hpp"
#include "JsonConfigParser.hpp"
#include "ConfigurationService.hpp"
#include "StateManager.hpp"
#include "IMUService.hpp" // Included
#include "SystemState.hpp"
#include "ConfigData.hpp"

#include "ComponentHandler.hpp"
#include "RobotController.hpp"

// Logging Tags
static const char* TAG = "AppMain";
static const char* CONTROL_TASK_TAG = "ControlTask";

// Task Argument Struct
struct ControlTaskArgs {
    RobotController& robotController;
    int loopIntervalMs;
};


// --- Control Task Function ---
static void control_task(void* arg) {
    if (!arg) { ESP_LOGE(CONTROL_TASK_TAG, "Control Task Args NULL!"); vTaskDelete(NULL); return; }

    ControlTaskArgs* taskArgs = static_cast<ControlTaskArgs*>(arg);
    RobotController& robotController = taskArgs->robotController;
    const int intervalMs = taskArgs->loopIntervalMs;

    TickType_t xFrequency = pdMS_TO_TICKS(intervalMs);
    if (xFrequency == 0) { ESP_LOGE(CONTROL_TASK_TAG, "Control Task Frequency is 0!"); vTaskDelete(NULL); return; }

    TickType_t xLastWakeTime = xTaskGetTickCount();
    int64_t lastWakeTimeMicros = esp_timer_get_time();
    ESP_LOGI(CONTROL_TASK_TAG, "Control task started on Core %d, Interval: %dms", xPortGetCoreID(), intervalMs);

    while (1) {
        int64_t startTimeMicros = esp_timer_get_time();
        float dt = (startTimeMicros - lastWakeTimeMicros) / 1000000.0f;
        lastWakeTimeMicros = startTimeMicros;

        const float nominal_dt = (float)intervalMs / 1000.0f;
        if (dt <= 0 || dt > (nominal_dt * 5.0f)) {
             ESP_LOGW(CONTROL_TASK_TAG, "Invalid dt (%.4f), using nominal dt (%.4f)", dt, nominal_dt);
             dt = nominal_dt;
        }

        robotController.runControlStep(dt); // Runs the main control logic

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// --- Main Application Entry Point ---
extern "C" void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_INFO);
    ESP_LOGI(TAG, "====== Balancing Robot Starting Up ======");

    // --- Initialize Core Systems ---
    EventBus& eventBus = EventBus::getInstance(); ESP_LOGI(TAG, "EventBus Accessed.");

    // Storage, Parser, ConfigService (Dependency for ComponentHandler)
    static SPIFFSStorageService storageService; ESP_ERROR_CHECK_WITHOUT_ABORT(storageService.init()); ESP_LOGI(TAG, "Storage Init.");
    static JsonConfigParser configParser; ESP_LOGI(TAG, "Parser Init.");
    static ConfigurationService configService(storageService, configParser, eventBus); ESP_ERROR_CHECK_WITHOUT_ABORT(configService.init()); ESP_LOGI(TAG, "ConfigService Init.");

    // ComponentHandler creates and initializes all internal components
    static ComponentHandler componentHandler(configService, eventBus);

    // Create StateManager AFTER ComponentHandler, passing IMUService
    static StateManager stateManager(eventBus, componentHandler.getIMUService()); ESP_LOGI(TAG, "StateManager Created.");

    // Now initialize ComponentHandler, passing StateManager
    if (componentHandler.init(stateManager) != ESP_OK) { // Pass stateManager here
        ESP_LOGE(TAG, "ComponentHandler Init Failed. Halting.");
        stateManager.setState(SystemState::FATAL_ERROR); // Set fatal error state
        return; // Stop execution
    }
    ESP_LOGI(TAG, "ComponentHandler Initialized.");

    // Initialize StateManager subscriptions *after* ComponentHandler has created everything
     if (stateManager.init() != ESP_OK) {
         ESP_LOGE(TAG, "StateManager Subscription Init Failed. Halting.");
         stateManager.setState(SystemState::FATAL_ERROR);
         return;
     }
     ESP_LOGI(TAG, "StateManager Initialized (Subscriptions Active).");


    // --- Create RobotController Facade ---
    // Pass components obtained from the handler
    static RobotController robotController(
        componentHandler.getOrientationEstimator(),
        componentHandler.getEncoderService(),
        componentHandler.getMotorService(),
        componentHandler.getBalancingAlgorithm(),
        stateManager, // Pass StateManager directly
        componentHandler.getFallDetector(),
        componentHandler.getWebServer(),
        componentHandler.getBatteryService(),
        componentHandler.getCommandProcessor()
    );
    ESP_LOGI(TAG, "RobotController Facade Created.");

    // <<< ADDED: Initialize RobotController subscriptions >>>
    if (robotController.init(eventBus) != ESP_OK) {
        ESP_LOGE(TAG, "RobotController Subscription Init Failed. Halting.");
        stateManager.setState(SystemState::FATAL_ERROR);
        return;
    }
    ESP_LOGI(TAG, "RobotController Initialized (Subscriptions Active).");
    // <<< END ADDED >>>


    // --- Setup and Start Control Task ---
    int intervalMs = configService.getMainLoopConfig().interval_ms;
    if (intervalMs <= 0 || intervalMs > 100) {
        ESP_LOGW(TAG, "Invalid loop interval %dms from config, defaulting to 10ms", intervalMs);
        intervalMs = 10;
    }
    ESP_LOGI(TAG, "Control Loop Interval: %d ms", intervalMs);

    const uint32_t controlTaskStackSize = 4096;
    const UBaseType_t controlTaskPriority = configMAX_PRIORITIES - 1;
    const BaseType_t controlTaskCoreID = 1; // Run control on Core 1

    static ControlTaskArgs controlArgs = { robotController, intervalMs }; // Simplified init

    ESP_LOGI(TAG, "Creating Control Task...");
    BaseType_t controlTaskCreated = xTaskCreatePinnedToCore(
        control_task,
        CONTROL_TASK_TAG,
        controlTaskStackSize,
        &controlArgs,
        controlTaskPriority,
        NULL, // No task handle needed here
        controlTaskCoreID
    );

    if (controlTaskCreated != pdPASS) {
        ESP_LOGE(TAG, "Failed Create Control Task! Halting.");
        stateManager.setState(SystemState::FATAL_ERROR);
        return; // Stop execution
    }
    ESP_LOGI(TAG, "Control Task created successfully on Core %d.", controlTaskCoreID);

    // Set initial state after everything is ready
    stateManager.setState(SystemState::IDLE);
    ESP_LOGI(TAG, "System State set to IDLE.");

    ESP_LOGI(TAG, "======= Init complete. Main loop yielding. =======");
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(60000)); // Main task can sleep
    }
}