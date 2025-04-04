// main.cpp

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_pthread.h"

#include "include/ComponentHandler.hpp"
#include "include/RuntimeConfig.hpp"
#include "include/BalancingController.hpp" 

#include "interfaces/IWebServer.hpp"

#define TAG "AppMain" // Change main tag
#define CONTROL_TASK_TAG "ControlTask"
#define TELEMETRY_TAG "TelemetryHandler"

#define TELEMETRY_QUEUE_LENGTH 10
static QueueHandle_t telemetryQueue = NULL;

struct TelemetryTaskArgs {
    QueueHandle_t queue;
    IWebServer& webServer;
};

static void telemetry_handler_task(void* arg) {
    if (arg == nullptr) {
        ESP_LOGE(TELEMETRY_TAG, "Task arguments are NULL! Deleting task.");
        vTaskDelete(NULL);
        return;
    }
    TelemetryTaskArgs* taskArgs = static_cast<TelemetryTaskArgs*>(arg);
    QueueHandle_t queue = taskArgs->queue;
    IWebServer& webServer = taskArgs->webServer;

    ESP_LOGI(TELEMETRY_TAG, "Telemetry Handler Task started.");
    TelemetryDataPoint receivedData;
    while (1) {
        if (xQueueReceive(queue, &receivedData, portMAX_DELAY) == pdPASS) {
            ESP_LOGV(TELEMETRY_TAG, "Received telemetry point, adding to buffer.");
            webServer.addTelemetryData(receivedData);
        } else {
            ESP_LOGE(TELEMETRY_TAG, "xQueueReceive failed unexpectedly.");
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}


// --- Control Task Function (Modified) ---
static void control_task(void* arg) {
    BalancingController* controller = static_cast<BalancingController*>(arg);
    if (!controller) {
        ESP_LOGE(CONTROL_TASK_TAG, "Controller instance is NULL! Deleting task.");
        vTaskDelete(NULL);
        return;
    }

    struct ControlTaskArgs {
        BalancingController* controller;
        int loopIntervalMs;
    };

    ControlTaskArgs* taskArgs = static_cast<ControlTaskArgs*>(arg);
    controller = taskArgs->controller; // Get controller pointer
    const int intervalMs = taskArgs->loopIntervalMs; // Get interval

    if (!controller) { /* Re-check after cast */ }

    TickType_t xFrequency = pdMS_TO_TICKS(intervalMs);
    if (xFrequency == 0) {
         ESP_LOGE(CONTROL_TASK_TAG, "Calculated task frequency is 0 ticks! Interval: %d ms. Halting.", intervalMs);
         vTaskDelete(NULL); return;
    }

    TickType_t xLastWakeTime = xTaskGetTickCount();
    ESP_LOGI(CONTROL_TASK_TAG, "Control task started. Loop Interval: %d ms, Frequency: %lu ticks", intervalMs, (unsigned long)xFrequency);

    bool safetyStopActive = false; // Safety stop logic local to the task runner
    TickType_t safetyStopTime = 0;
    const int MAX_CONSECUTIVE_ERRORS = 100; // Define error threshold locally if needed
    int consecutiveErrorCounter = 0; // Local error counter if needed

    while (1) {
        controller->runCycle();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}


extern "C" void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_INFO);
    ESP_LOGI(TAG, "Starting Balancing Robot Initialization");

    // Initialize Config
    RuntimeConfig config;
    if (config.init("/spiffs/config.json") != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize RuntimeConfig. Halting."); return;
    }
    ESP_LOGI(TAG, "RuntimeConfig Initialized.");

    // Initialize Components
    ComponentHandler handler(config);
    if (handler.init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ComponentHandler. Halting."); return;
    }
    ESP_LOGI(TAG, "ComponentHandler Initialized.");

    // Get Loop Interval
    int intervalMs = config.getMainLoopIntervalMs();
    if (intervalMs <= 0) {
        ESP_LOGW(TAG, "Invalid main_loop_interval_ms (%d). Using 5ms.", intervalMs);
        intervalMs = 5;
    }

    ESP_LOGI(TAG, "Creating Telemetry Queue (Length: %d)", TELEMETRY_QUEUE_LENGTH);
    telemetryQueue = xQueueCreate(TELEMETRY_QUEUE_LENGTH, sizeof(TelemetryDataPoint));
    if (!telemetryQueue) {
        ESP_LOGE(TAG, "Failed to create Telemetry Queue! Halting."); return;
    }
    ESP_LOGI(TAG, "Telemetry Queue created successfully.");

    static BalancingController controllerInstance(
        handler.getMPU6050Manager(),
        handler.getMotorLeft(),
        handler.getMotorRight(),
        handler.getEncoderLeft(),
        handler.getEncoderRight(),
        handler.getAnglePIDController(),
        handler.getSpeedPIDControllerLeft(),
        handler.getSpeedPIDControllerRight(),
        telemetryQueue, 
        intervalMs
    );
    ESP_LOGI(TAG, "BalancingController instance created.");

    const uint32_t controlTaskStackSize = 4096;
    const UBaseType_t controlTaskPriority = configMAX_PRIORITIES - 1;
    const BaseType_t controlTaskCoreID = 1;

    struct ControlTaskArgs { 
        BalancingController* controller;
        int loopIntervalMs;
    };
    static ControlTaskArgs controlArgs = { 
        .controller = &controllerInstance,
        .loopIntervalMs = intervalMs
    };
    // -----------------------------------------

    ESP_LOGI(TAG, "Creating Control Task (Stack: %u words, Priority: %u, Core: %d)",
             (unsigned int)controlTaskStackSize, (unsigned int)controlTaskPriority, (int)controlTaskCoreID);
    BaseType_t controlTaskCreated = xTaskCreatePinnedToCore(
        control_task, CONTROL_TASK_TAG, controlTaskStackSize,
        &controlArgs, // Pass address of the args struct
        controlTaskPriority, NULL, controlTaskCoreID
    );
    if (controlTaskCreated != pdPASS) {
        ESP_LOGE(TAG, "Failed to create Control Task! Halting."); return;
    }
    ESP_LOGI(TAG, "Control Task created successfully.");


    const uint32_t telemetryTaskStackSize = 2048;
    const UBaseType_t telemetryTaskPriority = tskIDLE_PRIORITY + 2;
    const BaseType_t telemetryTaskCoreID = tskNO_AFFINITY;
    static TelemetryTaskArgs telemetryArgs = {
        .queue = telemetryQueue,
        .webServer = handler.getWebServer()
    };
    ESP_LOGI(TAG, "Creating Telemetry Handler Task (Stack: %u words, Priority: %u, Core: %s)",
             (unsigned int)telemetryTaskStackSize, (unsigned int)telemetryTaskPriority,
             (telemetryTaskCoreID == tskNO_AFFINITY) ? "Any" : "Pinned");
    BaseType_t telemetryTaskCreated = xTaskCreatePinnedToCore(
        telemetry_handler_task, TELEMETRY_TAG, telemetryTaskStackSize,
        &telemetryArgs, telemetryTaskPriority, NULL, telemetryTaskCoreID
    );
    if ( telemetryTaskCreated != pdPASS) {
        ESP_LOGE(TAG, "Failed to create Telemetry Handler Task! Telemetry may not work.");
    } else {
        ESP_LOGI(TAG, "Telemetry Handler Task created successfully.");
    }

    ESP_LOGI(TAG, "Initialization complete. Main task entering idle state.");
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

