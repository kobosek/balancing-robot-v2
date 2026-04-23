#include "ApplicationRuntime.hpp"

#include "ApplicationContext.hpp"
#include "BatteryTask.hpp"
#include "ControlEventDispatcher.hpp"
#include "ControlTask.hpp"
#include "IMUService.hpp"
#include "esp_check.h"
#include "esp_log.h"

ApplicationRuntime::ApplicationRuntime() = default;
ApplicationRuntime::~ApplicationRuntime() = default;

esp_err_t ApplicationRuntime::start(ApplicationContext& context, int controlIntervalMs, int batteryIntervalMs)
{
    ESP_LOGI(TAG, "Creating and starting application tasks");

    if (controlIntervalMs <= 0 || controlIntervalMs > 1000) {
        ESP_LOGW(TAG, "Invalid loop interval %dms provided, defaulting to 10ms", controlIntervalMs);
        controlIntervalMs = 10;
    }
    if (batteryIntervalMs <= 0 || batteryIntervalMs > 60000) {
        ESP_LOGW(TAG, "Invalid battery interval %dms provided, defaulting to 5000ms", batteryIntervalMs);
        batteryIntervalMs = 5000;
    }

    ESP_LOGI(TAG, "Control Loop Interval: %d ms, Battery Read Interval: %d ms", controlIntervalMs, batteryIntervalMs);

    m_controlTask = std::make_unique<ControlTask>(context.robotController(), controlIntervalMs);
    ESP_RETURN_ON_FALSE(m_controlTask != nullptr, ESP_ERR_NO_MEM, TAG, "Failed to allocate control task");

    m_batteryMonitorTask = std::make_unique<BatteryMonitorTask>(context.batteryService(), batteryIntervalMs);
    ESP_RETURN_ON_FALSE(m_batteryMonitorTask != nullptr, ESP_ERR_NO_MEM, TAG, "Failed to allocate battery monitor task");

    if (!context.controlEventDispatcher().start(6, 0, 4096)) {
        ESP_LOGE(TAG, "Failed to start Control Event Dispatcher task!");
        return ESP_FAIL;
    }

    if (!m_controlTask->start(configMAX_PRIORITIES - 1, 1, 4096)) {
        ESP_LOGE(TAG, "Failed to start Control task!");
        return ESP_FAIL;
    }

    if (!m_batteryMonitorTask->start(5, 0, 3072)) {
        ESP_LOGE(TAG, "Failed to start Battery Monitor task!");
        return ESP_FAIL;
    }

    if (!context.imuService().startTasks()) {
        ESP_LOGE(TAG, "Failed to start IMU tasks!");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "All application tasks started successfully");
    return ESP_OK;
}
