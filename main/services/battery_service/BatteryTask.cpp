#include "BatteryTask.hpp"
#include "BatteryService.hpp"
#include "esp_log.h"

static const char* TASK_TAG = "BattMonTask";

// Constructor takes service and interval
BatteryMonitorTask::BatteryMonitorTask(BatteryService& batteryService, int intervalMs)
    : Task(TASK_TAG),
      m_batteryService(batteryService),
      m_intervalTicks(pdMS_TO_TICKS(intervalMs > 0 ? intervalMs : 5000)) // Use provided interval, default 5s
{
    if (m_intervalTicks == 0) { // Ensure ticks is not zero
        ESP_LOGW(TASK_TAG, "Calculated interval ticks is 0, using 1 tick instead.");
        m_intervalTicks = 1;
    }
}

void BatteryMonitorTask::run() {
    ESP_LOGI(TASK_TAG, "Battery Monitor Task Started on Core %d, Interval Ticks: %lu",
             xPortGetCoreID(), m_intervalTicks);

    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (true) {
        m_batteryService.updateBatteryStatus();

        // Delay until next execution time
        vTaskDelayUntil(&xLastWakeTime, m_intervalTicks);
    }
}