#include "BatteryTask.hpp"
#include "BatteryService.hpp"
#include "esp_log.h"

static const char* TASK_TAG = "BattMonTask";

BatteryMonitorTask::BatteryMonitorTask(BatteryService& batteryService)
    : Task(TASK_TAG), m_batteryService(batteryService) {
}

void BatteryMonitorTask::run() {
    ESP_LOGI(TASK_TAG, "Battery Monitor Task Started on Core %d", xPortGetCoreID());
    
    while (true) {
        m_batteryService.updateBatteryStatus();
        
        // Use the configured interval from BatteryService
        constexpr TickType_t delay_ticks = pdMS_TO_TICKS(5000); // 5 seconds, matching BatteryService.hpp READ_INTERVAL_MS
        vTaskDelay(delay_ticks);
    }
}
