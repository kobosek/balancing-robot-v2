#include "HealthMonitorTask.hpp"
#include "IMUHealthMonitor.hpp"
#include "esp_log.h"
#include "esp_timer.h"

// Tags for logging

static const char* TAG = "IMUHealthTask";

// Constructor without taskCore parameter
HealthMonitorTask::HealthMonitorTask(IMUHealthMonitor& healthMonitor)
    : Task(TAG), m_healthMonitor(healthMonitor) {} // Assuming Task(name, stack, prio)

void HealthMonitorTask::run() {
    ESP_LOGI(TAG, "IMU Health Monitor task started (Core Affinity set by Start method), Priority %d", 
            uxTaskPriorityGet(nullptr));

    const TickType_t xFrequency = pdMS_TO_TICKS(50); 
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (true) {
        m_healthMonitor.checkHealth();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
