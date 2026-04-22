#include "HealthMonitorTask.hpp"
#include "IMUHealthMonitor.hpp"
#include "IMUService.hpp"
#include "esp_log.h"

// Tags for logging

static const char* TAG = "IMUHealthTask";

// Constructor without taskCore parameter
HealthMonitorTask::HealthMonitorTask(IMUHealthMonitor& healthMonitor, IMUService& imuService)
    : Task(TAG), m_healthMonitor(healthMonitor), m_imuService(imuService) {}

void HealthMonitorTask::run() {
    ESP_LOGI(TAG, "IMU Health Monitor task started (Core Affinity set by Start method), Priority %d", 
            uxTaskPriorityGet(nullptr));

    const TickType_t xFrequency = pdMS_TO_TICKS(50); 
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (true) {
        m_healthMonitor.checkHealth();
        m_imuService.pollBackgroundMaintenance();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
