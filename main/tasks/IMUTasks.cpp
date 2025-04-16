#include "IMUTasks.hpp"
#include "IMUService.hpp"
#include "IMUHealthMonitor.hpp" // Include the new monitor header
#include "esp_log.h"
#include "esp_timer.h"
#include <cmath> // Keep includes if needed by other parts

// Tags for logging
static const char* FIFO_TASK_TAG = "IMUFifoTask";
static const char* HEALTH_MONITOR_TASK_TAG = "IMUHealthTask"; // Renamed tag

// --- IMUFifoTask implementation ---
IMUFifoTask::IMUFifoTask(IMUService& imuService)
    : Task(FIFO_TASK_TAG), m_imuService(imuService) {}

void IMUFifoTask::run() {
    ESP_LOGI(FIFO_TASK_TAG, "IMU FIFO task started on Core %d", xPortGetCoreID());

    // Task delegates to IMUService's data pipeline method
    while (true) {
        m_imuService.processDataPipeline();

        // Short delay to prevent CPU hogging, adjust as needed
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// --- IMUHealthMonitorTask implementation (Formerly IMUWatchdogTask) ---
IMUHealthMonitorTask::IMUHealthMonitorTask(IMUHealthMonitor& healthMonitor) // Takes the monitor now
    : Task(HEALTH_MONITOR_TASK_TAG), m_healthMonitor(healthMonitor) {} // Pass monitor

void IMUHealthMonitorTask::run() {
    ESP_LOGI(HEALTH_MONITOR_TASK_TAG, "IMU Health Monitor task started on Core %d", xPortGetCoreID());

    // Use the interval defined within the monitor or a default here
    const TickType_t xFrequency = pdMS_TO_TICKS(50);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (true) {
        // Delegate health check to the monitor component
        m_healthMonitor.checkHealth();

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}