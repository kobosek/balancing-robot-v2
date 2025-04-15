#include "IMUTasks.hpp"
#include "IMUService.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include <cmath>

// Constants for FIFO data structure
// These were previously defined in IMUService::fifo_task
constexpr size_t FIFO_PACKET_SIZE = 12; // 6 bytes accel + 6 bytes gyro
constexpr size_t ACCEL_X_OFFSET = 0;
constexpr size_t ACCEL_Y_OFFSET = 2;
constexpr size_t ACCEL_Z_OFFSET = 4;
constexpr size_t GYRO_X_OFFSET = 6;
constexpr size_t GYRO_Y_OFFSET = 8;
constexpr size_t GYRO_Z_OFFSET = 10;

// Tags for logging
static const char* FIFO_TASK_TAG = "IMUFifoTask";
static const char* WATCHDOG_TASK_TAG = "IMUWatchdogTask";

// IMUFifoTask implementation
IMUFifoTask::IMUFifoTask(IMUService& imuService)
    : Task(FIFO_TASK_TAG), m_imuService(imuService) {
}

void IMUFifoTask::run() {
    ESP_LOGI(FIFO_TASK_TAG, "IMU FIFO task started on Core %d", xPortGetCoreID());
    
    // This task will delegate to IMUService's processFifoData method
    // which will contain the logic previously in IMUService::fifo_task
    while (true) {
        m_imuService.processFifoData();
        
        // Short delay to prevent CPU hogging
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// IMUWatchdogTask implementation
IMUWatchdogTask::IMUWatchdogTask(IMUService& imuService)
    : Task(WATCHDOG_TASK_TAG), m_imuService(imuService) {
}

void IMUWatchdogTask::run() {
    ESP_LOGI(WATCHDOG_TASK_TAG, "IMU Watchdog task started on Core %d", xPortGetCoreID());
    
    // This task will delegate to IMUService's checkImuHealth method
    // which will contain the logic previously in IMUService::watchdog_task
    const TickType_t xFrequency = pdMS_TO_TICKS(1000); // Check every second
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    while (true) {
        m_imuService.checkImuHealth();
        
        // Use vTaskDelayUntil for consistent timing
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
