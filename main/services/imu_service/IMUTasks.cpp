#include "IMUTasks.hpp"
#include "IMUService.hpp"
#include "IMUHealthMonitor.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "IMUCalibrationService.hpp"
#include "EventBus.hpp"
#include "EventTypes.hpp"
#include "BaseEvent.hpp"
#include "IMU_CalibrationRequest.hpp" 
#include "freertos/semphr.h" 

// Tags for logging
static const char* FIFO_TASK_TAG = "IMUFifoTask";
static const char* HEALTH_MONITOR_TASK_TAG = "IMUHealthTask";
static const char* CALIB_TASK_TAG = "IMUCalibTask";


// Constructor without taskCore parameter
IMUFifoTask::IMUFifoTask(IMUService& imuService)
    : Task(FIFO_TASK_TAG), m_imuService(imuService) {} // Assuming Task(name, stack, prio)

void IMUFifoTask::run() {
    // xPortGetCoreID() will report the core this task *ends up running on* after Start() is called.
    ESP_LOGI(FIFO_TASK_TAG, "IMU FIFO task started (Core Affinity set by Start method), Priority %d", 
            uxTaskPriorityGet(nullptr));

    const TickType_t xFrequency = pdMS_TO_TICKS(2); 
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (true) {
        m_imuService.processDataPipeline();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Constructor without taskCore parameter
IMUHealthMonitorTask::IMUHealthMonitorTask(IMUHealthMonitor& healthMonitor)
    : Task(HEALTH_MONITOR_TASK_TAG), m_healthMonitor(healthMonitor) {} // Assuming Task(name, stack, prio)

void IMUHealthMonitorTask::run() {
    ESP_LOGI(HEALTH_MONITOR_TASK_TAG, "IMU Health Monitor task started (Core Affinity set by Start method), Priority %d", 
            uxTaskPriorityGet(nullptr));

    const TickType_t xFrequency = pdMS_TO_TICKS(50); 
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (true) {
        m_healthMonitor.checkHealth();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Constructor without taskCore parameter
IMUCalibrationTask::IMUCalibrationTask(IMUCalibrationService& calibrationService, EventBus& bus)
    : Task(CALIB_TASK_TAG), m_calibrationService(calibrationService) // Assuming Task(name, stack, prio)
{
    m_calibSemaphore = xSemaphoreCreateBinary();
    if (!m_calibSemaphore) {
        ESP_LOGE(CALIB_TASK_TAG, "Failed to create calibration semaphore!");
    }
    ESP_LOGI(CALIB_TASK_TAG, "Calibration task created. Subscribing to IMU_CALIBRATION_REQUEST.");
    bus.subscribe(EventType::IMU_CALIBRATION_REQUEST, [this](const BaseEvent& ev) {
        if (ev.type == EventType::IMU_CALIBRATION_REQUEST) {
            ESP_LOGI(CALIB_TASK_TAG, "IMU_CALIBRATION_REQUEST event received, giving semaphore.");
            if (m_calibSemaphore) {
                xSemaphoreGive(m_calibSemaphore);
            }
        }
    });
}

IMUCalibrationTask::~IMUCalibrationTask() { 
    if (m_calibSemaphore) {
        vSemaphoreDelete(m_calibSemaphore);
        m_calibSemaphore = nullptr;
    }
}


void IMUCalibrationTask::run() {
    ESP_LOGI(CALIB_TASK_TAG, "IMU Calibration task started (Core Affinity set by Start method), Priority %d", 
            uxTaskPriorityGet(nullptr));
    while (true) {
        if (m_calibSemaphore && xSemaphoreTake(m_calibSemaphore, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(CALIB_TASK_TAG, "Calibration semaphore taken, starting calibration process...");
            esp_err_t ret = m_calibrationService.calibrate();
            if (ret != ESP_OK) {
                ESP_LOGE(CALIB_TASK_TAG, "Calibration process in service reported failure: %s", esp_err_to_name(ret));
            } else {
                ESP_LOGI(CALIB_TASK_TAG, "Calibration process in service reported success.");
            }
        }
    }
}