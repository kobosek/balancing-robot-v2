#include "FIFOTask.hpp"
#include "FIFOProcessor.hpp"
#include "esp_log.h"
#include "esp_timer.h"

static const char* TAG = "IMUFifoTask";

FIFOTask::FIFOTask(FIFOProcessor& fifoProcessor)
    : Task(TAG), m_fifoProcessor(fifoProcessor) {} // Assuming Task(name, stack, prio)

void FIFOTask::run() {
    // xPortGetCoreID() will report the core this task *ends up running on* after Start() is called.
    ESP_LOGI(TAG, "IMU FIFO task started (Core Affinity set by Start method), Priority %d", 
            uxTaskPriorityGet(nullptr));

    // Get the semaphore handle from the FIFOProcessor
    SemaphoreHandle_t dataReadySemaphore = m_fifoProcessor.getDataReadySemaphore();
    if (dataReadySemaphore == NULL) {
        ESP_LOGE(TAG, "Data ready semaphore is NULL, task will not process fifo");
    }

    while (true) {
        if (dataReadySemaphore != NULL) {
            // Wait for notification from ISR (with timeout as a safety measure)
            BaseType_t waitResult = xSemaphoreTake(dataReadySemaphore, portMAX_DELAY);
            
            if (waitResult == pdTRUE) {
                m_fifoProcessor.processFIFO();
            } else {
                ESP_LOGV(TAG, "Semaphore wait timeout, NOT checking FIFO");
            }
        }
    }
}   