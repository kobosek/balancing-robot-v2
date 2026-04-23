#include "FIFOTask.hpp"
#include "FIFOProcessor.hpp"
#include "esp_log.h"

FIFOTask::FIFOTask(FIFOProcessor& fifoProcessor)
    : Task(TAG), m_fifoProcessor(fifoProcessor) {} // Assuming Task(name, stack, prio)

void FIFOTask::run() {
    // xPortGetCoreID() will report the core this task *ends up running on* after Start() is called.
    ESP_LOGI(TAG, "IMU FIFO task started (Core Affinity set by Start method), Priority %d", 
            uxTaskPriorityGet(nullptr));

    // Get the semaphore handle from the FIFOProcessor
    SemaphoreHandle_t dataReadySemaphore = m_fifoProcessor.getDataReadySemaphore();
    const TickType_t pollInterval = pdMS_TO_TICKS(5);
    if (dataReadySemaphore == NULL) {
        ESP_LOGE(TAG, "Data ready semaphore is NULL, falling back to timed FIFO polling");
    }

    while (true) {
        if (dataReadySemaphore != NULL) {
            const BaseType_t waitResult = xSemaphoreTake(dataReadySemaphore, pollInterval);
            if (waitResult != pdTRUE) {
                continue;
            }
        } else {
            vTaskDelay(pollInterval);
        }

        m_fifoProcessor.processFIFO();
    }
}   
