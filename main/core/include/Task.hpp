#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <mutex>
#include <string>

class Task {
public:
    virtual ~Task() {
        stop();
    }

    virtual bool start(UBaseType_t priority, BaseType_t coreId = -1, uint32_t stackSize = 4096) {
        std::lock_guard<std::mutex> lock(m_taskMutex);
        if (m_taskHandle != nullptr) {
            ESP_LOGW(m_taskName, "Task already running");
            return false;
        }

        BaseType_t result;
        if (coreId >= 0) {
            result = xTaskCreatePinnedToCore(
                taskFunction,
                m_taskName,
                stackSize,
                this,
                priority,
                &m_taskHandle,
                coreId
            );
        } else {
            result = xTaskCreate(
                taskFunction,
                m_taskName,
                stackSize,
                this,
                priority,
                &m_taskHandle
            );
        }

        if (result != pdPASS) {
            ESP_LOGE(m_taskName, "Failed to create task");
            return false;
        }

        ESP_LOGI(m_taskName, "Task started successfully");
        return true;
    }

    virtual void stop() {
        TaskHandle_t taskHandle = nullptr;
        {
            std::lock_guard<std::mutex> lock(m_taskMutex);
            taskHandle = m_taskHandle;
            m_taskHandle = nullptr;
        }

        if (taskHandle != nullptr) {
            ESP_LOGI(m_taskName, "Stopping task");
            vTaskDelete(taskHandle);
        }
    }

    bool isRunning() const {
        std::lock_guard<std::mutex> lock(m_taskMutex);
        return m_taskHandle != nullptr;
    }

protected:
    explicit Task(const char* taskName) : m_taskName(taskName), m_taskHandle(nullptr) {}

    virtual void run() = 0;

private:

    static void taskFunction(void* arg) {
        Task* task = static_cast<Task*>(arg);
        task->run();
        
        // If we get here, the task has exited its run method
        // Mark it as not running anymore
        {
            std::lock_guard<std::mutex> lock(task->m_taskMutex);
            task->m_taskHandle = nullptr;
        }
        vTaskDelete(NULL);
    }

    const char* m_taskName;
    TaskHandle_t m_taskHandle;
    mutable std::mutex m_taskMutex;
};
