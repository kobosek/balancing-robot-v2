#include "ControlTask.hpp"
#include "ControlLoopTiming.hpp"
#include "RobotController.hpp"
#include "esp_log.h"
#include "esp_timer.h"

static const char* TASK_TAG = "ControlTask";

ControlTask::ControlTask(RobotController& robotController, int intervalMs)
    : Task(TASK_TAG), m_robotController(robotController), m_intervalMs(intervalMs) {
}

void ControlTask::run() {
    ESP_LOGI(TASK_TAG, "Control task started on Core %d, Interval: %dms", 
             xPortGetCoreID(), m_intervalMs);

    TickType_t xFrequency = pdMS_TO_TICKS(m_intervalMs);
    if (xFrequency == 0) { 
        ESP_LOGE(TASK_TAG, "Control Task Frequency is 0!"); 
        return; 
    }

    ControlLoopTiming timing(xTaskGetTickCount(), esp_timer_get_time());
    while (true) {
        auto wakeTick = timing.wakeTick();
        // Wait before the first step too. Reanchor after every actual wake so a
        // delayed task does not run PID repeatedly to catch up with old deadlines.
        if (xTaskDelayUntil(&wakeTick, xFrequency) == pdFALSE) vTaskDelay(1);
        const float dt = timing.beginStep(xTaskGetTickCount(), esp_timer_get_time());
        // Never replace a long measured interval with a fictitious nominal one.
        // RobotController retains input freshness and final motor-commit checks.
        m_robotController.runControlStep(dt);
    }
}
