#include "ControlTask.hpp"
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

    TickType_t xLastWakeTime = xTaskGetTickCount();
    int64_t lastWakeTimeMicros = esp_timer_get_time();

    while (true) {
        int64_t startTimeMicros = esp_timer_get_time();
        float dt = (startTimeMicros - lastWakeTimeMicros) / 1000000.0f;
        lastWakeTimeMicros = startTimeMicros;

        const float nominal_dt = (float)m_intervalMs / 1000.0f;
        if (dt <= 0 || dt > (nominal_dt * 5.0f)) {
             ESP_LOGW(TASK_TAG, "Invalid dt (%.4f), using nominal dt (%.4f)", dt, nominal_dt);
             dt = nominal_dt;
        }

        m_robotController.runControlStep(dt); // Runs the main control logic

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
