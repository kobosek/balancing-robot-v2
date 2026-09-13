#include "ControlTask.hpp"
#include "ControlLoopTiming.hpp"
#include "RobotController.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include <algorithm>
#include <cstdint>

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
    int activeIntervalMs = m_intervalMs;
    int64_t metricsWindowStartUs = esp_timer_get_time();
    int64_t worstStepUs = 0;
    int64_t minDtUs = INT64_MAX;
    int64_t maxDtUs = 0;
    int64_t sumDtUs = 0;
    uint32_t stepCount = 0;
    uint32_t lateStepCount = 0;
    while (true) {
        const int configuredIntervalMs = m_robotController.controlIntervalMs();
        if (configuredIntervalMs != activeIntervalMs) {
            activeIntervalMs = configuredIntervalMs > 0 ? configuredIntervalMs : m_intervalMs;
            xFrequency = pdMS_TO_TICKS(activeIntervalMs);
            if (xFrequency == 0) xFrequency = 1;
            // A period change is a new schedule, not a backlog to replay.
            timing = ControlLoopTiming(xTaskGetTickCount(), esp_timer_get_time());
        }
        auto wakeTick = timing.wakeTick();
        // Wait before the first step too. Reanchor after every actual wake so a
        // delayed task does not run PID repeatedly to catch up with old deadlines.
        if (xTaskDelayUntil(&wakeTick, xFrequency) == pdFALSE) vTaskDelay(1);
        const float dt = timing.beginStep(xTaskGetTickCount(), esp_timer_get_time());
        const int64_t measuredDtUs = std::max<int64_t>(0,
            static_cast<int64_t>(dt * 1000000.0f));
        minDtUs = std::min(minDtUs, measuredDtUs);
        maxDtUs = std::max(maxDtUs, measuredDtUs);
        sumDtUs += measuredDtUs;
        ++stepCount;
        // Never replace a long measured interval with a fictitious nominal one.
        // RobotController retains input freshness and final motor-commit checks.
        const int64_t stepStartUs = esp_timer_get_time();
        m_robotController.runControlStep(dt);
        const int64_t stepCostUs = std::max<int64_t>(0,
            esp_timer_get_time() - stepStartUs);
        worstStepUs = std::max(worstStepUs, stepCostUs);
        if (stepCostUs > static_cast<int64_t>(activeIntervalMs) * 1000) {
            ++lateStepCount;
        }

        const int64_t metricsNowUs = esp_timer_get_time();
        if (metricsNowUs - metricsWindowStartUs >= 1000000) {
            const UBaseType_t highWaterWords = uxTaskGetStackHighWaterMark(nullptr);
            ESP_LOGI(TASK_TAG,
                     "metrics: steps=%lu dt_min=%lldus dt_max=%lldus dt_avg=%lldus worst_step=%lldus late_steps=%lu stack_high_water=%lu words",
                     static_cast<unsigned long>(stepCount),
                     static_cast<long long>(stepCount ? minDtUs : 0),
                     static_cast<long long>(maxDtUs),
                     static_cast<long long>(stepCount ? sumDtUs / stepCount : 0),
                     static_cast<long long>(worstStepUs),
                     static_cast<unsigned long>(lateStepCount),
                     static_cast<unsigned long>(highWaterWords));
            metricsWindowStartUs = metricsNowUs;
            minDtUs = INT64_MAX;
            maxDtUs = 0;
            sumDtUs = 0;
            stepCount = 0;
            worstStepUs = 0;
            lateStepCount = 0;
        }

        // A busy control step can consume the whole nominal period.  Give
        // lower-priority system/idle work a real scheduling window before the
        // next release; the next measured dt reflects this yield naturally.
        vTaskDelay(1);
    }
}
