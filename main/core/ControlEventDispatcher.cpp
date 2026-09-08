#include "ControlEventDispatcher.hpp"

#include "EventBus.hpp"
#include "IMU_OrientationData.hpp"
#include "TELEMETRY_Snapshot.hpp"
#include "esp_log.h"
#include <algorithm>

ControlEventDispatcher::ControlEventDispatcher(EventBus& eventBus, UBaseType_t queueDepth)
    : Task(TAG),
      m_eventBus(eventBus),
      m_queueDepth(queueDepth) {}

ControlEventDispatcher::~ControlEventDispatcher() {
    stop();
    if (m_queue != nullptr) {
        vQueueDelete(m_queue);
        m_queue = nullptr;
    }
}

esp_err_t ControlEventDispatcher::init() {
    if (m_queue != nullptr) {
        return ESP_OK;
    }

    m_queue = xQueueCreate(m_queueDepth, sizeof(DispatchItem));
    if (m_queue == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate dispatch queue");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Initialized control event queue depth=%u", static_cast<unsigned>(m_queueDepth));
    return ESP_OK;
}

bool ControlEventDispatcher::enqueueOrientation(const OrientationEstimate& estimate) {
    DispatchItem item = {};
    item.type = ItemType::Orientation;
    item.orientation = estimate;
    return enqueueItem(item);
}

bool ControlEventDispatcher::enqueueTelemetry(const TelemetryDataPoint& snapshot) {
    DispatchItem item = {};
    item.type = ItemType::Telemetry;
    item.telemetry = snapshot;
    return enqueueItem(item);
}

bool ControlEventDispatcher::enqueueItem(const DispatchItem& item) {
    if (m_queue == nullptr) {
        m_droppedEvents.fetch_add(1, std::memory_order_relaxed);
        return false;
    }

    if (const auto worker = m_worker.load()) xTaskNotifyGive(worker);
    if (xQueueSendToBack(m_queue, &item, 0) == pdTRUE) {
        return true;
    }

    DispatchItem discarded = {};
    (void)xQueueReceive(m_queue, &discarded, 0);
    m_droppedEvents.fetch_add(1, std::memory_order_relaxed);
    return xQueueSendToBack(m_queue, &item, 0) == pdTRUE;
}

void ControlEventDispatcher::run() {
    ESP_LOGI(TAG, "Control event dispatcher started on Core %d", xPortGetCoreID());

    m_worker = xTaskGetCurrentTaskHandle();
    DispatchItem item = {};
    while (true) {
        ImuControlFault fault;
        portENTER_CRITICAL(&m_faultMux);
        const bool pending = m_faultPending;
        if (pending) { fault = m_fault; m_faultPending = false; }
        portEXIT_CRITICAL(&m_faultMux);
        if (pending) {
            m_eventBus.publish(CONTROL_ImuDataInvalid(fault));
            ESP_LOGW(TAG, "Control inhibited: %s arm=%llu generation=%lu age=%lldus latest_age=%lldus error=%s",
                fault.cause, static_cast<unsigned long long>(fault.armId), static_cast<unsigned long>(fault.generation),
                fault.sampleTimestampUs > 0 ? fault.observedUs - fault.sampleTimestampUs : -1LL,
                fault.latestTimestampUs > 0 ? fault.observedUs - fault.latestTimestampUs : -1LL,
                esp_err_to_name(fault.error));
            continue;
        }
        if (xQueueReceive(m_queue, &item, 0) != pdTRUE) {
            ulTaskNotifyTake(pdTRUE, std::max<TickType_t>(1, pdMS_TO_TICKS(5)));
            continue;
        }
        if (item.type == ItemType::Orientation) m_eventBus.publish(IMU_OrientationData(item.orientation));
        else m_eventBus.publish(TELEMETRY_Snapshot(item.telemetry));
    }
}
void ControlEventDispatcher::latchImuFault(const ImuControlFault& fault) {
    portENTER_CRITICAL(&m_faultMux);
    if (!m_faultPending || fault.armId >= m_fault.armId) m_fault = fault;
    m_faultPending = true;
    portEXIT_CRITICAL(&m_faultMux);
    if (const auto worker = m_worker.load()) xTaskNotifyGive(worker);
}
