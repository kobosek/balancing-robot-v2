#include "ControlEventDispatcher.hpp"

#include "EventBus.hpp"
#include "IMU_OrientationData.hpp"
#include "TELEMETRY_Snapshot.hpp"
#include "esp_log.h"

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

bool ControlEventDispatcher::enqueueOrientation(float pitch_rad, float pitch_rate_rad) {
    DispatchItem item = {};
    item.type = ItemType::Orientation;
    item.pitch_rad = pitch_rad;
    item.pitch_rate_rad = pitch_rate_rad;
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

    DispatchItem item = {};
    while (true) {
        if (xQueueReceive(m_queue, &item, pdMS_TO_TICKS(20)) != pdTRUE) {
            continue;
        }

        switch (item.type) {
            case ItemType::Orientation: {
                IMU_OrientationData event(item.pitch_rad, item.pitch_rate_rad);
                m_eventBus.publish(event);
                break;
            }
            case ItemType::Telemetry: {
                TELEMETRY_Snapshot event(item.telemetry);
                m_eventBus.publish(event);
                break;
            }
            default:
                break;
        }
    }
}
