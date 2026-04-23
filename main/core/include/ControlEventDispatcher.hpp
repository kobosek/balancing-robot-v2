#pragma once

#include "Task.hpp"
#include "TelemetryDataPoint.hpp"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <atomic>

class EventBus;

class ControlEventDispatcher : public Task {
public:
    explicit ControlEventDispatcher(EventBus& eventBus, UBaseType_t queueDepth = 64);
    ~ControlEventDispatcher() override;

    esp_err_t init();

    bool enqueueOrientation(float pitch_rad, float pitch_rate_rad);
    bool enqueueTelemetry(const TelemetryDataPoint& snapshot);

    uint32_t getDroppedEventCount() const {
        return m_droppedEvents.load(std::memory_order_relaxed);
    }

protected:
    void run() override;

private:
    enum class ItemType : uint8_t {
        Orientation,
        Telemetry
    };

    struct DispatchItem {
        ItemType type = ItemType::Telemetry;
        float pitch_rad = 0.0f;
        float pitch_rate_rad = 0.0f;
        TelemetryDataPoint telemetry = {};
    };

    bool enqueueItem(const DispatchItem& item);

    static constexpr const char* TAG = "CtrlEventDisp";

    EventBus& m_eventBus;
    UBaseType_t m_queueDepth;
    QueueHandle_t m_queue = nullptr;
    std::atomic<uint32_t> m_droppedEvents{0};
};
