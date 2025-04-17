#pragma once

#include "Task.hpp"

// Forward declarations
class BatteryService;

class BatteryMonitorTask : public Task {
public:
    // Constructor takes service and interval
    explicit BatteryMonitorTask(BatteryService& batteryService, int intervalMs);
    ~BatteryMonitorTask() override = default;

protected:
    void run() override;

private:
    BatteryService& m_batteryService;
    TickType_t m_intervalTicks; // Store interval in ticks
};