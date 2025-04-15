#pragma once

#include "Task.hpp"

// Forward declarations
class BatteryService;

class BatteryMonitorTask : public Task {
public:

    explicit BatteryMonitorTask(BatteryService& batteryService);

    ~BatteryMonitorTask() override = default;

protected:

    void run() override;

private:
    BatteryService& m_batteryService;
};
