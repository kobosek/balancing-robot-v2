#pragma once
#include "Task.hpp"

class IMUHealthMonitor;

// Task to periodically check IMU health via IMUHealthMonitor
class HealthMonitorTask : public Task {
public:
    // Constructor without taskCore parameter
    explicit HealthMonitorTask(IMUHealthMonitor& healthMonitor);
    ~HealthMonitorTask() override = default;

protected:
    void run() override;

private:
    IMUHealthMonitor& m_healthMonitor;
};
