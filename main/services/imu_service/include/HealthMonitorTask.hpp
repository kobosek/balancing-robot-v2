#pragma once
#include "Task.hpp"

class IMUHealthMonitor;
class IMUService;

// Task to periodically check IMU health via IMUHealthMonitor
class HealthMonitorTask : public Task {
public:
    // Constructor without taskCore parameter
    HealthMonitorTask(IMUHealthMonitor& healthMonitor, IMUService& imuService);
    ~HealthMonitorTask() override = default;

protected:
    void run() override;

private:
    IMUHealthMonitor& m_healthMonitor;
    IMUService& m_imuService;
};
