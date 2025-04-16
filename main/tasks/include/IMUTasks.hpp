#pragma once

#include "Task.hpp"
#include <memory>

// Forward declarations
class IMUService;
class IMUHealthMonitor; // Need the new monitor class

// Task to process FIFO data via IMUService
class IMUFifoTask : public Task {
public:
    explicit IMUFifoTask(IMUService& imuService);
    ~IMUFifoTask() override = default;

protected:
    void run() override;

private:
    IMUService& m_imuService;
};

// Task to periodically check IMU health via IMUHealthMonitor
class IMUHealthMonitorTask : public Task { // Renamed from IMUWatchdogTask
public:
    explicit IMUHealthMonitorTask(IMUHealthMonitor& healthMonitor);
    ~IMUHealthMonitorTask() override = default;

protected:
    void run() override;

private:
    IMUHealthMonitor& m_healthMonitor;
};