#pragma once

#include "Task.hpp" // Your base task class header
#include "freertos/semphr.h" // For SemaphoreHandle_t in IMUCalibrationTask

// Forward declarations
class IMUService;
class IMUHealthMonitor;
class IMUCalibrationService;
class EventBus;

// Task to process FIFO data via IMUService
class IMUFifoTask : public Task {
public:
    // Constructor without taskCore parameter
    explicit IMUFifoTask(IMUService& imuService);
    ~IMUFifoTask() override = default;

protected:
    void run() override;

private:
    IMUService& m_imuService;
};

// Task to periodically check IMU health via IMUHealthMonitor
class IMUHealthMonitorTask : public Task {
public:
    // Constructor without taskCore parameter
    explicit IMUHealthMonitorTask(IMUHealthMonitor& healthMonitor);
    ~IMUHealthMonitorTask() override = default;

protected:
    void run() override;

private:
    IMUHealthMonitor& m_healthMonitor;
};

// Task to handle asynchronous calibration requests
class IMUCalibrationTask : public Task {
public:
    // Constructor without taskCore parameter
    explicit IMUCalibrationTask(IMUCalibrationService& calibrationService, EventBus& bus);
    ~IMUCalibrationTask() override; // Explicit destructor for semaphore cleanup

protected:
    void run() override;

private:
    IMUCalibrationService& m_calibrationService;
    SemaphoreHandle_t m_calibSemaphore; // Semaphore handle for triggering calibration
};