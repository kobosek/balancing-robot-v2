#pragma once

#include "Task.hpp"
#include <memory>

// Forward declarations
class IMUService;

class IMUFifoTask : public Task {
public:

    explicit IMUFifoTask(IMUService& imuService);

    ~IMUFifoTask() override = default;

protected:
    void run() override;

private:
    IMUService& m_imuService;
};

class IMUWatchdogTask : public Task {
public:
    explicit IMUWatchdogTask(IMUService& imuService);
    ~IMUWatchdogTask() override = default;

protected:
    void run() override;

private:
    IMUService& m_imuService;
};
