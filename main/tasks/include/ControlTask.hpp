#pragma once

#include "Task.hpp"

// Forward declarations
class RobotController;

class ControlTask : public Task {
public:

    ControlTask(RobotController& robotController, int intervalMs);

    ~ControlTask() override = default;

protected:
    void run() override;

private:
    RobotController& m_robotController;
    int m_intervalMs;
};
