#pragma once

#include <memory>
#include "esp_err.h"

class ApplicationContext;
class BatteryMonitorTask;
class ControlTask;

class ApplicationRuntime {
public:
    ApplicationRuntime();
    ~ApplicationRuntime();

    esp_err_t start(ApplicationContext& context, int controlIntervalMs, int batteryIntervalMs);

private:
    static constexpr const char* TAG = "AppRuntime";

    std::unique_ptr<ControlTask> m_controlTask;
    std::unique_ptr<BatteryMonitorTask> m_batteryMonitorTask;
};
