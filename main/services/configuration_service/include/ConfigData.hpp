#pragma once

#include "config/PIDConfig.hpp"
#include "config/WiFiConfig.hpp"
#include "config/MainLoopConfig.hpp"
#include "config/ControlConfig.hpp"
#include "config/MPU6050Config.hpp"
#include "config/EncoderConfig.hpp"
#include "config/MotorConfig.hpp"
#include "config/BatteryConfig.hpp"
#include "config/SystemBehaviorConfig.hpp"
#include "config/RobotDimensionsConfig.hpp"
#include "config/WebServerConfig.hpp"
#include "config/PidTuningConfig.hpp"
#include <cstdint>

struct ConfigData {
    int config_version = 3;
    // Monotonic revision of the complete persisted document. Strategy
    // revisions remain scoped to their typed records; this field protects a
    // full read/modify/write request from overwriting unrelated changes.
    uint32_t config_revision = 0;
    WiFiConfig wifi;
    MainLoopConfig mainLoop;
    ControlConfig control;
    MPU6050Config imu;
    EncoderConfig encoder;
    MotorConfig motor;
    BatteryConfig battery;
    PidTuningConfig pid_tuning;
    SystemBehaviorConfig behavior;
    RobotDimensionsConfig dimensions;
    WebServerConfig web;
};
