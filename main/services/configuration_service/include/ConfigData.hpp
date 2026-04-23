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

struct ConfigData {
    int config_version = 1;
    WiFiConfig wifi;
    MainLoopConfig mainLoop;
    ControlConfig control;
    MPU6050Config imu;
    EncoderConfig encoder;
    MotorConfig motor;
    BatteryConfig battery;
    PIDConfig pid_angle;
    PIDConfig pid_speed_left;
    PIDConfig pid_speed_right;
    PIDConfig pid_yaw_rate;
    PidTuningConfig pid_tuning;
    SystemBehaviorConfig behavior;
    RobotDimensionsConfig dimensions;
    WebServerConfig web;
};
