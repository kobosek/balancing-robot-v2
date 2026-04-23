#pragma once

#include "ConfigData.hpp"
#include "cJSON.h"

namespace json_config_sections {

cJSON* serializeWiFi(const WiFiConfig& config);
bool deserializeWiFi(cJSON* obj, WiFiConfig& config);

cJSON* serializeMainLoop(const MainLoopConfig& config);
bool deserializeMainLoop(cJSON* obj, MainLoopConfig& config);

cJSON* serializeControl(const ControlConfig& config);
bool deserializeControl(cJSON* obj, ControlConfig& config);

cJSON* serializeImu(const MPU6050Config& config);
bool deserializeImu(cJSON* obj, MPU6050Config& config);

cJSON* serializeEncoder(const EncoderConfig& config);
bool deserializeEncoder(cJSON* obj, EncoderConfig& config);

cJSON* serializeMotor(const MotorConfig& config);
bool deserializeMotor(cJSON* obj, MotorConfig& config);

cJSON* serializeBattery(const BatteryConfig& config);
bool deserializeBattery(cJSON* obj, BatteryConfig& config);

cJSON* serializeBehavior(const SystemBehaviorConfig& config);
bool deserializeBehavior(cJSON* obj, SystemBehaviorConfig& config);

cJSON* serializeDimensions(const RobotDimensionsConfig& config);
bool deserializeDimensions(cJSON* obj, RobotDimensionsConfig& config);

cJSON* serializeWeb(const WebServerConfig& config);
bool deserializeWeb(cJSON* obj, WebServerConfig& config);

cJSON* serializePid(const PIDConfig& config);
bool deserializePid(cJSON* obj, PIDConfig& config);

cJSON* serializePidTuning(const PidTuningConfig& config);
bool deserializePidTuning(cJSON* obj, PidTuningConfig& config);

}
