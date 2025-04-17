#pragma once

#include "ConfigData.hpp" // Include full definition here
#include "esp_err.h"
#include <string>
#include "cJSON.h"

// Forward declarations of specific structs within ConfigData
struct PIDConfig;
struct ControlConfig;
struct SystemBehaviorConfig;
struct RobotDimensionsConfig;
struct WebServerConfig;

class IConfigParser {
public:
    virtual ~IConfigParser() = default;
    virtual esp_err_t serialize(const ConfigData& config, std::string& output) const = 0;
    virtual esp_err_t deserialize(const std::string& input, ConfigData& configOutput) const = 0;
};

class JsonConfigParser : public IConfigParser {
public:
    JsonConfigParser() = default;
    ~JsonConfigParser() override = default;

    esp_err_t serialize(const ConfigData& config, std::string& output) const override;
    esp_err_t deserialize(const std::string& input, ConfigData& configOutput) const override;

private:
    static constexpr const char* TAG = "JsonConfigParser";

    // Helper macros for cleaner parsing
    #define GET_JSON_STRING(obj, key, target) \
        do { \
            cJSON* item = cJSON_GetObjectItem(obj, key); \
            if (item && cJSON_IsString(item)) target = item->valuestring; \
        } while(0)
    #define GET_JSON_NUMBER_INT(obj, key, target) \
        do { \
            cJSON* item = cJSON_GetObjectItem(obj, key); \
            if (item && cJSON_IsNumber(item)) target = item->valueint; \
            else ESP_LOGW(TAG, "Missing/invalid number '%s'", key); \
        } while(0)
     #define GET_JSON_NUMBER_INT_CAST(obj, key, target, type) \
        do { \
            cJSON* item = cJSON_GetObjectItem(obj, key); \
            if (item && cJSON_IsNumber(item)) target = (type)item->valueint; \
            else ESP_LOGW(TAG, "Missing/invalid number '%s'", key); \
        } while(0)
    #define GET_JSON_NUMBER_DOUBLE(obj, key, target) \
        do { \
            cJSON* item = cJSON_GetObjectItem(obj, key); \
            if (item && cJSON_IsNumber(item)) target = item->valuedouble; \
            else ESP_LOGW(TAG, "Missing/invalid number '%s'", key); \
        } while(0)
     #define GET_JSON_BOOL(obj, key, target) \
        do { \
            cJSON* item = cJSON_GetObjectItem(obj, key); \
            if (item && cJSON_IsBool(item)) target = cJSON_IsTrue(item); \
            else ESP_LOGW(TAG, "Missing/invalid bool '%s'", key); \
        } while(0)

    cJSON* serializePid(const PIDConfig& pidConfig) const;
    bool deserializePid(cJSON* pid_obj, PIDConfig& pidConfigOutput) const;
    cJSON* serializeControl(const ControlConfig& controlConfig) const;
    bool deserializeControl(cJSON* control_obj, ControlConfig& controlConfigOutput) const;

    // --- Helpers for new config sections ---
    cJSON* serializeBehavior(const SystemBehaviorConfig& b) const;
    bool deserializeBehavior(cJSON* obj, SystemBehaviorConfig& b) const;
    cJSON* serializeDimensions(const RobotDimensionsConfig& d) const;
    bool deserializeDimensions(cJSON* obj, RobotDimensionsConfig& d) const;
    cJSON* serializeWeb(const WebServerConfig& w) const;
    bool deserializeWeb(cJSON* obj, WebServerConfig& w) const;
};