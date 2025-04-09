#pragma once

#include "esp_err.h"
#include <string>
#include "cJSON.h"

struct ConfigData;
struct PIDConfig;
struct ControlConfig;

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
    cJSON* serializePid(const PIDConfig& pidConfig) const;
    bool deserializePid(cJSON* pid_obj, PIDConfig& pidConfigOutput) const;
    cJSON* serializeControl(const ControlConfig& controlConfig) const;
    bool deserializeControl(cJSON* control_obj, ControlConfig& controlConfigOutput) const;
};