#pragma once

#include "esp_err.h"
#include <string>                     // For std::string
#include "cJSON.h"                    // Needed for private helper declarations

// Forward declare ConfigData struct (if not including ConfigData.hpp directly)
struct ConfigData; // Defined in config/include
struct PIDConfig;  // Defined in config/include

// Define the interface if still needed, otherwise remove
class IConfigParser {
public:
    virtual ~IConfigParser() = default;
    virtual esp_err_t serialize(const ConfigData& config, std::string& output) const = 0;
    virtual esp_err_t deserialize(const std::string& input, ConfigData& configOutput) const = 0;
};

class JsonConfigParser : public IConfigParser { // Inherit from interface if kept
public:
    JsonConfigParser() = default;
    ~JsonConfigParser() override = default;

    esp_err_t serialize(const ConfigData& config, std::string& output) const override;
    esp_err_t deserialize(const std::string& input, ConfigData& configOutput) const override;

private:
    static constexpr const char* TAG = "JsonConfigParser";
    cJSON* serializePid(const PIDConfig& pidConfig) const;
    bool deserializePid(cJSON* pid_obj, PIDConfig& pidConfigOutput) const;
};