#pragma once

#include "ConfigData.hpp" // Include full definition here
#include "esp_err.h"
#include <string>

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
};
