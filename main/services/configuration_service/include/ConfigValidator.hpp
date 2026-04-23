#pragma once

#include "ConfigData.hpp"
#include <string>

class ConfigValidator {
public:
    bool validate(const ConfigData& config, std::string& error) const;
};
