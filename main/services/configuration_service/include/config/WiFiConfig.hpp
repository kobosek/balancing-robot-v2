#pragma once

#include <string>

struct WiFiConfig {
    std::string ssid = "DEFAULT_SSID";
    std::string password = "DEFAULT_PASSWORD";

    bool operator!=(const WiFiConfig& other) const {
        return ssid != other.ssid ||
               password != other.password;
    }

    bool operator==(const WiFiConfig& other) const {
        return !(*this != other);
    }
};
