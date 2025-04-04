#pragma once

#include "interfaces/IRuntimeConfig.hpp"
#include "esp_log.h"

class RuntimeConfig : public IRuntimeConfig{
public:
    esp_err_t init(const std::string& p_filename = "/spiffs/config.json") override;
    esp_err_t save(const std::string& p_filename = "/spiffs/config.json") const override;


    PIDConfig getAnglePidConfig() const override { return anglePidConfig;};
    void setAnglePidConfig(PIDConfig config) override { anglePidConfig = config; };

    PIDConfig getSpeedPidLeftConfig() const override { return speedPidLeftConfig; };
    void setSpeedPidLeftConfig(PIDConfig config ) override { speedPidLeftConfig = config; };

    PIDConfig getSpeedPidRightConfig() const override { return speedPidRightConfig; };
    void setSpeedPidRightConfig(PIDConfig config ) override { speedPidRightConfig = config; };

    // MPU6050 parameters
    int getMpu6050CalibrationSamples() const override { return mpu6050_calibration_samples; }
    void setMpu6050CalibrationSamples(int value) override { mpu6050_calibration_samples = value; }

    // Main loop parameters
    int getMainLoopIntervalMs() const override { return main_loop_interval_ms; }
    void setMainLoopIntervalMs(int value) override { main_loop_interval_ms = value; }

    // WiFi parameters
    std::string getWifiSsid() const override { return wifi_ssid; }
    std::string getWifiPassword() const override { return wifi_password; }
    void setWifiSsid(const std::string& value) override { wifi_ssid = value; }
    void setWifiPassword(const std::string& value) override { wifi_password = value; }

    // JSON serialization/deserialization
    std::string toJson() const override;
    esp_err_t fromJson(const std::string& json) override;

private:
    static constexpr const char* TAG = "RuntimeConfig";
    esp_err_t init_spiffs();
    esp_err_t init_nvs_Flash();

    PIDConfig anglePidConfig;
    PIDConfig speedPidLeftConfig;
    PIDConfig speedPidRightConfig;

    // MPU6050 parameters
    int mpu6050_calibration_samples;

    // Main loop parameters
    int main_loop_interval_ms;

    // WiFi parameters
    std::string wifi_ssid;
    std::string wifi_password;
};