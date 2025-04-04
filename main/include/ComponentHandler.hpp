#pragma once

#include "esp_err.h"
#include "esp_log.h"
#include <vector>
#include <memory>

class IConfigObserver;
class IMotorDriver;
class IWebServer;
class IPIDController;
class IMPU6050Manager;
class IWiFiManager;
class IRuntimeConfig;
class IEncoder;

class ComponentHandler {
public:
    ComponentHandler(IRuntimeConfig& config) : runtimeConfig(config) {}
    esp_err_t init();
    void notifyConfigUpdate();

    const IWiFiManager& getWifiManager() { return *wifiManager; }
    IWebServer& getWebServer() { return *webServer; }
    IPIDController& getAnglePIDController() { return *anglePidController; }
    IPIDController& getSpeedPIDControllerLeft() { return *speedPidControllerLeft; }
    IPIDController& getSpeedPIDControllerRight() { return *speedPidControllerRight; }
    IMPU6050Manager& getMPU6050Manager() { return *mpu6050Manager; }

    IMotorDriver& getMotorLeft() {return *motorLeft; }
    IMotorDriver& getMotorRight() {return *motorRight; }
    IEncoder& getEncoderLeft() { return *encoderLeft; }
    IEncoder& getEncoderRight() { return *encoderRight; }

private:
    static constexpr const char* TAG = "Component Handler";
    
    void registerObserver(std::shared_ptr<IConfigObserver>);
    void unregisterObserver(std::shared_ptr<IConfigObserver>);

    IRuntimeConfig& runtimeConfig;

    std::shared_ptr<IWiFiManager> wifiManager;
    std::shared_ptr<IWebServer> webServer;
    std::shared_ptr<IPIDController> anglePidController;
    std::shared_ptr<IPIDController> speedPidControllerLeft;
    std::shared_ptr<IPIDController> speedPidControllerRight;
    std::shared_ptr<IMPU6050Manager> mpu6050Manager;

    std::shared_ptr<IMotorDriver> motorLeft;
    std::shared_ptr<IMotorDriver> motorRight;
    std::shared_ptr<IEncoder> encoderLeft;
    std::shared_ptr<IEncoder> encoderRight;

    std::vector<std::shared_ptr<IConfigObserver>> observers;
};
