#pragma once

#include "MPU6050Profile.hpp"
#include "config/MPU6050Config.hpp"
#include "esp_err.h"

class I2CDevice;
class MPU6050Driver;

class MPU6050HardwareController {
public:
    MPU6050HardwareController(I2CDevice& i2cDevice, MPU6050Driver& driver);

    esp_err_t connectAndConfigure(const MPU6050Config& config, MPU6050Profile& profile);
    esp_err_t applyConfiguration(const MPU6050Config& config, const MPU6050Profile& profile);
    void disconnect();

private:
    esp_err_t probeSensor() const;
    esp_err_t clearFifoState();

    I2CDevice& m_i2cDevice;
    MPU6050Driver& m_driver;
};
