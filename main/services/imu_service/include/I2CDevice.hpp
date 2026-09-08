#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"
#include <cstddef>
#include <cstdint>
#include <mutex>

struct I2CConfig {
    uint32_t timeout_ms = 40;
};

class I2CDevice {
public:
    I2CDevice();
    ~I2CDevice();

    esp_err_t open(i2c_port_t i2cPort,
                   gpio_num_t sdaPin,
                   gpio_num_t sclPin,
                   uint16_t deviceAddress,
                   uint32_t i2cFrequencyHz);
    esp_err_t close();

    bool isOpen() const;

    esp_err_t writeRegister(uint8_t reg, uint8_t data);
    esp_err_t readRegisters(uint8_t reg, uint8_t* data, size_t len, uint32_t timeoutMs = 0) const;

    void setConfig(const I2CConfig& config);
    I2CConfig getConfig() const;

private:
    esp_err_t writeRegisterOnce(uint8_t reg, uint8_t data) const;
    esp_err_t readRegistersOnce(uint8_t reg, uint8_t* data, size_t len, uint32_t timeoutMs = 0) const;
    esp_err_t closeUnlocked();

    mutable std::mutex m_mutex;
    I2CConfig m_config{};
    i2c_master_bus_handle_t m_busHandle;
    i2c_master_dev_handle_t m_deviceHandle;
};
