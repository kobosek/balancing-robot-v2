#include "I2CDevice.hpp"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {
constexpr const char* TAG = "I2CDevice";
}

I2CDevice::I2CDevice() :
    m_mutex(),
    m_healthTracker(),
    m_config(),
    m_busHandle(nullptr),
    m_deviceHandle(nullptr) {}

I2CDevice::~I2CDevice() {
    close();
}

esp_err_t I2CDevice::open(i2c_port_t i2cPort,
                          gpio_num_t sdaPin,
                          gpio_num_t sclPin,
                          uint16_t deviceAddress,
                          uint32_t i2cFrequencyHz) {
    std::lock_guard<std::mutex> lock(m_mutex);
    closeUnlocked();

    i2c_master_bus_config_t busConfig{};
    busConfig.i2c_port = i2cPort;
    busConfig.sda_io_num = sdaPin;
    busConfig.scl_io_num = sclPin;
    busConfig.clk_source = I2C_CLK_SRC_DEFAULT;
    busConfig.glitch_ignore_cnt = 7;
    busConfig.intr_priority = 0;
    busConfig.trans_queue_depth = 0;
    busConfig.flags.enable_internal_pullup = true;

    esp_err_t ret = i2c_new_master_bus(&busConfig, &m_busHandle);
    if (ret != ESP_OK || m_busHandle == nullptr) {
        ESP_LOGE(TAG, "Failed to create I2C bus: %s", esp_err_to_name(ret));
        m_busHandle = nullptr;
        return ret != ESP_OK ? ret : ESP_FAIL;
    }

    i2c_device_config_t deviceConfig{};
    deviceConfig.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    deviceConfig.device_address = deviceAddress;
    deviceConfig.scl_speed_hz = i2cFrequencyHz;
    deviceConfig.scl_wait_us = 20000;
    deviceConfig.flags.disable_ack_check = false;

    ret = i2c_master_bus_add_device(m_busHandle, &deviceConfig, &m_deviceHandle);
    if (ret != ESP_OK || m_deviceHandle == nullptr) {
        ESP_LOGE(TAG, "Failed to add I2C device: %s", esp_err_to_name(ret));
        closeUnlocked();
        return ret != ESP_OK ? ret : ESP_FAIL;
    }

    return ESP_OK;
}

void I2CDevice::close() {
    std::lock_guard<std::mutex> lock(m_mutex);
    closeUnlocked();
}

bool I2CDevice::isOpen() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_deviceHandle != nullptr;
}

esp_err_t I2CDevice::writeRegister(uint8_t reg, uint8_t data) {
    esp_err_t lastError = ESP_FAIL;
    uint8_t attempts = 0;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_deviceHandle == nullptr) {
            m_healthTracker.recordFailure(ESP_ERR_INVALID_STATE);
            return ESP_ERR_INVALID_STATE;
        }
        attempts = static_cast<uint8_t>(m_config.max_retries + 1);
    }

    for (uint8_t attempt = 0; attempt < attempts; ++attempt) {
        uint32_t retryDelayMs = 0;
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            if (m_deviceHandle == nullptr) {
                m_healthTracker.recordFailure(ESP_ERR_INVALID_STATE);
                return ESP_ERR_INVALID_STATE;
            }

            retryDelayMs = m_config.retry_delay_ms;
            lastError = writeRegisterOnce(reg, data);
            if (lastError == ESP_OK) {
                m_healthTracker.recordSuccess();
                return ESP_OK;
            }

            m_healthTracker.recordFailure(lastError);
        }

        if (attempt + 1 < attempts) {
            vTaskDelay(pdMS_TO_TICKS(retryDelayMs));
        }
    }

    return lastError;
}

esp_err_t I2CDevice::readRegisters(uint8_t reg, uint8_t* data, size_t len) const {
    if (data == nullptr || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t lastError = ESP_FAIL;
    uint8_t attempts = 0;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_deviceHandle == nullptr) {
            m_healthTracker.recordFailure(ESP_ERR_INVALID_STATE);
            return ESP_ERR_INVALID_STATE;
        }
        attempts = static_cast<uint8_t>(m_config.max_retries + 1);
    }

    for (uint8_t attempt = 0; attempt < attempts; ++attempt) {
        uint32_t retryDelayMs = 0;
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            if (m_deviceHandle == nullptr) {
                m_healthTracker.recordFailure(ESP_ERR_INVALID_STATE);
                return ESP_ERR_INVALID_STATE;
            }

            retryDelayMs = m_config.retry_delay_ms;
            lastError = readRegistersOnce(reg, data, len);
            if (lastError == ESP_OK) {
                m_healthTracker.recordSuccess();
                return ESP_OK;
            }

            m_healthTracker.recordFailure(lastError);
        }

        if (attempt + 1 < attempts) {
            vTaskDelay(pdMS_TO_TICKS(retryDelayMs));
        }
    }

    return lastError;
}

void I2CDevice::setConfig(const I2CConfig& config) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_config = config;
}

I2CConfig I2CDevice::getConfig() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_config;
}

I2CStats I2CDevice::getStats() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_healthTracker.getStats();
}

void I2CDevice::resetStats() {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_healthTracker.reset();
}

esp_err_t I2CDevice::writeRegisterOnce(uint8_t reg, uint8_t data) const {
    uint8_t writeBuffer[2] = {reg, data};
    return i2c_master_transmit(
        m_deviceHandle,
        writeBuffer,
        sizeof(writeBuffer),
        static_cast<int>(m_config.timeout_ms));
}

esp_err_t I2CDevice::readRegistersOnce(uint8_t reg, uint8_t* data, size_t len) const {
    return i2c_master_transmit_receive(
        m_deviceHandle,
        &reg,
        1,
        data,
        len,
        static_cast<int>(m_config.timeout_ms));
}

void I2CDevice::closeUnlocked() {
    if (m_deviceHandle != nullptr) {
        i2c_master_bus_rm_device(m_deviceHandle);
        m_deviceHandle = nullptr;
    }
    if (m_busHandle != nullptr) {
        i2c_del_master_bus(m_busHandle);
        m_busHandle = nullptr;
    }
}
