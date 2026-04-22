#include "MPU6050HardwareController.hpp"

#include "I2CDevice.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mpu6050.hpp"

namespace {
constexpr const char* TAG = "MPU6050HwCtrl";
constexpr uint32_t SAFE_I2C_FREQUENCY_HZ = 100000;
constexpr TickType_t DEVICE_RESET_DELAY_TICKS = pdMS_TO_TICKS(100);
constexpr TickType_t CLOCK_SETTLE_DELAY_TICKS = pdMS_TO_TICKS(30);
}

MPU6050HardwareController::MPU6050HardwareController(I2CDevice& i2cDevice, MPU6050Driver& driver) :
    m_i2cDevice(i2cDevice),
    m_driver(driver) {}

esp_err_t MPU6050HardwareController::connectAndConfigure(const MPU6050Config& config, MPU6050Profile& profile) {
    auto tryConnectAtFrequency = [&](uint32_t i2cFrequencyHz) {
        esp_err_t ret = m_i2cDevice.open(
            static_cast<i2c_port_t>(config.i2c_port),
            static_cast<gpio_num_t>(config.sda_pin),
            static_cast<gpio_num_t>(config.scl_pin),
            config.device_address,
            i2cFrequencyHz);
        if (ret != ESP_OK) {
            return ret;
        }

        ret = probeSensor();
        if (ret != ESP_OK) {
            m_i2cDevice.close();
            return ret;
        }

        profile = MPU6050Profile::fromConfig(config);
        if (profile.sampleRateLimitedByTransport) {
            ESP_LOGW(
                TAG,
                "Limiting MPU sample rate divisor from %u to %u for %lu Hz I2C",
                static_cast<unsigned>(config.sample_rate_divisor & 0xFF),
                static_cast<unsigned>(static_cast<uint8_t>(profile.sampleRateDivReg)),
                static_cast<unsigned long>(i2cFrequencyHz));
        }
        ret = applyConfiguration(config, profile);
        if (ret != ESP_OK) {
            m_i2cDevice.close();
        }
        return ret;
    };

    esp_err_t ret = tryConnectAtFrequency(config.i2c_freq_hz);
    if (ret == ESP_OK || config.i2c_freq_hz <= SAFE_I2C_FREQUENCY_HZ) {
        return ret;
    }

    ESP_LOGW(
        TAG,
        "IMU attach failed at %lu Hz (%s), retrying at %lu Hz",
        static_cast<unsigned long>(config.i2c_freq_hz),
        esp_err_to_name(ret),
        static_cast<unsigned long>(SAFE_I2C_FREQUENCY_HZ));
    return tryConnectAtFrequency(SAFE_I2C_FREQUENCY_HZ);
}

esp_err_t MPU6050HardwareController::applyConfiguration(const MPU6050Config& config, const MPU6050Profile& profile) {
    esp_err_t ret = m_driver.resetSensor();
    if (ret != ESP_OK) {
        return ret;
    }
    vTaskDelay(DEVICE_RESET_DELAY_TICKS);

    ret = m_driver.setPowerManagementReg(MPU6050PowerManagement::CLOCK_PLL_X_GYRO);
    if (ret != ESP_OK) {
        return ret;
    }
    vTaskDelay(CLOCK_SETTLE_DELAY_TICKS);

    ret = m_driver.setAccelRangeReg(profile.accelRangeReg);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = m_driver.setGyroRangeReg(profile.gyroRangeReg);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = m_driver.setDLPFConfigReg(profile.dlpfReg);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = m_driver.setSampleRateDivReg(profile.sampleRateDivReg);
    if (ret != ESP_OK) {
        return ret;
    }

    const MPU6050Interrupt interruptBits = profile.interruptEnabled ?
        MPU6050Interrupt::DATA_READY :
        static_cast<MPU6050Interrupt>(0);
    ret = m_driver.configureInterruptPinReg(profile.interruptPinConfig, interruptBits);
    if (ret != ESP_OK) {
        return ret;
    }

    (void)config;
    return clearFifoState();
}

void MPU6050HardwareController::disconnect() {
    m_i2cDevice.close();
}

esp_err_t MPU6050HardwareController::probeSensor() const {
    uint8_t deviceId = 0;
    esp_err_t ret = m_driver.getDeviceID(deviceId);
    if (ret != ESP_OK) {
        return ret;
    }

    if (deviceId != 0x68 && deviceId != 0x69) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    return ESP_OK;
}

esp_err_t MPU6050HardwareController::clearFifoState() {
    esp_err_t ret = m_driver.resetFIFO();
    if (ret != ESP_OK) {
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(2));

    ret = m_driver.resetSignalPath();
    if (ret != ESP_OK) {
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(2));

    return m_driver.disableFIFO();
}
