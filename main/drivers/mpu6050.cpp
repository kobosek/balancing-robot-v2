// main/mpu6050.cpp
#include "mpu6050.hpp"                  // Relative path within module's include dir
#include "driver/i2c_master.h"
#include "esp_log.h"
#include <cstddef>                      // For size_t
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"              // For vTaskDelay

MPU6050::MPU6050() : _dev_handle(nullptr) {} // Initialize handle

esp_err_t MPU6050::init(const i2c_port_t i2c_port,
                        const gpio_num_t sda_io,
                        const gpio_num_t scl_io,
                        const uint16_t i2c_addr,
                        const uint32_t i2c_freq) {
    ESP_LOGD(TAG, "Initializing MPU6050 Driver");

    // --- Configure I2C Bus ---
    // Note: This assumes this is the ONLY user of this I2C bus.
    // If sharing the bus, the bus should be initialized externally.
    i2c_master_bus_config_t bus_config = {
        .i2c_port = i2c_port,
        .sda_io_num = sda_io,
        .scl_io_num = scl_io,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7, // Default
        .intr_priority = 0,  // Default interrupt flags
        .flags = {
            .enable_internal_pullup = true
        }
    };
    i2c_master_bus_handle_t bus_handle;
    esp_err_t ret = i2c_new_master_bus(&bus_config, &bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed create I2C master bus: %s", esp_err_to_name(ret));
        return ret;
    }

    // --- Add I2C Device ---
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = i2c_addr,
        .scl_speed_hz = i2c_freq,
    };
    // Add the device to the bus, store the device handle
    ret = i2c_master_bus_add_device(bus_handle, &dev_config, &_dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed add I2C device: %s", esp_err_to_name(ret));
        // Clean up bus? i2c_del_master_bus(bus_handle); // If bus init is local
        return ret;
    }
    if (_dev_handle == nullptr) {
         ESP_LOGE(TAG, "Failed to get valid I2C device handle");
         // i2c_del_master_bus(bus_handle); // If bus init is local
         return ESP_FAIL;
    }


    // --- Basic Sensor Setup ---
    // Wake up sensor and set clock source (recommended: PLL with gyro ref)
    ret = writeRegister(MPU6050Register::PWR_MGMT_1, static_cast<uint8_t>(MPU6050PowerManagement::CLOCK_PLL_XGYRO));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed set power management/clock source: %s", esp_err_to_name(ret));
        // Cleanup i2c resources...
        return ret;
    }
    // Small delay after wake-up/clock change
    vTaskDelay(pdMS_TO_TICKS(50)); // Allow sensor to stabilize


    ESP_LOGI(TAG, "MPU6050 Driver Initialized Successfully");
    return ESP_OK;
}

esp_err_t MPU6050::setDLPFConfig(MPU6050DLPFConfig config) {
    ESP_LOGD(TAG, "Setting DLPF config: 0x%02X", static_cast<uint8_t>(config));
    return writeRegister(MPU6050Register::DLPF_CONFIG, static_cast<uint8_t>(config));
}

esp_err_t MPU6050::setSampleRate(MPU6050SampleRateDiv rate) {
    ESP_LOGD(TAG, "Setting sample rate divisor: 0x%02X", static_cast<uint8_t>(rate));
    return writeRegister(MPU6050Register::SMPLRT_DIV, static_cast<uint8_t>(rate));
}

esp_err_t MPU6050::setAccelRange(MPU6050AccelConfig range) {
    ESP_LOGD(TAG, "Setting accelerometer range: 0x%02X", static_cast<uint8_t>(range));
    esp_err_t ret = writeRegister(MPU6050Register::ACCEL_CONFIG, static_cast<uint8_t>(range));
    if (ret == ESP_OK) {
        // Update internal scale factor based on the selected range
        switch(range) {
            case MPU6050AccelConfig::RANGE_2G:  _accel_scale = 16384.0f; break;
            case MPU6050AccelConfig::RANGE_4G:  _accel_scale = 8192.0f;  break;
            case MPU6050AccelConfig::RANGE_8G:  _accel_scale = 4096.0f;  break;
            case MPU6050AccelConfig::RANGE_16G: _accel_scale = 2048.0f;  break;
            default: ESP_LOGW(TAG, "Unknown accel range set, scale might be incorrect!"); break;
        }
         ESP_LOGD(TAG, "Accel scale set to: %.1f LSB/g", _accel_scale);
    }
    return ret;
}

esp_err_t MPU6050::setGyroRange(MPU6050GyroConfig range) {
    ESP_LOGD(TAG, "Setting gyroscope range: 0x%02X", static_cast<uint8_t>(range));
    esp_err_t ret = writeRegister(MPU6050Register::GYRO_CONFIG, static_cast<uint8_t>(range));
    if (ret == ESP_OK) {
        // Update internal scale factor based on the selected range
        switch(range) {
            case MPU6050GyroConfig::RANGE_250_DEG:  _gyro_scale = 131.0f; break;
            case MPU6050GyroConfig::RANGE_500_DEG:  _gyro_scale = 65.5f;  break;
            case MPU6050GyroConfig::RANGE_1000_DEG: _gyro_scale = 32.8f;  break;
            case MPU6050GyroConfig::RANGE_2000_DEG: _gyro_scale = 16.4f;  break;
             default: ESP_LOGW(TAG, "Unknown gyro range set, scale might be incorrect!"); break;
        }
        ESP_LOGD(TAG, "Gyro scale set to: %.1f LSB/(deg/s)", _gyro_scale);
    }
    return ret;
}

esp_err_t MPU6050::setupInterrupt(MPU6050InterruptPinConfig intPinConfig, MPU6050Interrupt intEnableBits) {
    ESP_LOGD(TAG, "Setting up interrupt pin config: 0x%02X, enable bits: 0x%02X",
            static_cast<uint8_t>(intPinConfig), static_cast<uint8_t>(intEnableBits));
    esp_err_t ret = writeRegister(MPU6050Register::INT_PIN_CFG, static_cast<uint8_t>(intPinConfig));
    if (ret == ESP_OK) {
        ret = writeRegister(MPU6050Register::INTERRUPT_EN, static_cast<uint8_t>(intEnableBits));
    }
    return ret;
}

esp_err_t MPU6050::setupFIFO(MPU6050UserControl userCtrlBits, MPU6050FIFOEnable fifoEnableBits) {
    ESP_LOGD(TAG, "Setting up FIFO user control: 0x%02X, enable bits: 0x%02X",
            static_cast<uint8_t>(userCtrlBits), static_cast<uint8_t>(fifoEnableBits));
    esp_err_t ret = writeRegister(MPU6050Register::USER_CTRL, static_cast<uint8_t>(userCtrlBits));
    if (ret == ESP_OK) {
        // Only write FIFO_EN if USER_CTRL succeeded and FIFO is actually being enabled
        if(static_cast<uint8_t>(userCtrlBits) & static_cast<uint8_t>(MPU6050UserControl::FIFO_ENABLE)) {
             ret = writeRegister(MPU6050Register::FIFO_EN, static_cast<uint8_t>(fifoEnableBits));
        }
    }
    return ret;
}

esp_err_t MPU6050::isDataReady() const {
    uint8_t int_status = 0;
    esp_err_t ret = readRegisters(MPU6050Register::INTERRUPT_STATUS, &int_status, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read interrupt status: %s", esp_err_to_name(ret));
        return ret;
    }
    if (int_status & static_cast<uint8_t>(MPU6050Interrupt::DATA_READY)) {
        ESP_LOGV(TAG, "Data Ready interrupt flag SET (Status: 0x%02X)", int_status);
        return ESP_OK; // Data is ready
    } else {
        ESP_LOGV(TAG, "Data Ready interrupt flag NOT SET (Status: 0x%02X)", int_status);
        return ESP_FAIL; // Data not ready (use FAIL instead of INVALID_STATE)
    }
}

esp_err_t MPU6050::isFIFOOverflow() const {
    uint8_t int_status = 0;
    esp_err_t ret = readRegisters(MPU6050Register::INTERRUPT_STATUS, &int_status, 1);
     if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read interrupt status: %s", esp_err_to_name(ret));
        return ret;
    }
    if (int_status & static_cast<uint8_t>(MPU6050Interrupt::FIFO_OVERFLOW)) {
        ESP_LOGW(TAG, "FIFO overflow interrupt flag SET (Status: 0x%02X)", int_status);
        return ESP_OK; // Overflow occurred
    } else {
        ESP_LOGV(TAG, "FIFO overflow interrupt flag NOT SET (Status: 0x%02X)", int_status);
        return ESP_FAIL; // No overflow (use FAIL)
    }
}

// --- REMOVED readFromFifo definition ---
// esp_err_t MPU6050::readFromFifo(...) { ... }

esp_err_t MPU6050::getAcceleration(float& ax, float& ay, float& az) const {
    uint8_t data[6];
    esp_err_t ret = readRegisters(MPU6050Register::ACCEL_XOUT_H, data, 6);
    if (ret != ESP_OK) { return ret; } // Propagate error

    int16_t raw_ax = (data[0] << 8) | data[1];
    int16_t raw_ay = (data[2] << 8) | data[3];
    int16_t raw_az = (data[4] << 8) | data[5];

    // Convert raw values to g's using the stored scale factor
    ax = static_cast<float>(raw_ax) / _accel_scale;
    ay = static_cast<float>(raw_ay) / _accel_scale;
    az = static_cast<float>(raw_az) / _accel_scale;

    ESP_LOGV(TAG, "Raw Accel: X=%d, Y=%d, Z=%d | Scaled Accel (g): X=%.2f, Y=%.2f, Z=%.2f", raw_ax, raw_ay, raw_az, ax, ay, az);
    return ESP_OK;
}

esp_err_t MPU6050::getRotation(float& gx, float& gy, float& gz) const {
    uint8_t data[6];
    esp_err_t ret = readRegisters(MPU6050Register::GYRO_XOUT_H, data, 6);
     if (ret != ESP_OK) { return ret; } // Propagate error

    int16_t raw_gx = (data[0] << 8) | data[1];
    int16_t raw_gy = (data[2] << 8) | data[3];
    int16_t raw_gz = (data[4] << 8) | data[5];

    // Convert raw values to deg/s using the stored scale factor
    gx = static_cast<float>(raw_gx) / _gyro_scale;
    gy = static_cast<float>(raw_gy) / _gyro_scale;
    gz = static_cast<float>(raw_gz) / _gyro_scale;

    ESP_LOGV(TAG, "Raw Gyro: X=%d, Y=%d, Z=%d | Scaled Gyro (dps): X=%.2f, Y=%.2f, Z=%.2f", raw_gx, raw_gy, raw_gz, gx, gy, gz);
    return ESP_OK;
}

// --- Low-level I2C Read/Write (Moved to public in header) ---
esp_err_t MPU6050::writeRegister(MPU6050Register reg, uint8_t data) {
     if (_dev_handle == nullptr) {
        ESP_LOGE(TAG, "Device handle not initialized for writeRegister");
        return ESP_ERR_INVALID_STATE;
    }
    uint8_t write_buf[2] = {static_cast<uint8_t>(reg), data};
    // Use i2c_master_transmit with timeout (e.g., 100ms)
    esp_err_t ret = i2c_master_transmit(_dev_handle, write_buf, sizeof(write_buf), pdMS_TO_TICKS(100));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C transmit failed to reg 0x%02X: %s", static_cast<uint8_t>(reg), esp_err_to_name(ret));
    } else {
         ESP_LOGV(TAG, "Wrote 0x%02X to reg 0x%02X", data, static_cast<uint8_t>(reg));
    }
    return ret;
}

esp_err_t MPU6050::readRegisters(MPU6050Register reg, uint8_t* data, size_t len) const {
     if (_dev_handle == nullptr) {
        ESP_LOGE(TAG, "Device handle not initialized for readRegisters");
        return ESP_ERR_INVALID_STATE;
    }
     if (data == nullptr || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t reg_addr = static_cast<uint8_t>(reg);
    // Use i2c_master_transmit_receive with timeout
    esp_err_t ret = i2c_master_transmit_receive(_dev_handle, &reg_addr, 1, data, len, pdMS_TO_TICKS(100));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C transmit_receive failed from reg 0x%02X: %s", reg_addr, esp_err_to_name(ret));
    } else {
         ESP_LOGV(TAG, "Read %d bytes from reg 0x%02X", len, reg_addr);
    }
    return ret;
}