#include "mpu6050.hpp"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h" // For vTaskDelay

MPU6050Driver::MPU6050Driver() :
    _dev_handle(nullptr),
    _i2c_mutex()
{}

esp_err_t MPU6050Driver::init(const i2c_port_t i2c_port,
                               const gpio_num_t sda_io,
                               const gpio_num_t scl_io,
                               const uint16_t i2c_addr,
                               const uint32_t i2c_freq) {
    ESP_LOGD(TAG, "Initializing MPU6050 Driver");
    std::lock_guard<std::mutex> lock(_i2c_mutex); // Lock during init

    if (_dev_handle) {
         ESP_LOGW(TAG, "Driver already initialized.");
         return ESP_OK;
    }

    // --- Configure I2C Bus ---
    i2c_master_bus_config_t bus_config = {
        .i2c_port = i2c_port,
        .sda_io_num = sda_io,
        .scl_io_num = scl_io,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .flags = { .enable_internal_pullup = true }
    };
    i2c_master_bus_handle_t bus_handle;
    // Check if bus already exists? For simplicity, assume we own it.
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
    ret = i2c_master_bus_add_device(bus_handle, &dev_config, &_dev_handle);
    if (ret != ESP_OK || _dev_handle == nullptr) {
        ESP_LOGE(TAG, "Failed add I2C device: %s", esp_err_to_name(ret));
        // i2c_del_master_bus(bus_handle); // Cleanup bus if created locally
        _dev_handle = nullptr; // Ensure handle is null on failure
        return ret == ESP_OK ? ESP_FAIL : ret; // Return ESP_FAIL if ret was OK but handle is null
    }

    ESP_LOGI(TAG, "MPU6050 Driver Initialized Successfully (Device Handle Created)");
    return ESP_OK;
}

// --- Configuration Methods ---

esp_err_t MPU6050Driver::setPowerManagementReg(MPU6050PowerManagement powerBits) {
     ESP_LOGD(TAG, "Setting PWR_MGMT_1: 0x%02X", static_cast<uint8_t>(powerBits));
     return writeRegister(MPU6050Register::PWR_MGMT_1, static_cast<uint8_t>(powerBits));
}

esp_err_t MPU6050Driver::resetSensor() {
     ESP_LOGI(TAG, "Resetting MPU6050 sensor...");
     // Set reset bit
     esp_err_t ret = writeRegister(MPU6050Register::PWR_MGMT_1, static_cast<uint8_t>(MPU6050PowerManagement::RESET));
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to write RESET bit: %s", esp_err_to_name(ret));
         return ret;
     }
     // Wait for reset to complete (MPU6050 datasheet recommends >50ms, 100ms is safe)
     vTaskDelay(pdMS_TO_TICKS(100));

     // Optional: Wake up sensor after reset (might be done by subsequent config)
     // ret = setPowerManagementReg(MPU6050PowerManagement::CLOCK_PLL_XGYRO); // Example wake-up
     // if (ret != ESP_OK) {
     //     ESP_LOGE(TAG, "Failed to wake sensor after reset: %s", esp_err_to_name(ret));
     // } else {
          ESP_LOGI(TAG, "Sensor reset command sent.");
     // }
     return ESP_OK; // Return status of the reset command itself
}

esp_err_t MPU6050Driver::setDLPFConfigReg(MPU6050DLPFConfig config) {
    ESP_LOGD(TAG, "Setting DLPF_CONFIG: 0x%02X", static_cast<uint8_t>(config));
    return writeRegister(MPU6050Register::DLPF_CONFIG, static_cast<uint8_t>(config));
}

esp_err_t MPU6050Driver::setSampleRateDivReg(MPU6050SampleRateDiv rate_div) {
    ESP_LOGD(TAG, "Setting SMPLRT_DIV: 0x%02X", static_cast<uint8_t>(rate_div));
    return writeRegister(MPU6050Register::SMPLRT_DIV, static_cast<uint8_t>(rate_div));
}

esp_err_t MPU6050Driver::setAccelRangeReg(MPU6050AccelConfig range) {
    ESP_LOGD(TAG, "Setting ACCEL_CONFIG: 0x%02X", static_cast<uint8_t>(range));
    return writeRegister(MPU6050Register::ACCEL_CONFIG, static_cast<uint8_t>(range));
}

esp_err_t MPU6050Driver::setGyroRangeReg(MPU6050GyroConfig range) {
    ESP_LOGD(TAG, "Setting GYRO_CONFIG: 0x%02X", static_cast<uint8_t>(range));
    return writeRegister(MPU6050Register::GYRO_CONFIG, static_cast<uint8_t>(range));
}

esp_err_t MPU6050Driver::configureInterruptPinReg(MPU6050InterruptPinConfig intPinConfig, MPU6050Interrupt intEnableBits) {
    ESP_LOGD(TAG, "Setting INT_PIN_CFG: 0x%02X, INTERRUPT_EN: 0x%02X",
            static_cast<uint8_t>(intPinConfig), static_cast<uint8_t>(intEnableBits));
    esp_err_t ret = writeRegister(MPU6050Register::INT_PIN_CFG, static_cast<uint8_t>(intPinConfig));
    if (ret == ESP_OK) {
        ret = writeRegister(MPU6050Register::INTERRUPT_EN, static_cast<uint8_t>(intEnableBits));
    }
    return ret;
}

esp_err_t MPU6050Driver::configureFIFOReg(MPU6050UserControl userCtrlBits, MPU6050FIFOEnable fifoEnableBits) {
     ESP_LOGD(TAG, "Setting USER_CTRL: 0x%02X, FIFO_EN: 0x%02X",
            static_cast<uint8_t>(userCtrlBits), static_cast<uint8_t>(fifoEnableBits));
    // Write USER_CTRL first (handles FIFO_ENABLE bit)
    esp_err_t ret = writeRegister(MPU6050Register::USER_CTRL, static_cast<uint8_t>(userCtrlBits));
    if (ret == ESP_OK) {
        // Only write FIFO_EN if USER_CTRL succeeded and FIFO is actually being enabled
        if(static_cast<uint8_t>(userCtrlBits) & static_cast<uint8_t>(MPU6050UserControl::FIFO_ENABLE)) {
             ret = writeRegister(MPU6050Register::FIFO_EN, static_cast<uint8_t>(fifoEnableBits));
             if (ret != ESP_OK) { ESP_LOGE(TAG, "Failed to write FIFO_EN register: %s", esp_err_to_name(ret)); }
        }
    } else {
         ESP_LOGE(TAG, "Failed to write USER_CTRL register: %s", esp_err_to_name(ret));
    }
    return ret;
}

// --- Raw Data Reading --- REMOVED SCALING

esp_err_t MPU6050Driver::readRawAccelXYZ(int16_t& ax, int16_t& ay, int16_t& az) const {
    uint8_t data[6];
    esp_err_t ret = readRegisters(MPU6050Register::ACCEL_XOUT_H, data, 6);
    if (ret != ESP_OK) { return ret; }

    // Combine bytes (Big Endian)
    ax = (int16_t)((data[0] << 8) | data[1]);
    ay = (int16_t)((data[2] << 8) | data[3]);
    az = (int16_t)((data[4] << 8) | data[5]);

    ESP_LOGV(TAG, "Read Raw Accel: X=%d, Y=%d, Z=%d", ax, ay, az);
    return ESP_OK;
}

esp_err_t MPU6050Driver::readRawGyroXYZ(int16_t& gx, int16_t& gy, int16_t& gz) const {
    uint8_t data[6];
    esp_err_t ret = readRegisters(MPU6050Register::GYRO_XOUT_H, data, 6);
     if (ret != ESP_OK) { return ret; }

    // Combine bytes (Big Endian)
    gx = (int16_t)((data[0] << 8) | data[1]);
    gy = (int16_t)((data[2] << 8) | data[3]);
    gz = (int16_t)((data[4] << 8) | data[5]);

    ESP_LOGV(TAG, "Read Raw Gyro: X=%d, Y=%d, Z=%d", gx, gy, gz);
    return ESP_OK;
}

// --- FIFO and Status ---

esp_err_t MPU6050Driver::readFifoCount(uint16_t& count) const {
    uint8_t count_buf[2];
    esp_err_t ret = readRegisters(MPU6050Register::FIFO_COUNT_H, count_buf, 2);
    if (ret == ESP_OK) {
        count = (uint16_t)((count_buf[0] << 8) | count_buf[1]);
        ESP_LOGV(TAG, "Read FIFO Count: %d bytes", count);
    } else {
        count = 0;
        ESP_LOGE(TAG, "Failed to read FIFO count: %s", esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t MPU6050Driver::readFifoBuffer(uint8_t* buffer, size_t len) const {
     if (len == 0) return ESP_OK; // Nothing to read
     if (!buffer) return ESP_ERR_INVALID_ARG;
     ESP_LOGV(TAG, "Reading %d bytes from FIFO buffer", len);
     return readRegisters(MPU6050Register::FIFO_R_W, buffer, len);
}

esp_err_t MPU6050Driver::readInterruptStatus(uint8_t& status) const {
     esp_err_t ret = readRegisters(MPU6050Register::INTERRUPT_STATUS, &status, 1);
      if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to read interrupt status: %s", esp_err_to_name(ret));
     } else {
         ESP_LOGV(TAG, "Read Interrupt Status: 0x%02X", status);
     }
     return ret;
}

esp_err_t MPU6050Driver::isFIFOOverflow(bool& isOverflow) const {
     uint8_t status = 0;
     isOverflow = false; // Default
     esp_err_t ret = readInterruptStatus(status);
     if (ret == ESP_OK) {
         isOverflow = (status & static_cast<uint8_t>(MPU6050Interrupt::FIFO_OVERFLOW));
         if (isOverflow) {
             ESP_LOGW(TAG, "FIFO Overflow Flag SET in Interrupt Status (0x%02X)", status);
         } else {
              ESP_LOGV(TAG, "FIFO Overflow Flag NOT SET in Interrupt Status (0x%02X)", status);
         }
     }
     // Return the status of the I2C read operation, not the overflow state itself
     return ret;
}


// --- Low-level I2C Communication ---

esp_err_t MPU6050Driver::writeRegister(MPU6050Register reg, uint8_t data) {
    std::lock_guard<std::mutex> lock(_i2c_mutex);
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

esp_err_t MPU6050Driver::readRegisters(MPU6050Register reg, uint8_t* data, size_t len) const {
    std::lock_guard<std::mutex> lock(_i2c_mutex);
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