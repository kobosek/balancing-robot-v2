#include "include/mpu6050.hpp"

MPU6050::MPU6050() : _dev_handle(NULL) {}

esp_err_t MPU6050::init(const i2c_port_t i2c_port, 
                        const gpio_num_t sda_io, 
                        const gpio_num_t scl_io,
                        const uint16_t i2c_addr,
                        const uint32_t i2c_freq) {
    ESP_LOGI(TAG, "Initializing MPU6050");

    i2c_master_bus_config_t bus_config = {
        .i2c_port = i2c_port,
        .sda_io_num = sda_io,
        .scl_io_num = scl_io,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags = {
            .enable_internal_pullup = true
        }
    };

    i2c_master_bus_handle_t bus_handle;
    esp_err_t ret = i2c_new_master_bus(&bus_config, &bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C master bus: %s", esp_err_to_name(ret));
        return ret;
    }

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = i2c_addr,
        .scl_speed_hz = i2c_freq,
    };

    ret = i2c_master_bus_add_device(bus_handle, &dev_config, &_dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = writeRegister(MPU6050Register::PWR_MGMT_1, static_cast<uint8_t>(MPU6050PowerManagement::CLOCK_INTERNAL));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set power management: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "MPU6050 initialized successfully");
    return ESP_OK;
}

esp_err_t MPU6050::setDLPFConfig(MPU6050DLPFConfig config) {
    ESP_LOGD(TAG, "Setting DLPF config");
    return writeRegister(MPU6050Register::DLPF_CONFIG, static_cast<uint8_t>(config));
}

esp_err_t MPU6050::setSampleRate(MPU6050SampleRateDiv rate) {
    ESP_LOGD(TAG, "Setting sample rate");
    return writeRegister(MPU6050Register::SMPLRT_DIV, static_cast<uint8_t>(rate));
}

esp_err_t MPU6050::setAccelRange(MPU6050AccelConfig range) {
    ESP_LOGD(TAG, "Setting accelerometer range");
    esp_err_t ret = writeRegister(MPU6050Register::ACCEL_CONFIG, static_cast<uint8_t>(range));
    if (ret == ESP_OK) {
        switch(range) {
            case MPU6050AccelConfig::RANGE_2G: _accel_scale = 16384.0f; break;
            case MPU6050AccelConfig::RANGE_4G: _accel_scale = 8192.0f; break;
            case MPU6050AccelConfig::RANGE_8G: _accel_scale = 4096.0f; break;
            case MPU6050AccelConfig::RANGE_16G: _accel_scale = 2048.0f; break;
        }
    }
    return ret;
}

esp_err_t MPU6050::setupInterrupt(MPU6050InterruptPinConfig intPinConfig, MPU6050Interrupt intEn) {
    ESP_LOGD(TAG, "Setting up interrupt");
    esp_err_t ret = writeRegister(MPU6050Register::INT_PIN_CFG, static_cast<uint8_t>(intPinConfig));
    if (ret == ESP_OK) {
        ret = writeRegister(MPU6050Register::INTERRUPT_EN, static_cast<uint8_t>(intEn));
    }
    return ret;
}

esp_err_t MPU6050::isDataReady() const {
    ESP_LOGD(TAG, "Checking interrupt flag");
    uint8_t _int_status;
    esp_err_t ret = readRegisters(MPU6050Register::INTERRUPT_STATUS, &_int_status, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read interrupt status: %s", esp_err_to_name(ret));
        return ret;
    }
    if (_int_status & static_cast<uint8_t>(MPU6050Interrupt::DATA_READY)) {
        ESP_LOGD(TAG, "Data Ready interrupt triggered");
        return ret;
    } else {
        ESP_LOGD(TAG, "Data Ready interrupt not triggered");
        return ESP_ERR_INVALID_STATE;
    }
}

esp_err_t MPU6050::isFIFOOverflow() const {
    ESP_LOGD(TAG, "Checking interrupt flag");
    uint8_t _int_status;
    esp_err_t ret = readRegisters(MPU6050Register::INTERRUPT_STATUS, &_int_status, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read interrupt status: %s", esp_err_to_name(ret));
        return ret;
    }
    if (_int_status & static_cast<uint8_t>(MPU6050Interrupt::FIFO_OVERFLOW)) {
        ESP_LOGD(TAG, "FIFO overflow interrupt triggered");
        return ret;
    } else {
        ESP_LOGD(TAG, "FIFO overflow interrupt not triggered");
        return ESP_ERR_INVALID_STATE;
    }
}

esp_err_t MPU6050::setupFIFO(MPU6050UserControl userCtrl, MPU6050FIFOEnable fifoEn) {
    ESP_LOGD(TAG, "Setting up FIFO");
    esp_err_t ret = writeRegister(MPU6050Register::USER_CTRL, static_cast<uint8_t>(userCtrl));
    if (ret == ESP_OK) {
        ret = writeRegister(MPU6050Register::FIFO_EN, static_cast<uint8_t>(fifoEn));
    }
    return ret;
}

esp_err_t MPU6050::readFromFifo(uint8_t* count_buf, uint8_t* fifo_data) {
    esp_err_t ret = readRegisters(MPU6050Register::FIFO_COUNT_H, count_buf, 2);
    uint16_t fifo_count = (count_buf[0] << 8) | count_buf[1];

    if (fifo_count > 1024) {
        ESP_LOGW(TAG, "FIFO overflow, resetting");
        ret = writeRegister(MPU6050Register::USER_CTRL, static_cast<uint8_t>(MPU6050UserControl::FIFO_RESET));
    } else if (fifo_count > 0) {
        ret = readRegisters(MPU6050Register::FIFO_R_W, fifo_data, fifo_count);
    }
    return ret;
}

esp_err_t MPU6050::setGyroRange(MPU6050GyroConfig range) {
    ESP_LOGD(TAG, "Setting gyroscope range");
    esp_err_t ret = writeRegister(MPU6050Register::GYRO_CONFIG, static_cast<uint8_t>(range));
    if (ret == ESP_OK) {
        switch(range) {
            case MPU6050GyroConfig::RANGE_250_DEG: _gyro_scale = 131.0f; break;
            case MPU6050GyroConfig::RANGE_500_DEG: _gyro_scale = 65.5f; break;
            case MPU6050GyroConfig::RANGE_1000_DEG: _gyro_scale = 32.8f; break;
            case MPU6050GyroConfig::RANGE_2000_DEG: _gyro_scale = 16.4f; break;
        }
    }
    return ret;
}
esp_err_t MPU6050::getAcceleration(float& ax, float& ay, float& az) const {
    uint8_t data[6];
    esp_err_t ret = readRegisters(MPU6050Register::ACCEL_XOUT_H, data, 6);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read acceleration data: %s", esp_err_to_name(ret));
        return ret;
    }

    int16_t raw_ax = (data[0] << 8) | data[1];
    int16_t raw_ay = (data[2] << 8) | data[3];
    int16_t raw_az = (data[4] << 8) | data[5];

    ax = raw_ax / _accel_scale; 
    ay = raw_ay / _accel_scale;
    az = raw_az / _accel_scale;

    ESP_LOGV(TAG, "Acceleration: X=%.2f, Y=%.2f, Z=%.2f", ax, ay, az);
    return ESP_OK;
}



esp_err_t MPU6050::getRotation(float& gx, float& gy, float& gz) const {
    uint8_t data[6];
    esp_err_t ret = readRegisters(MPU6050Register::GYRO_XOUT_H, data, 6);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read gyroscope data: %s", esp_err_to_name(ret));
        return ret;
    }

    int16_t raw_gx = (data[0] << 8) | data[1];
    int16_t raw_gy = (data[2] << 8) | data[3];
    int16_t raw_gz = (data[4] << 8) | data[5];

    gx = raw_gx / _gyro_scale; 
    gy = raw_gy / _gyro_scale;
    gz = raw_gz / _gyro_scale;

    ESP_LOGV(TAG, "Rotation: X=%.2f, Y=%.2f, Z=%.2f", gx, gy, gz);
    return ESP_OK;
}

esp_err_t MPU6050::writeRegister(MPU6050Register reg, uint8_t data) {
    uint8_t write_buf[2] = {static_cast<uint8_t>(reg), data};
    esp_err_t ret = i2c_master_transmit(_dev_handle, write_buf, sizeof(write_buf), -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write register 0x%02X: %s", static_cast<uint8_t>(reg), esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t MPU6050::readRegisters(MPU6050Register reg, uint8_t* data, size_t len) const {
    uint8_t reg_addr = static_cast<uint8_t>(reg);
    esp_err_t ret = i2c_master_transmit_receive(_dev_handle, &reg_addr, 1, data, len, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read register 0x%02X: %s", reg_addr, esp_err_to_name(ret));
    }
    return ret;
}