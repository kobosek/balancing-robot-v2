#include "mpu6050.hpp"

#include "I2CDevice.hpp"

MPU6050Driver::MPU6050Driver(I2CDevice& device) :
    m_device(device) {}

esp_err_t MPU6050Driver::setPowerManagementReg(MPU6050PowerManagement powerBits) {
    return writeRegister(MPU6050Register::PWR_MGMT_1, static_cast<uint8_t>(powerBits));
}

esp_err_t MPU6050Driver::resetSensor() {
    return writeRegister(MPU6050Register::PWR_MGMT_1, static_cast<uint8_t>(MPU6050PowerManagement::RESET));
}

esp_err_t MPU6050Driver::setDLPFConfigReg(MPU6050DLPFConfig config) {
    return writeRegister(MPU6050Register::DLPF_CONFIG, static_cast<uint8_t>(config));
}

esp_err_t MPU6050Driver::setSampleRateDivReg(MPU6050SampleRateDiv rateDiv) {
    return writeRegister(MPU6050Register::SMPLRT_DIV, static_cast<uint8_t>(rateDiv));
}

esp_err_t MPU6050Driver::setAccelRangeReg(MPU6050AccelConfig range) {
    return writeRegister(MPU6050Register::ACCEL_CONFIG, static_cast<uint8_t>(range));
}

esp_err_t MPU6050Driver::setGyroRangeReg(MPU6050GyroConfig range) {
    return writeRegister(MPU6050Register::GYRO_CONFIG, static_cast<uint8_t>(range));
}

esp_err_t MPU6050Driver::configureInterruptPinReg(MPU6050InterruptPinConfig intPinConfig,
                                                  MPU6050Interrupt intEnableBits) {
    esp_err_t ret = writeRegister(MPU6050Register::INT_PIN_CFG, static_cast<uint8_t>(intPinConfig));
    if (ret != ESP_OK) {
        return ret;
    }

    return writeRegister(MPU6050Register::INTERRUPT_EN, static_cast<uint8_t>(intEnableBits));
}

esp_err_t MPU6050Driver::configureFIFOReg(MPU6050UserControl userCtrlBits, MPU6050FIFOEnable fifoEnableBits) {
    esp_err_t ret = writeRegister(MPU6050Register::USER_CTRL, static_cast<uint8_t>(userCtrlBits));
    if (ret != ESP_OK) {
        return ret;
    }

    if ((static_cast<uint8_t>(userCtrlBits) & static_cast<uint8_t>(MPU6050UserControl::FIFO_ENABLE)) == 0) {
        return ESP_OK;
    }

    return writeRegister(MPU6050Register::FIFO_EN, static_cast<uint8_t>(fifoEnableBits));
}

esp_err_t MPU6050Driver::readRawGyroXYZ(int16_t& gx, int16_t& gy, int16_t& gz) const {
    uint8_t data[6] = {0};
    esp_err_t ret = readRegisters(MPU6050Register::GYRO_XOUT_H, data, sizeof(data));
    if (ret != ESP_OK) {
        return ret;
    }

    gx = static_cast<int16_t>((data[0] << 8) | data[1]);
    gy = static_cast<int16_t>((data[2] << 8) | data[3]);
    gz = static_cast<int16_t>((data[4] << 8) | data[5]);
    return ESP_OK;
}

esp_err_t MPU6050Driver::readFifoCount(uint16_t& count) const {
    uint8_t data[2] = {0};
    esp_err_t ret = readRegisters(MPU6050Register::FIFO_COUNT_H, data, sizeof(data));
    if (ret != ESP_OK) {
        count = 0;
        return ret;
    }

    count = static_cast<uint16_t>((data[0] << 8) | data[1]);
    return ESP_OK;
}

esp_err_t MPU6050Driver::readFifoBuffer(uint8_t* buffer, size_t len) const {
    if (len == 0) {
        return ESP_OK;
    }
    if (buffer == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    return readRegisters(MPU6050Register::FIFO_R_W, buffer, len);
}

esp_err_t MPU6050Driver::getInterruptStatus(uint8_t& status) const {
    return readRegisters(MPU6050Register::INTERRUPT_STATUS, &status, 1);
}

esp_err_t MPU6050Driver::isFIFOOverflow(bool& isOverflow) const {
    uint8_t status = 0;
    esp_err_t ret = getInterruptStatus(status);
    if (ret != ESP_OK) {
        isOverflow = false;
        return ret;
    }

    isOverflow = (status & static_cast<uint8_t>(MPU6050Interrupt::FIFO_OVERFLOW)) != 0;
    return ESP_OK;
}

esp_err_t MPU6050Driver::getDeviceID(uint8_t& id) const {
    return readRegisters(MPU6050Register::WHO_AM_I, &id, 1);
}

esp_err_t MPU6050Driver::disableFIFO() {
    esp_err_t ret = writeRegister(MPU6050Register::FIFO_EN, 0);
    if (ret != ESP_OK) {
        return ret;
    }

    return writeRegister(MPU6050Register::USER_CTRL, 0);
}

esp_err_t MPU6050Driver::resetFIFO() {
    return writeRegister(MPU6050Register::USER_CTRL, static_cast<uint8_t>(MPU6050UserControl::FIFO_RESET));
}

esp_err_t MPU6050Driver::resetSignalPath() {
    return writeRegister(MPU6050Register::USER_CTRL, static_cast<uint8_t>(MPU6050UserControl::SIG_COND_RESET));
}

esp_err_t MPU6050Driver::performFullFIFOReset() {
    esp_err_t ret = disableFIFO();
    if (ret != ESP_OK) {
        return ret;
    }

    ret = resetFIFO();
    if (ret != ESP_OK) {
        return ret;
    }

    return resetSignalPath();
}

esp_err_t MPU6050Driver::readRegisters(MPU6050Register reg, uint8_t* data, size_t len) const {
    return m_device.readRegisters(static_cast<uint8_t>(reg), data, len);
}

esp_err_t MPU6050Driver::writeRegister(MPU6050Register reg, uint8_t data) {
    return m_device.writeRegister(static_cast<uint8_t>(reg), data);
}
