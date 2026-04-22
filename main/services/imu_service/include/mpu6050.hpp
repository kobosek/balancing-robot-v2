#pragma once

#include "esp_err.h"
#include <cstddef>
#include <cstdint>

enum class MPU6050Register : uint8_t {
    PWR_MGMT_1 = 0x6B,
    SMPLRT_DIV = 0x19,
    USER_CTRL = 0x6A,
    FIFO_EN = 0x23,
    INT_PIN_CFG = 0x37,
    INTERRUPT_EN = 0x38,
    INTERRUPT_STATUS = 0x3A,
    DLPF_CONFIG = 0x1A,
    GYRO_CONFIG = 0x1B,
    ACCEL_CONFIG = 0x1C,
    FIFO_COUNT_H = 0x72,
    FIFO_R_W = 0x74,
    GYRO_XOUT_H = 0x43,
    WHO_AM_I = 0x75
};

enum class MPU6050Interrupt : uint8_t {
    DATA_READY = 0x01,
    FIFO_OVERFLOW = 0x10,
};

enum class MPU6050InterruptPinConfig : uint8_t {
    ACTIVE_HIGH = 0x00,
    ACTIVE_LOW = 0x80,
    ACTIVE_HIGH_LATCH_CLEAR_ON_ANY_READ = 0x30,
    ACTIVE_LOW_LATCH_CLEAR_ON_ANY_READ = 0xB0
};

enum class MPU6050UserControl : uint8_t {
    FIFO_RESET = 0x04,
    SIG_COND_RESET = 0x01,
    FIFO_ENABLE = 0x40,
};

enum class MPU6050FIFOEnable : uint8_t {
    TEMP_OUT = 0x80,
    XG_OUT = 0x40,
    YG_OUT = 0x20,
    ZG_OUT = 0x10,
    ACCEL_OUT = 0x08,
    GYRO_ACCEL = XG_OUT | YG_OUT | ZG_OUT | ACCEL_OUT
};

enum class MPU6050PowerManagement : uint8_t {
    RESET = 0x80,
    CLOCK_INTERNAL = 0x00,
    CLOCK_PLL_X_GYRO = 0x01
};

enum class MPU6050AccelConfig : uint8_t {
    RANGE_2G = 0x00,
    RANGE_4G = 0x08,
    RANGE_8G = 0x10,
    RANGE_16G = 0x18
};

enum class MPU6050GyroConfig : uint8_t {
    RANGE_250_DEG = 0x00,
    RANGE_500_DEG = 0x08,
    RANGE_1000_DEG = 0x10,
    RANGE_2000_DEG = 0x18
};

enum class MPU6050DLPFConfig : uint8_t {
    DLPF_BW_260HZ_ACC_256HZ_GYRO = 0x00,
    DLPF_BW_184HZ_ACC_188HZ_GYRO = 0x01,
    DLPF_BW_94HZ_ACC_98HZ_GYRO = 0x02,
    DLPF_BW_44HZ_ACC_42HZ_GYRO = 0x03,
    DLPF_BW_21HZ_ACC_20HZ_GYRO = 0x04,
    DLPF_BW_10HZ_ACC_10HZ_GYRO = 0x05,
    DLPF_BW_5HZ_ACC_5HZ_GYRO = 0x06
};

enum class MPU6050SampleRateDiv : uint8_t {
    RATE_1KHZ = 0x00,
    RATE_500HZ = 0x01,
    RATE_250HZ = 0x03,
    RATE_200HZ = 0x04,
    RATE_125HZ = 0x07,
    RATE_100HZ = 0x09
};

class I2CDevice;

class MPU6050Driver {
public:
    explicit MPU6050Driver(I2CDevice& device);

    esp_err_t setPowerManagementReg(MPU6050PowerManagement powerBits);
    esp_err_t resetSensor();
    esp_err_t setDLPFConfigReg(MPU6050DLPFConfig config);
    esp_err_t setSampleRateDivReg(MPU6050SampleRateDiv rateDiv);
    esp_err_t setAccelRangeReg(MPU6050AccelConfig range);
    esp_err_t setGyroRangeReg(MPU6050GyroConfig range);
    esp_err_t configureInterruptPinReg(MPU6050InterruptPinConfig intPinConfig, MPU6050Interrupt intEnableBits);
    esp_err_t configureFIFOReg(MPU6050UserControl userCtrlBits, MPU6050FIFOEnable fifoEnableBits);

    esp_err_t readRawGyroXYZ(int16_t& gx, int16_t& gy, int16_t& gz) const;
    esp_err_t readFifoCount(uint16_t& count) const;
    esp_err_t readFifoBuffer(uint8_t* buffer, size_t len) const;
    esp_err_t getInterruptStatus(uint8_t& status) const;
    esp_err_t isFIFOOverflow(bool& isOverflow) const;
    esp_err_t getDeviceID(uint8_t& id) const;

    esp_err_t disableFIFO();
    esp_err_t resetFIFO();
    esp_err_t resetSignalPath();
    esp_err_t performFullFIFOReset();

private:
    esp_err_t readRegisters(MPU6050Register reg, uint8_t* data, size_t len) const;
    esp_err_t writeRegister(MPU6050Register reg, uint8_t data);

    I2CDevice& m_device;
};
