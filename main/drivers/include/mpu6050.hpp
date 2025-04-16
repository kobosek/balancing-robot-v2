#pragma once

#include "driver/i2c_master.h"
#include "esp_log.h"
#include <stdint.h>
#include <mutex>
#include "esp_err.h" // Include esp_err_t

// Enums remain the same...
enum class MPU6050Register : uint8_t {
    PWR_MGMT_1 = 0x6B, SMPLRT_DIV = 0x19, USER_CTRL = 0x6A, FIFO_EN = 0x23,
    INT_PIN_CFG = 0x37, INTERRUPT_EN = 0x38, INTERRUPT_STATUS = 0x3A,
    DLPF_CONFIG = 0x1A, GYRO_CONFIG = 0x1B, ACCEL_CONFIG = 0x1C,
    FIFO_COUNT_H = 0x72, FIFO_R_W = 0x74,
    ACCEL_XOUT_H = 0x3B, GYRO_XOUT_H = 0x43,
    WHO_AM_I = 0x75,
    // Add other registers if needed
};

enum class MPU6050Interrupt : uint8_t {
    DATA_READY = 0x01,
    FIFO_OVERFLOW = 0x10,
    // Add other interrupt bits if needed
};

enum class MPU6050InterruptPinConfig : uint8_t {
    ACTIVE_HIGH = 0x00, // INT_LEVEL = 0, INT_OPEN = 0, LATCH_INT_EN = 0, INT_RD_CLEAR = 0
    ACTIVE_LOW = 0x80   // INT_LEVEL = 1, other bits 0
    // Add other configs like push-pull, latching if needed
};

enum class MPU6050UserControl : uint8_t {
    FIFO_RESET = 0x04,
    I2C_MST_RESET = 0x02,
    SIG_COND_RESET = 0x01,
    FIFO_ENABLE = 0x40,
    I2C_MST_EN = 0x20,
    I2C_IF_DIS = 0x10,
    FIFO_RESET_ENABLE = FIFO_RESET | FIFO_ENABLE // Combine flags
};

enum class MPU6050FIFOEnable : uint8_t {
    TEMP_OUT = 0x80, XG_OUT = 0x40, YG_OUT = 0x20, ZG_OUT = 0x10, ACCEL_OUT = 0x08,
    SLV2_OUT = 0x04, SLV1_OUT = 0x02, SLV0_OUT = 0x01,
    GYRO_ACCEL = XG_OUT | YG_OUT | ZG_OUT | ACCEL_OUT,
    ACCEL_ONLY = ACCEL_OUT,
    GYRO_ONLY = XG_OUT | YG_OUT | ZG_OUT
};

enum class MPU6050PowerManagement : uint8_t {
    RESET = 0x80, SLEEP = 0x40, CYCLE = 0x20, TEMP_DIS = 0x08,
    CLOCK_INTERNAL = 0x00, CLOCK_PLL_XGYRO = 0x01, CLOCK_PLL_YGYRO = 0x02,
    CLOCK_PLL_ZGYRO = 0x03, CLOCK_EXTERNAL_32KHZ = 0x04, CLOCK_EXTERNAL_19MHZ = 0x05,
    CLOCK_STOP = 0x07
};

enum class MPU6050AccelConfig : uint8_t {
    RANGE_2G = 0x00, RANGE_4G = 0x08, RANGE_8G = 0x10, RANGE_16G = 0x18
};

enum class MPU6050GyroConfig : uint8_t {
    RANGE_250_DEG = 0x00, RANGE_500_DEG = 0x08, RANGE_1000_DEG = 0x10, RANGE_2000_DEG = 0x18
};

enum class MPU6050DLPFConfig : uint8_t {
    DLPF_BW_260HZ_ACC_256HZ_GYRO = 0x00, DLPF_BW_184HZ_ACC_188HZ_GYRO = 0x01,
    DLPF_BW_94HZ_ACC_98HZ_GYRO = 0x02, DLPF_BW_44HZ_ACC_42HZ_GYRO = 0x03,
    DLPF_BW_21HZ_ACC_20HZ_GYRO = 0x04, DLPF_BW_10HZ_ACC_10HZ_GYRO = 0x05,
    DLPF_BW_5HZ_ACC_5HZ_GYRO = 0x06
};

enum class MPU6050SampleRateDiv : uint8_t {
    RATE_1KHZ = 0x00, RATE_500HZ = 0x01, RATE_250HZ = 0x03, RATE_200HZ = 0x04,
    RATE_125HZ = 0x07, RATE_100HZ = 0x09
};


class MPU6050Driver {
public:
    MPU6050Driver();
    ~MPU6050Driver() = default;

    // --- Initialization ---
    esp_err_t init(i2c_port_t i2c_port, gpio_num_t sda_io, gpio_num_t scl_io,
                   uint16_t i2c_addr = 0x68, uint32_t i2c_freq = 400000);

    // --- Configuration Methods (Write to Registers) ---
    esp_err_t setDLPFConfigReg(MPU6050DLPFConfig config);
    esp_err_t setSampleRateDivReg(MPU6050SampleRateDiv rate_div);
    esp_err_t setAccelRangeReg(MPU6050AccelConfig range);
    esp_err_t setGyroRangeReg(MPU6050GyroConfig range);
    esp_err_t configureInterruptPinReg(MPU6050InterruptPinConfig intPinConfig, MPU6050Interrupt intEnableBits);
    esp_err_t configureFIFOReg(MPU6050UserControl userCtrlBits, MPU6050FIFOEnable fifoEnableBits);
    esp_err_t setPowerManagementReg(MPU6050PowerManagement powerBits);
    esp_err_t resetSensor(); // Specific method for reset sequence

    // --- Raw Data Reading Methods ---
    esp_err_t readRawAccelXYZ(int16_t& ax, int16_t& ay, int16_t& az) const;
    esp_err_t readRawGyroXYZ(int16_t& gx, int16_t& gy, int16_t& gz) const;

    // --- FIFO and Status Methods ---
    esp_err_t readFifoCount(uint16_t& count) const;
    esp_err_t readFifoBuffer(uint8_t* buffer, size_t len) const;
    esp_err_t readInterruptStatus(uint8_t& status) const;
    esp_err_t isFIFOOverflow(bool& isOverflow) const; // Returns bool via parameter

    // --- Low-level I2C Communication (Public for flexibility/other services) ---
    esp_err_t readRegisters(MPU6050Register reg, uint8_t* data, size_t len) const;
    esp_err_t writeRegister(MPU6050Register reg, uint8_t data);

private:
    static constexpr const char* TAG = "MPU6050Driver";

    i2c_master_dev_handle_t _dev_handle;
    mutable std::mutex _i2c_mutex; // Protect I2C operations
};