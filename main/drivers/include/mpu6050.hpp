// main/include/mpu6050.hpp
#pragma once

      
#include "driver/i2c_master.h"
#include "esp_log.h"
#include <stdint.h>
#include <mutex> // <<< ADDED for I2C mutex
    

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
    DATA_READY_FIFO_OVERFLOW = 0x11 // Note: This is a combination bitmask
};

enum class MPU6050InterruptPinConfig : uint8_t {
    ACTIVE_HIGH = 0x00, // INT_LEVEL = 0, INT_OPEN = 0, LATCH_INT_EN = 0, INT_RD_CLEAR = 0
    ACTIVE_LOW = 0x80   // INT_LEVEL = 1, other bits 0
    // Add other configs like push-pull, latching if needed
};

enum class MPU6050UserControl : uint8_t {
    FIFO_RESET = 0x04,
    I2C_MST_RESET = 0x02, // Added for completeness
    SIG_COND_RESET = 0x01, // Added for completeness
    FIFO_ENABLE = 0x40,
    I2C_MST_EN = 0x20, // Added for completeness
    I2C_IF_DIS = 0x10, // Added for completeness
    FIFO_RESET_ENABLE = FIFO_RESET | FIFO_ENABLE // Combine flags
};

enum class MPU6050FIFOEnable : uint8_t {
    TEMP_OUT = 0x80,
    XG_OUT = 0x40,
    YG_OUT = 0x20,
    ZG_OUT = 0x10,
    ACCEL_OUT = 0x08,
    SLV2_OUT = 0x04, // Added for completeness
    SLV1_OUT = 0x02, // Added for completeness
    SLV0_OUT = 0x01, // Added for completeness
    // Common combinations
    GYRO_ACCEL = XG_OUT | YG_OUT | ZG_OUT | ACCEL_OUT, // All gyro axes + accel
    ACCEL_ONLY = ACCEL_OUT,
    GYRO_ONLY = XG_OUT | YG_OUT | ZG_OUT
};

enum class MPU6050PowerManagement : uint8_t {
    RESET = 0x80,
    SLEEP = 0x40,
    CYCLE = 0x20,
    TEMP_DIS = 0x08,
    CLOCK_INTERNAL = 0x00, // Internal 8MHz oscillator
    CLOCK_PLL_XGYRO = 0x01, // Recommended for stability
    CLOCK_PLL_YGYRO = 0x02,
    CLOCK_PLL_ZGYRO = 0x03,
    CLOCK_EXTERNAL_32KHZ = 0x04,
    CLOCK_EXTERNAL_19MHZ = 0x05,
    CLOCK_STOP = 0x07
};

enum class MPU6050AccelConfig : uint8_t {
    RANGE_2G = 0x00, RANGE_4G = 0x08, RANGE_8G = 0x10, RANGE_16G = 0x18
};

enum class MPU6050GyroConfig : uint8_t {
    RANGE_250_DEG = 0x00, RANGE_500_DEG = 0x08, RANGE_1000_DEG = 0x10, RANGE_2000_DEG = 0x18
};

enum class MPU6050DLPFConfig : uint8_t {
    DLPF_BW_260HZ_ACC_256HZ_GYRO = 0x00, // Accel BW 260Hz, Gyro BW 256Hz (FS=8kHz)
    DLPF_BW_184HZ_ACC_188HZ_GYRO = 0x01, // Accel BW 184Hz, Gyro BW 188Hz (FS=1kHz)
    DLPF_BW_94HZ_ACC_98HZ_GYRO = 0x02,   // Accel BW 94Hz,  Gyro BW 98Hz  (FS=1kHz)
    DLPF_BW_44HZ_ACC_42HZ_GYRO = 0x03,   // Accel BW 44Hz,  Gyro BW 42Hz  (FS=1kHz)
    DLPF_BW_21HZ_ACC_20HZ_GYRO = 0x04,   // Accel BW 21Hz,  Gyro BW 20Hz  (FS=1kHz)
    DLPF_BW_10HZ_ACC_10HZ_GYRO = 0x05,   // Accel BW 10Hz,  Gyro BW 10Hz  (FS=1kHz)
    DLPF_BW_5HZ_ACC_5HZ_GYRO = 0x06,     // Accel BW 5Hz,   Gyro BW 5Hz   (FS=1kHz)
    // DLPF_RESERVED = 0x07
};

enum class MPU6050SampleRateDiv : uint8_t {
    // Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
    // Gyroscope Output Rate is 8kHz when DLPF disabled (DLPF_CFG=0 or 7), 1kHz otherwise.
    // Example: To get 1kHz with DLPF enabled (Gyro Rate=1kHz), SMPLRT_DIV = 0
    // Example: To get 200Hz with DLPF enabled (Gyro Rate=1kHz), SMPLRT_DIV = 4
    RATE_1KHZ = 0x00, // If Gyro Rate is 1kHz
    RATE_500HZ = 0x01,
    RATE_250HZ = 0x03,
    RATE_200HZ = 0x04,
    RATE_125HZ = 0x07,
    RATE_100HZ = 0x09
};


class MPU6050 {
public:
    MPU6050();
    ~MPU6050() = default; // Default destructor ok if no manual cleanup needed

    esp_err_t init(i2c_port_t i2c_port, gpio_num_t sda_io, gpio_num_t scl_io,
                   uint16_t i2c_addr = 0x68, uint32_t i2c_freq = 400000);

    // Configuration methods
    esp_err_t setDLPFConfig(MPU6050DLPFConfig config);
    esp_err_t setSampleRate(MPU6050SampleRateDiv rate);
    esp_err_t setAccelRange(MPU6050AccelConfig range);
    esp_err_t setGyroRange(MPU6050GyroConfig range);
    esp_err_t setupInterrupt(MPU6050InterruptPinConfig intPinConfig, MPU6050Interrupt intEnableBits);
    esp_err_t setupFIFO(MPU6050UserControl userCtrlBits, MPU6050FIFOEnable fifoEnableBits);

    // Data reading methods
    esp_err_t getAcceleration(float& ax, float& ay, float& az) const;
    esp_err_t getRotation(float& gx, float& gy, float& gz) const;

    // Status/Helper methods
    float getAccelScale() const { return _accel_scale; } // LSB/(g)
    float getGyroScale() const { return _gyro_scale; }   // LSB/(deg/s)
    esp_err_t isDataReady() const; // Checks DATA_RDY_INT flag
    esp_err_t isFIFOOverflow() const; // Checks FIFO_OFLOW_INT flag

    // --- Moved from private to public ---
    esp_err_t readRegisters(MPU6050Register reg, uint8_t* data, size_t len) const;
    esp_err_t writeRegister(MPU6050Register reg, uint8_t data);
    // --- End moved section ---

    // Removed readFromFifo - higher level logic belongs in IMUService
    // esp_err_t readFromFifo(uint8_t* count_buf, uint8_t* fifo_data);

private:
    static constexpr const char* TAG = "MPU6050";

    i2c_master_dev_handle_t _dev_handle;    
    mutable std::mutex _i2c_mutex; // <<< ADDED Mutex to protect I2C operations
    float _accel_scale = 16384.0f; // Default scale for +/- 2g
    float _gyro_scale = 131.0f;   // Default scale for +/- 250 deg/s


    // Private members removed as methods moved to public
    // esp_err_t writeRegister(MPU6050Register, uint8_t);
    // esp_err_t readRegisters(MPU6050Register, uint8_t*, size_t) const;
};