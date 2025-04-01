#pragma once

#include "driver/i2c_master.h"
#include "esp_log.h"

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
    ACCEL_XOUT_H = 0x3B,
    GYRO_XOUT_H = 0x43
};

enum class MPU6050Interrupt : uint8_t {
    DATA_READY = 0x01,
    FIFO_OVERFLOW = 0x10,
    DATA_READY_FIFO_OVERFLOW = 0x11
};

enum class MPU6050InterruptPinConfig : uint8_t {
    ACTIVE_HIGH = 0x00, // b00000000 - Active high and clear automatically
    ACTIVE_LOW = 0x80  // b10000000 - Active low and clear automatically
};
enum class MPU6050UserControl : uint8_t {
    FIFO_RESET = 0x04,
    FIFO_ENABLE = 0x40,  //01000000 (bit 6 is fifo enable)
    FIFO_RESET_ENABLE = 0x44, //01000100
};

enum class MPU6050FIFOEnable : uint8_t {
    GYRO_ACCEL = 0x78,  //01111000 (bit 6,5,4 gyro, bit 3 accel)
};

enum class MPU6050PowerManagement : uint8_t {
    CLOCK_INTERNAL = 0x00 // Internal 8MHz oscillator
};

enum class MPU6050AccelConfig : uint8_t {
    RANGE_2G = 0x00,  // ± 2g
    RANGE_4G = 0x08,  // ± 4g
    RANGE_8G = 0x10,  // ± 8g
    RANGE_16G = 0x18  // ± 16g
};

enum class MPU6050GyroConfig : uint8_t {
    RANGE_250_DEG = 0x00,  // ± 250 °/s
    RANGE_500_DEG = 0x08,  // ± 500 °/s
    RANGE_1000_DEG = 0x10, // ± 1000 °/s
    RANGE_2000_DEG = 0x18  // ± 2000 °/s
};

enum class MPU6050DLPFConfig : uint8_t {
    DLPF_OFF = 0x00,                     // Disables DLPF, 8kHz sampling rate
    DLPF_BW_184HZ_ACC_188HZ_GYRO = 0x01, // 184 Hz Acc, 188 Hz Gyro
    DLPF_BW_94HZ_ACC_98HZ_GYRO = 0x02,   // 94 Hz Acc, 98 Hz Gyro
    DLPF_BW_44HZ_ACC_42HZ_GYRO = 0x03,   // 44 Hz Acc, 42 Hz Gyro (Recommended for balancing)
    DLPF_BW_21HZ_ACC_20HZ_GYRO = 0x04,   // 21 Hz Acc, 20 Hz Gyro
    DLPF_BW_10HZ_ACC_10HZ_GYRO = 0x05,   // 10 Hz Acc, 10 Hz Gyro
    DLPF_BW_5HZ_ACC_5HZ_GYRO = 0x06,     // 5 Hz Acc, 5 Hz Gyro
};

enum class MPU6050SampleRateDiv : uint8_t {
    RATE_1KHZ = 0x00,  // 1 kHz sampling rate
    RATE_500HZ = 0x01, // 500 Hz sampling rate
    RATE_250HZ = 0x03, // 250 Hz sampling rate
    RATE_125HZ = 0x07, // 125 Hz sampling rate
};

class MPU6050 {
public:
    MPU6050();
    esp_err_t init(const i2c_port_t, 
                    const gpio_num_t, 
                    const gpio_num_t,
                    const uint16_t,
                    const uint32_t);
                    
    float getAccelScale() const { return _accel_scale; }
    float getGyroScale() const { return _gyro_scale; }
    esp_err_t getAcceleration(float& , float&, float&) const;
    esp_err_t getRotation(float&, float&, float&) const;
    esp_err_t readFromFifo(uint8_t*, uint8_t*);

    esp_err_t isDataReady() const;
    esp_err_t isFIFOOverflow() const;

    esp_err_t setDLPFConfig(MPU6050DLPFConfig);
    esp_err_t setSampleRate(MPU6050SampleRateDiv);
    esp_err_t setAccelRange(MPU6050AccelConfig);
    esp_err_t setGyroRange(MPU6050GyroConfig);
    esp_err_t setupInterrupt(MPU6050InterruptPinConfig, MPU6050Interrupt);
    esp_err_t setupFIFO(MPU6050UserControl, MPU6050FIFOEnable);

private:
    static constexpr const char* TAG = "MPU6050";

    i2c_master_dev_handle_t _dev_handle;
    float _accel_scale;
    float _gyro_scale;

    esp_err_t writeRegister(MPU6050Register, uint8_t);
    esp_err_t readRegisters(MPU6050Register, uint8_t*, size_t) const;
};
