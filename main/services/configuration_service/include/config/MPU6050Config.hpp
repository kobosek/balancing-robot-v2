#pragma once

#include <cstdint>

struct MPU6050Config {
    int i2c_port = 0;
    int sda_pin = 7;
    int scl_pin = 8;
    uint8_t device_address = 0x68;
    uint32_t i2c_freq_hz = 400000;

    int int_pin = 9;
    bool interrupt_active_high = true;

    int accel_range = 1;
    int gyro_range = 1;
    int dlpf_config = 3;
    int sample_rate_divisor = 0;

    int calibration_samples = 1000;
    float comp_filter_alpha = 0.98f;
    int fifo_read_threshold = 10;

    float gyro_offset_x = 0.0f;
    float gyro_offset_y = 0.0f;
    float gyro_offset_z = 0.0f;

    bool operator!=(const MPU6050Config& other) const {
        return i2c_port != other.i2c_port ||
               sda_pin != other.sda_pin ||
               scl_pin != other.scl_pin ||
               device_address != other.device_address ||
               i2c_freq_hz != other.i2c_freq_hz ||
               int_pin != other.int_pin ||
               interrupt_active_high != other.interrupt_active_high ||
               accel_range != other.accel_range ||
               gyro_range != other.gyro_range ||
               dlpf_config != other.dlpf_config ||
               sample_rate_divisor != other.sample_rate_divisor ||
               calibration_samples != other.calibration_samples ||
               comp_filter_alpha != other.comp_filter_alpha ||
               fifo_read_threshold != other.fifo_read_threshold ||
               gyro_offset_x != other.gyro_offset_x ||
               gyro_offset_y != other.gyro_offset_y ||
               gyro_offset_z != other.gyro_offset_z;
    }

    bool operator==(const MPU6050Config& other) const {
        return !(*this != other);
    }

    bool requiresHardwareInit(const MPU6050Config& other) const {
        return i2c_port != other.i2c_port ||
               sda_pin != other.sda_pin ||
               scl_pin != other.scl_pin ||
               device_address != other.device_address ||
               i2c_freq_hz != other.i2c_freq_hz ||
               int_pin != other.int_pin ||
               interrupt_active_high != other.interrupt_active_high ||
               accel_range != other.accel_range ||
               gyro_range != other.gyro_range ||
               dlpf_config != other.dlpf_config ||
               sample_rate_divisor != other.sample_rate_divisor;
    }
};
