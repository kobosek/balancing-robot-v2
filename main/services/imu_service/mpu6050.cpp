#include "mpu6050.hpp"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

MPU6050Driver::MPU6050Driver() :
    _bus_handle(nullptr),
    _dev_handle(nullptr),
    _i2c_mutex(),
    _i2c_config(),
    _i2c_stats()
{}

esp_err_t MPU6050Driver::init(const i2c_port_t i2c_port,
    const gpio_num_t sda_io,
    const gpio_num_t scl_io,
    const uint16_t i2c_addr,
    const uint32_t i2c_freq) {

    ESP_LOGD(TAG, "Initializing MPU6050 Driver");
    std::lock_guard<std::mutex> lock(_i2c_mutex);

    if (_dev_handle) {
        ESP_LOGW(TAG, "Driver already initialized. Skipping init.");
        return ESP_OK; // Already initialized is not an error here
    }
    // Reset handles before attempting initialization
    _dev_handle = nullptr;
    _bus_handle = nullptr;

    esp_err_t ret = ESP_FAIL; // Default to failure

    // --- Configure I2C Bus ---
    i2c_master_bus_config_t bus_config = {
        .i2c_port = i2c_port,
        .sda_io_num = sda_io,
        .scl_io_num = scl_io,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0, // Or a suitable priority if needed
        .flags = { .enable_internal_pullup = true }
    };
    // Try to create the bus
    ret = i2c_new_master_bus(&bus_config, &_bus_handle);
    if (ret != ESP_OK || !_bus_handle) {
        ESP_LOGE(TAG, "Failed create I2C master bus: %s", esp_err_to_name(ret));
        _bus_handle = nullptr; 
        return ret; 
    }
    ESP_LOGD(TAG, "I2C Master Bus created (Handle: %p)", _bus_handle);

    // --- Add I2C Device ---
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = i2c_addr,
        .scl_speed_hz = i2c_freq,
    };
    // Try to add the device
    ret = i2c_master_bus_add_device(_bus_handle, &dev_config, &_dev_handle);
    if (ret != ESP_OK || !_dev_handle) {
        ESP_LOGE(TAG, "Failed add I2C device: %s", esp_err_to_name(ret));
        _dev_handle = nullptr; // Ensure null on failure
        // DO NOT delete bus or device here, let destructor handle cleanup of _bus_handle if it exists
        return ret == ESP_OK ? ESP_FAIL : ret; // Return error from device add, or ESP_FAIL if ret was OK but handle null
    }

    ESP_LOGI(TAG, "MPU6050 Driver Initialized Successfully (Bus Handle: %p, Device Handle: %p)", _bus_handle, _dev_handle);
    return ESP_OK; // Explicitly return OK only if both steps succeeded
}

MPU6050Driver::~MPU6050Driver() {
    ESP_LOGI(TAG, "Deinitializing MPU6050 Driver...");
    std::lock_guard<std::mutex> lock(_i2c_mutex); // Lock during deinit

    esp_err_t ret;
    // Remove device first (if it exists)
    if (_dev_handle) {
        ret = i2c_master_bus_rm_device(_dev_handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to remove I2C device: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGD(TAG, "I2C device removed.");
        }
        _dev_handle = nullptr;
    }

    if (_bus_handle) {
        ret = i2c_del_master_bus(_bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to delete I2C master bus: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGD(TAG, "I2C master bus deleted.");
    }
    _bus_handle = nullptr;
    }

    ESP_LOGI(TAG, "MPU6050 Driver Deinitialized.");
}


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
     return ESP_OK; 
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
    esp_err_t ret = writeRegister(MPU6050Register::USER_CTRL, static_cast<uint8_t>(userCtrlBits));
    if (ret == ESP_OK) {
        if(static_cast<uint8_t>(userCtrlBits) & static_cast<uint8_t>(MPU6050UserControl::FIFO_ENABLE)) {
             ret = writeRegister(MPU6050Register::FIFO_EN, static_cast<uint8_t>(fifoEnableBits));
             if (ret != ESP_OK) { ESP_LOGE(TAG, "Failed to write FIFO_EN register: %s", esp_err_to_name(ret)); }
        }
    } else {
         ESP_LOGE(TAG, "Failed to write USER_CTRL register: %s", esp_err_to_name(ret));
    }
    return ret;
}

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

esp_err_t MPU6050Driver::getInterruptStatus(uint8_t& status) const {
     esp_err_t ret = readRegisters(MPU6050Register::INTERRUPT_STATUS, &status, 1);
      if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to read interrupt status: %s", esp_err_to_name(ret));
     } else {
         ESP_LOGV(TAG, "Read Interrupt Status: 0x%02X", status);
     }
     return ret;
}

esp_err_t MPU6050Driver::isDataReady(bool& isDataReady) const {
     uint8_t status = 0;
     isDataReady = false; // Default
     esp_err_t ret = getInterruptStatus(status);
     if (ret == ESP_OK) {
         isDataReady = (status & static_cast<uint8_t>(MPU6050Interrupt::DATA_READY));
         if (isDataReady) {
             ESP_LOGV(TAG, "Data Ready Flag SET in Interrupt Status (0x%02X)", status);
         } else {
              ESP_LOGV(TAG, "Data Ready Flag NOT SET in Interrupt Status (0x%02X)", status);
         }
     }
     return ret;
}

esp_err_t MPU6050Driver::isFIFOOverflow(bool& isOverflow) const {
     uint8_t status = 0;
     isOverflow = false; // Default
     esp_err_t ret = getInterruptStatus(status);
     if (ret == ESP_OK) {
         isOverflow = (status & static_cast<uint8_t>(MPU6050Interrupt::FIFO_OVERFLOW));
         if (isOverflow) {
             ESP_LOGW(TAG, "FIFO Overflow Flag SET in Interrupt Status (0x%02X)", status);
         } else {
              ESP_LOGV(TAG, "FIFO Overflow Flag NOT SET in Interrupt Status (0x%02X)", status);
         }
     }
     return ret;
}

esp_err_t MPU6050Driver::writeRegister(MPU6050Register reg, uint8_t data) {
    return writeRegisterWithRetry(reg, data);
}

esp_err_t MPU6050Driver::readRegisters(MPU6050Register reg, uint8_t* data, size_t len) const {
    return readRegistersWithRetry(reg, data, len);
}

// --- New methods for encapsulated functionality ---

esp_err_t MPU6050Driver::enableFIFO(MPU6050FIFOEnable fifoEnableBits) {
    ESP_LOGD(TAG, "Enabling FIFO with sensor bits: 0x%02X", static_cast<uint8_t>(fifoEnableBits));
    // First set user control to enable FIFO
    esp_err_t ret = writeRegister(MPU6050Register::USER_CTRL, 
                                 static_cast<uint8_t>(MPU6050UserControl::FIFO_ENABLE));
    if (ret != ESP_OK) return ret;
    
    // Then enable specific sensors for FIFO
    return writeRegister(MPU6050Register::FIFO_EN, static_cast<uint8_t>(fifoEnableBits));
}

esp_err_t MPU6050Driver::disableFIFO() {
    ESP_LOGD(TAG, "Disabling FIFO");
    // First clear FIFO_EN register (disable all sensors)
    esp_err_t ret = writeRegister(MPU6050Register::FIFO_EN, 0);
    if (ret != ESP_OK) return ret;
    
    // Then clear FIFO_ENABLE bit in USER_CTRL
    return writeRegister(MPU6050Register::USER_CTRL, 0);
}

esp_err_t MPU6050Driver::resetFIFO() {
    ESP_LOGD(TAG, "Resetting FIFO");
    // Set FIFO_RESET bit in USER_CTRL
    return writeRegister(MPU6050Register::USER_CTRL, 
                        static_cast<uint8_t>(MPU6050UserControl::FIFO_RESET));
}

esp_err_t MPU6050Driver::resetSignalPath() {
    ESP_LOGD(TAG, "Resetting signal path");
    return writeRegister(MPU6050Register::USER_CTRL, 
                        static_cast<uint8_t>(MPU6050UserControl::SIG_COND_RESET));
}

esp_err_t MPU6050Driver::clearAllUserControlBits() {
    ESP_LOGD(TAG, "Clearing all USER_CTRL bits");
    return writeRegister(MPU6050Register::USER_CTRL, 0);
}

esp_err_t MPU6050Driver::performFullFIFOReset() {
    ESP_LOGD(TAG, "Performing full FIFO reset sequence");
    // First disable FIFO
    esp_err_t ret = disableFIFO();
    if (ret != ESP_OK) return ret;
    
    // Reset FIFO
    ret = resetFIFO();
    if (ret != ESP_OK) return ret;
    
    // Reset signal paths
    ret = resetSignalPath();
    if (ret != ESP_OK) return ret;
    
    return ESP_OK;
}

esp_err_t MPU6050Driver::setUserControlBits(MPU6050UserControl bits) {
    ESP_LOGD(TAG, "Setting USER_CTRL bits: 0x%02X", static_cast<uint8_t>(bits));
    uint8_t current_value = 0;
    esp_err_t ret = readRegisters(MPU6050Register::USER_CTRL, &current_value, 1);
    if (ret != ESP_OK) return ret;
    
    // Set new bits while preserving others
    uint8_t new_value = current_value | static_cast<uint8_t>(bits);
    return writeRegister(MPU6050Register::USER_CTRL, new_value);
}

esp_err_t MPU6050Driver::clearUserControlBits(MPU6050UserControl bits) {
    ESP_LOGD(TAG, "Clearing USER_CTRL bits: 0x%02X", static_cast<uint8_t>(bits));
    uint8_t current_value = 0;
    esp_err_t ret = readRegisters(MPU6050Register::USER_CTRL, &current_value, 1);
    if (ret != ESP_OK) return ret;
    
    // Clear specified bits while preserving others
    uint8_t new_value = current_value & ~static_cast<uint8_t>(bits);
    return writeRegister(MPU6050Register::USER_CTRL, new_value);
}

esp_err_t MPU6050Driver::getDeviceID(uint8_t& id) {
    ESP_LOGD(TAG, "Reading device ID (WHO_AM_I register)");
    return readRegisters(MPU6050Register::WHO_AM_I, &id, 1);
}

esp_err_t MPU6050Driver::validateSensorID(uint8_t& id_value) {
    ESP_LOGD(TAG, "Validating sensor ID");
    esp_err_t ret = getDeviceID(id_value);
    if (ret != ESP_OK) return ret;
    
    // Check ID is valid (0x68 = default address, 0x69 = alternate address when AD0 pin is high)
    if (id_value != 0x68 && id_value != 0x69) {
        ESP_LOGE(TAG, "Invalid WHO_AM_I value: 0x%02X (expected 0x68 or 0x69)", id_value);
        return ESP_ERR_INVALID_RESPONSE;
    }
    
    ESP_LOGI(TAG, "Sensor ID validated: 0x%02X", id_value);
    return ESP_OK;
}

esp_err_t MPU6050Driver::writeRegisterMasked(MPU6050Register reg, uint8_t value, uint8_t mask) {
    uint8_t current_value = 0;
    esp_err_t ret = readRegisters(reg, &current_value, 1);
    if (ret != ESP_OK) return ret;
    
    // Clear bits that will be set, then set new bits
    uint8_t new_value = (current_value & ~mask) | (value & mask);
    return writeRegister(reg, new_value);
}

esp_err_t MPU6050Driver::configureForDataAcquisition(
    MPU6050DLPFConfig dlpfConfig,
    MPU6050SampleRateDiv sampleRate,
    MPU6050AccelConfig accelRange,
    MPU6050GyroConfig gyroRange
) {
    ESP_LOGI(TAG, "Configuring MPU6050 for data acquisition");
    
    esp_err_t ret = setDLPFConfigReg(dlpfConfig);
    if (ret != ESP_OK) return ret;
    
    ret = setSampleRateDivReg(sampleRate);
    if (ret != ESP_OK) return ret;
    
    ret = setAccelRangeReg(accelRange);
    if (ret != ESP_OK) return ret;
    
    ret = setGyroRangeReg(gyroRange);
    if (ret != ESP_OK) return ret;
    
    ESP_LOGI(TAG, "MPU6050 configuration complete");
    return ESP_OK;
}

esp_err_t MPU6050Driver::setGyroOffsets(float x_offset_dps, float y_offset_dps, float z_offset_dps) {
    ESP_LOGI(TAG, "Setting gyroscope offsets: X=%.3f, Y=%.3f, Z=%.3f dps", 
             x_offset_dps, y_offset_dps, z_offset_dps);
    
    // Convert from degrees per second to raw register values
    // MPU6050 gyro offset registers use a scale factor of 4 LSB/dps for ±1000dps range
    // The offset registers are 16-bit signed values stored in big-endian format
    const float OFFSET_SCALE_FACTOR = 4.0f; // LSB per dps
    
    int16_t x_offset_raw = (int16_t)(x_offset_dps * OFFSET_SCALE_FACTOR);
    int16_t y_offset_raw = (int16_t)(y_offset_dps * OFFSET_SCALE_FACTOR);
    int16_t z_offset_raw = (int16_t)(z_offset_dps * OFFSET_SCALE_FACTOR);
    
    ESP_LOGD(TAG, "Raw offset values: X=%d, Y=%d, Z=%d", x_offset_raw, y_offset_raw, z_offset_raw);
    
    // Write X-axis gyro offset (high byte then low byte)
    esp_err_t ret = writeRegister(MPU6050Register::XG_OFFS_USRH, (uint8_t)(x_offset_raw >> 8));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write X gyro offset high byte: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = writeRegister(MPU6050Register::XG_OFFS_USRL, (uint8_t)(x_offset_raw & 0xFF));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write X gyro offset low byte: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Write Y-axis gyro offset (high byte then low byte)
    ret = writeRegister(MPU6050Register::YG_OFFS_USRH, (uint8_t)(y_offset_raw >> 8));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write Y gyro offset high byte: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = writeRegister(MPU6050Register::YG_OFFS_USRL, (uint8_t)(y_offset_raw & 0xFF));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write Y gyro offset low byte: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Write Z-axis gyro offset (high byte then low byte)
    ret = writeRegister(MPU6050Register::ZG_OFFS_USRH, (uint8_t)(z_offset_raw >> 8));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write Z gyro offset high byte: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = writeRegister(MPU6050Register::ZG_OFFS_USRL, (uint8_t)(z_offset_raw & 0xFF));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write Z gyro offset low byte: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Gyroscope offsets set successfully");
    return ESP_OK;
}
// --- I2C Configuration and Statistics Methods ---

void MPU6050Driver::setI2CConfig(const I2CConfig& config) {
    std::lock_guard<std::mutex> lock(_i2c_mutex);
    _i2c_config = config;
    ESP_LOGI(TAG, "I2C Config updated: timeout=%lums, max_retries=%d, base_delay=%lums", 
             _i2c_config.timeout_ms, _i2c_config.max_retries, _i2c_config.base_delay_ms);
}

I2CConfig MPU6050Driver::getI2CConfig() const {
    std::lock_guard<std::mutex> lock(_i2c_mutex);
    return _i2c_config;
}

I2CStats MPU6050Driver::getI2CStats() const {
    std::lock_guard<std::mutex> lock(_i2c_mutex);
    return _i2c_stats;
}

void MPU6050Driver::resetI2CStats() {
    std::lock_guard<std::mutex> lock(_i2c_mutex);
    _i2c_stats = I2CStats{};
    ESP_LOGI(TAG, "I2C statistics reset");
}

// --- Enhanced Error Classification and Analysis ---

I2CErrorSeverity MPU6050Driver::getErrorSeverity(I2CErrorType error_type) const {
    return classifyErrorSeverity(error_type);
}

bool MPU6050Driver::isTemporaryError(I2CErrorType error_type) const {
    return classifyErrorSeverity(error_type) == I2CErrorSeverity::TEMPORARY;
}

bool MPU6050Driver::isPermanentError(I2CErrorType error_type) const {
    return classifyErrorSeverity(error_type) == I2CErrorSeverity::PERMANENT;
}

float MPU6050Driver::getCurrentErrorRate() const {
    std::lock_guard<std::mutex> lock(_i2c_mutex);
    return _i2c_stats.current_error_rate_percent;
}

bool MPU6050Driver::isErrorTrendIncreasing() const {
    std::lock_guard<std::mutex> lock(_i2c_mutex);
    return _i2c_stats.error_trend.trend_increasing;
}

// --- Enhanced I2C Communication with Retry Logic ---

esp_err_t MPU6050Driver::writeRegisterWithRetry(MPU6050Register reg, uint8_t data) {
    std::lock_guard<std::mutex> lock(_i2c_mutex);
    
    if (_dev_handle == nullptr) {
        ESP_LOGE(TAG, "Device handle not initialized for writeRegister");
        updateI2CStats(false, I2CErrorType::PERMANENT_FAILURE);
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t write_buf[2] = {static_cast<uint8_t>(reg), data};
    esp_err_t ret = ESP_FAIL;
    I2CErrorType error_type = I2CErrorType::NONE;
    
    for (uint8_t attempt = 0; attempt <= _i2c_config.max_retries; attempt++) {
        _i2c_stats.total_operations++;
        
        // Perform I2C write operation
        ret = i2c_master_transmit(_dev_handle, write_buf, sizeof(write_buf), 
                                 pdMS_TO_TICKS(_i2c_config.timeout_ms));
        
        if (ret == ESP_OK) {
            ESP_LOGV(TAG, "Wrote 0x%02X to reg 0x%02X (attempt %d)", data, static_cast<uint8_t>(reg), attempt + 1);
            updateI2CStats(true, I2CErrorType::NONE);
            return ESP_OK;
        }
        
        // Classify the error
        error_type = classifyI2CError(ret);
        I2CErrorSeverity severity = classifyErrorSeverity(error_type);
        
        ESP_LOGW(TAG, "I2C write failed to reg 0x%02X (attempt %d/%d): %s [Error: %d, Severity: %d]", 
                 static_cast<uint8_t>(reg), attempt + 1, _i2c_config.max_retries + 1, 
                 esp_err_to_name(ret), static_cast<int>(error_type), static_cast<int>(severity));
        
        // Don't retry on permanent failures
        if (severity == I2CErrorSeverity::PERMANENT) {
            ESP_LOGE(TAG, "Permanent I2C failure detected (type: %d), aborting retries", 
                     static_cast<int>(error_type));
            break;
        }
        
        // Don't delay after the last attempt
        if (attempt < _i2c_config.max_retries) {
            // Attempt bus recovery if enabled and this is a recoverable error
            if (_i2c_config.enable_bus_recovery && severity == I2CErrorSeverity::RECOVERABLE) {
                ESP_LOGW(TAG, "Attempting I2C bus recovery for error type: %d", 
                         static_cast<int>(error_type));
                performBusRecovery();
            }
            
            // Calculate exponential backoff delay
            uint32_t delay_ms = calculateBackoffDelay(attempt);
            ESP_LOGD(TAG, "Retrying in %lums...", delay_ms);
            vTaskDelay(pdMS_TO_TICKS(delay_ms));
        }
    }
    
    ESP_LOGE(TAG, "I2C write to reg 0x%02X failed after %d attempts: %s", 
             static_cast<uint8_t>(reg), _i2c_config.max_retries + 1, esp_err_to_name(ret));
    updateI2CStats(false, error_type);
    return ret;
}

esp_err_t MPU6050Driver::readRegistersWithRetry(MPU6050Register reg, uint8_t* data, size_t len) const {
    std::lock_guard<std::mutex> lock(_i2c_mutex);
    
    if (_dev_handle == nullptr) {
        ESP_LOGE(TAG, "Device handle not initialized for readRegisters");
        updateI2CStats(false, I2CErrorType::PERMANENT_FAILURE);
        return ESP_ERR_INVALID_STATE;
    }
    
    if (data == nullptr || len == 0) {
        ESP_LOGE(TAG, "Invalid arguments for readRegisters");
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t reg_addr = static_cast<uint8_t>(reg);
    esp_err_t ret = ESP_FAIL;
    I2CErrorType error_type = I2CErrorType::NONE;
    
    for (uint8_t attempt = 0; attempt <= _i2c_config.max_retries; attempt++) {
        _i2c_stats.total_operations++;
        
        // Perform I2C read operation
        ret = i2c_master_transmit_receive(_dev_handle, &reg_addr, 1, data, len, 
                                         pdMS_TO_TICKS(_i2c_config.timeout_ms));
        
        if (ret == ESP_OK) {
            ESP_LOGV(TAG, "Read %d bytes from reg 0x%02X (attempt %d)", len, reg_addr, attempt + 1);
            updateI2CStats(true, I2CErrorType::NONE);
            return ESP_OK;
        }
        
        // Classify the error
        error_type = classifyI2CError(ret);
        I2CErrorSeverity severity = classifyErrorSeverity(error_type);
        
        ESP_LOGW(TAG, "I2C read failed from reg 0x%02X (attempt %d/%d): %s [Error: %d, Severity: %d]", 
                 reg_addr, attempt + 1, _i2c_config.max_retries + 1, 
                 esp_err_to_name(ret), static_cast<int>(error_type), static_cast<int>(severity));
        
        // Don't retry on permanent failures
        if (severity == I2CErrorSeverity::PERMANENT) {
            ESP_LOGE(TAG, "Permanent I2C failure detected (type: %d), aborting retries", 
                     static_cast<int>(error_type));
            break;
        }
        
        // Don't delay after the last attempt
        if (attempt < _i2c_config.max_retries) {
            // Attempt bus recovery if enabled and this is a recoverable error
            if (_i2c_config.enable_bus_recovery && severity == I2CErrorSeverity::RECOVERABLE) {
                ESP_LOGW(TAG, "Attempting I2C bus recovery for error type: %d", 
                         static_cast<int>(error_type));
                const_cast<MPU6050Driver*>(this)->performBusRecovery();
            }
            
            // Calculate exponential backoff delay
            uint32_t delay_ms = calculateBackoffDelay(attempt);
            ESP_LOGD(TAG, "Retrying in %lums...", delay_ms);
            vTaskDelay(pdMS_TO_TICKS(delay_ms));
        }
    }
    
    ESP_LOGE(TAG, "I2C read from reg 0x%02X failed after %d attempts: %s", 
             reg_addr, _i2c_config.max_retries + 1, esp_err_to_name(ret));
    updateI2CStats(false, error_type);
    return ret;
}

// --- I2C Error Handling and Recovery ---

I2CErrorType MPU6050Driver::classifyI2CError(esp_err_t error) const {
    switch (error) {
        case ESP_ERR_TIMEOUT:
            return I2CErrorType::TIMEOUT;
        case ESP_ERR_INVALID_RESPONSE:
            return I2CErrorType::INVALID_RESPONSE;
        case ESP_FAIL:
            return I2CErrorType::NACK;
        case ESP_ERR_INVALID_STATE:
            return I2CErrorType::BUS_ERROR;
        case ESP_ERR_NOT_FOUND:
            return I2CErrorType::DEVICE_NOT_FOUND;
        case ESP_ERR_INVALID_ARG:
            return I2CErrorType::PERMANENT_FAILURE;
        case ESP_ERR_NO_MEM:
            return I2CErrorType::HARDWARE_FAULT;
        default:
            // For unknown errors, classify based on error code range
            if (error >= ESP_ERR_INVALID_ARG && error <= ESP_ERR_INVALID_STATE) {
                return I2CErrorType::PERMANENT_FAILURE;
            } else {
                return I2CErrorType::HARDWARE_FAULT;
            }
    }
}

I2CErrorSeverity MPU6050Driver::classifyErrorSeverity(I2CErrorType error_type) const {
    switch (error_type) {
        case I2CErrorType::TIMEOUT:
        case I2CErrorType::NACK:
        case I2CErrorType::FIFO_OVERFLOW:
            return I2CErrorSeverity::TEMPORARY;
        
        case I2CErrorType::BUS_ERROR:
        case I2CErrorType::ARBITRATION_LOST:
        case I2CErrorType::INVALID_RESPONSE:
            return I2CErrorSeverity::RECOVERABLE;
        
        case I2CErrorType::PERMANENT_FAILURE:
        case I2CErrorType::DEVICE_NOT_FOUND:
        case I2CErrorType::HARDWARE_FAULT:
            return I2CErrorSeverity::PERMANENT;
        
        default:
            return I2CErrorSeverity::PERMANENT;
    }
}

esp_err_t MPU6050Driver::performBusRecovery() {
    _i2c_stats.recovery_attempts++;
    
    ESP_LOGW(TAG, "Performing I2C bus recovery procedure");
    
    // Note: ESP-IDF I2C master driver doesn't expose direct bus recovery functions
    // The recovery is typically handled internally by the driver
    // However, we can attempt to reinitialize the bus as a recovery mechanism
    
    if (_bus_handle == nullptr) {
        ESP_LOGE(TAG, "Cannot perform bus recovery: bus handle is null");
        return ESP_ERR_INVALID_STATE;
    }
    
    // For now, we'll implement a simple recovery by waiting
    // In a more advanced implementation, we could:
    // 1. Generate clock pulses on SCL to clear stuck devices
    // 2. Reset the I2C peripheral
    // 3. Reinitialize the bus configuration
    
    vTaskDelay(pdMS_TO_TICKS(50)); // Wait 50ms for bus to settle
    
    _i2c_stats.successful_recoveries++;
    ESP_LOGI(TAG, "I2C bus recovery completed");
    
    return ESP_OK;
}

void MPU6050Driver::updateI2CStats(bool success, I2CErrorType error_type) const {
    uint64_t current_time = getCurrentTimestampUs();
    
    // Update timing statistics
    if (_i2c_stats.last_operation_timestamp > 0) {
        uint32_t operation_time = (uint32_t)(current_time - _i2c_stats.last_operation_timestamp);
        if (operation_time > _i2c_stats.max_operation_time_us) {
            _i2c_stats.max_operation_time_us = operation_time;
        }
        // Simple moving average for operation time
        _i2c_stats.avg_operation_time_us = 
            (_i2c_stats.avg_operation_time_us * 9 + operation_time) / 10;
    }
    _i2c_stats.last_operation_timestamp = current_time;
    
    if (success) {
        _i2c_stats.successful_operations++;
        _i2c_stats.consecutive_failures = 0;
    } else {
        // Update error counters by specific type
        switch (error_type) {
            case I2CErrorType::TIMEOUT:
                _i2c_stats.timeout_errors++;
                break;
            case I2CErrorType::NACK:
                _i2c_stats.nack_errors++;
                break;
            case I2CErrorType::BUS_ERROR:
                _i2c_stats.bus_errors++;
                break;
            case I2CErrorType::ARBITRATION_LOST:
                _i2c_stats.arbitration_lost_errors++;
                break;
            case I2CErrorType::FIFO_OVERFLOW:
                _i2c_stats.fifo_overflow_errors++;
                break;
            case I2CErrorType::PERMANENT_FAILURE:
                _i2c_stats.permanent_failure_errors++;
                break;
            case I2CErrorType::DEVICE_NOT_FOUND:
                _i2c_stats.device_not_found_errors++;
                break;
            case I2CErrorType::INVALID_RESPONSE:
                _i2c_stats.invalid_response_errors++;
                break;
            case I2CErrorType::HARDWARE_FAULT:
                _i2c_stats.hardware_fault_errors++;
                break;
            default:
                break;
        }
        
        // Update consecutive failure tracking
        _i2c_stats.consecutive_failures++;
        if (_i2c_stats.consecutive_failures > _i2c_stats.max_consecutive_failures) {
            _i2c_stats.max_consecutive_failures = _i2c_stats.consecutive_failures;
        }
        
        // Update error trend analysis
        updateErrorTrend(error_type);
    }
    
    // Calculate current error rate
    if (_i2c_stats.total_operations > 0) {
        uint32_t total_errors = _i2c_stats.total_operations - _i2c_stats.successful_operations;
        _i2c_stats.current_error_rate_percent = 
            (float)total_errors / _i2c_stats.total_operations * 100.0f;
    }
}

void MPU6050Driver::updateErrorTrend(I2CErrorType error_type) const {
    uint64_t current_time = getCurrentTimestampUs();
    I2CErrorTrend& trend = _i2c_stats.error_trend;
    
    // Update rolling window of error counts
    trend.error_count_window[trend.window_index]++;
    trend.total_window_errors++;
    
    // Calculate error rate per second
    if (trend.last_error_timestamp > 0) {
        uint64_t time_diff_us = current_time - trend.last_error_timestamp;
        if (time_diff_us > 0) {
            // Calculate errors per second based on recent window
            float time_diff_s = time_diff_us / 1000000.0f;
            trend.error_rate_per_second = trend.total_window_errors / time_diff_s;
        }
    }
    
    trend.last_error_timestamp = current_time;
    
    // Advance window every 100 errors or every 10 seconds
    static uint32_t error_count_in_current_window = 0;
    static uint64_t window_start_time = current_time;
    
    error_count_in_current_window++;
    
    if (error_count_in_current_window >= 100 || 
        (current_time - window_start_time) >= 10000000) { // 10 seconds in microseconds
        
        // Calculate trend direction
        uint32_t recent_errors = 0;
        uint32_t older_errors = 0;
        
        // Compare recent half vs older half of the window
        for (int i = 0; i < 5; i++) {
            recent_errors += trend.error_count_window[(trend.window_index + 6 + i) % 10];
            older_errors += trend.error_count_window[(trend.window_index + 1 + i) % 10];
        }
        
        trend.trend_increasing = (recent_errors > older_errors);
        
        // Move to next window
        trend.window_index = (trend.window_index + 1) % 10;
        trend.total_window_errors -= trend.error_count_window[trend.window_index];
        trend.error_count_window[trend.window_index] = 0;
        
        error_count_in_current_window = 0;
        window_start_time = current_time;
    }
}

uint64_t MPU6050Driver::getCurrentTimestampUs() const {
    return esp_timer_get_time();
}

uint32_t MPU6050Driver::calculateBackoffDelay(uint8_t retry_count) const {
    // Exponential backoff: base_delay * 2^retry_count
    uint32_t delay_ms = _i2c_config.base_delay_ms * (1 << retry_count);
    
    // Cap at maximum delay
    if (delay_ms > _i2c_config.max_delay_ms) {
        delay_ms = _i2c_config.max_delay_ms;
    }
    
    return delay_ms;
}

