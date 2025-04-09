#include "IMUService.hpp"
#include "driver/gpio.h"
#include "esp_check.h"
#include <cmath>
#include <atomic>
#include <vector>
#include <numeric>
#include <algorithm>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "mpu6050.hpp"
#include "OrientationEstimator.hpp"
#include "OrientationDataEvent.hpp"

#include "EventBus.hpp"
#include "BaseEvent.hpp"
#include "StartCalibrationRequestEvent.hpp"
#include "CalibrationCompleteEvent.hpp"
#include "EventTypes.hpp"
#include "IMU_CommunicationErrorEvent.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

const float G_ACCEL = 9.80665f;

// Constructor
IMUService::IMUService(const MPU6050Config& config, EventBus& bus, OrientationEstimator& estimator) :
    m_config(config),
    m_eventBus(bus),
    m_estimator(estimator),
    m_sensor(),
    m_fifo_task_handle(nullptr),
    m_calibration_mutex(nullptr),
    m_is_calibrating(false),
    m_gyro_offset_dps{0.0f, 0.0f, 0.0f}, // Units: DPS
    m_isr_data_counter(0),
    m_calib_gx_samples(),
    m_calib_gy_samples(),
    m_calib_gz_samples()
{
    // Reserve space in vectors based on config
    m_calib_gx_samples.reserve(m_config.calibration_samples);
    m_calib_gy_samples.reserve(m_config.calibration_samples);
    m_calib_gz_samples.reserve(m_config.calibration_samples);
}

IMUService::~IMUService() {
    ESP_LOGI(TAG, "Deconstructing IMUService...");
    if (m_fifo_task_handle) { vTaskDelete(m_fifo_task_handle); m_fifo_task_handle = nullptr; }
    if (m_calibration_mutex) { vSemaphoreDelete(m_calibration_mutex); m_calibration_mutex = nullptr; }
    if (m_config.int_pin >= 0 && m_config.int_pin < GPIO_NUM_MAX) {
         // Ideally check if handler was installed before removing
         gpio_isr_handler_remove(m_config.int_pin);
    }
    ESP_LOGI(TAG, "IMUService deconstructed.");
}

esp_err_t IMUService::init() {
    ESP_LOGI(TAG, "Initializing IMUService...");
    m_calibration_mutex = xSemaphoreCreateMutex();
    ESP_RETURN_ON_FALSE(m_calibration_mutex != NULL, ESP_FAIL, TAG, "Failed create calibration mutex");

    esp_err_t ret = m_sensor.init(m_config.i2c_port, m_config.sda_pin, m_config.scl_pin, m_config.device_address, m_config.i2c_freq_hz);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed init MPU6050 driver");

    // Sensor configuration
    MPU6050AccelConfig accelRange = static_cast<MPU6050AccelConfig>(m_config.accel_range);
    MPU6050GyroConfig gyroRange = static_cast<MPU6050GyroConfig>(m_config.gyro_range);
    MPU6050DLPFConfig dlpf = static_cast<MPU6050DLPFConfig>(m_config.dlpf_config);
    MPU6050SampleRateDiv rateDiv = static_cast<MPU6050SampleRateDiv>(m_config.sample_rate_divisor);
    ESP_RETURN_ON_ERROR(m_sensor.setAccelRange(accelRange), TAG, "Failed set Accel Range");
    ESP_RETURN_ON_ERROR(m_sensor.setGyroRange(gyroRange), TAG, "Failed set Gyro Range");
    ESP_RETURN_ON_ERROR(m_sensor.setDLPFConfig(dlpf), TAG, "Failed set DLPF");
    ESP_RETURN_ON_ERROR(m_sensor.setSampleRate(rateDiv), TAG, "Failed set Sample Rate");

    // Initial calibration
    ret = performCalibration();
    CalibrationCompleteEvent initial_calib_event(ret);
    m_eventBus.publish(initial_calib_event);
    ESP_RETURN_ON_ERROR(ret, TAG, "Initial Gyro calibration failed");

    // Interrupt setup
    if (m_config.int_pin >= 0 && m_config.int_pin < GPIO_NUM_MAX) {
        gpio_config_t io_conf = {};
        io_conf.pin_bit_mask = (1ULL << m_config.int_pin); io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE; io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.intr_type = m_config.interrupt_active_high ? GPIO_INTR_POSEDGE : GPIO_INTR_NEGEDGE;
        ret = gpio_config(&io_conf); ESP_RETURN_ON_ERROR(ret, TAG, "Failed config INT GPIO %d", m_config.int_pin);
        static bool isr_service_installed = false;
        if (!isr_service_installed) { ret = gpio_install_isr_service(ESP_INTR_FLAG_LOWMED); if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) { ESP_RETURN_ON_ERROR(ret, TAG, "Failed install ISR service"); } isr_service_installed = (ret == ESP_OK || ret == ESP_ERR_INVALID_STATE); }
        ret = gpio_isr_handler_add(m_config.int_pin, isrHandler, this); if (ret == ESP_ERR_INVALID_STATE) { ESP_LOGW(TAG, "ISR handler already added for pin %d.", m_config.int_pin); ret = ESP_OK; }
        ESP_RETURN_ON_ERROR(ret, TAG, "Failed add ISR handler for pin %d", m_config.int_pin); ESP_LOGI(TAG, "GPIO interrupt configured for INT pin %d", m_config.int_pin);
        MPU6050InterruptPinConfig intPinConfig = m_config.interrupt_active_high ? MPU6050InterruptPinConfig::ACTIVE_HIGH : MPU6050InterruptPinConfig::ACTIVE_LOW;
        ESP_RETURN_ON_ERROR(m_sensor.setupInterrupt(intPinConfig, MPU6050Interrupt::DATA_READY), TAG, "Failed setup MPU Interrupt");
    } else { ESP_LOGW(TAG, "Interrupt pin not configured (pin=%d)", m_config.int_pin); }

    // FIFO setup
    ESP_RETURN_ON_ERROR(m_sensor.setupFIFO(MPU6050UserControl::FIFO_RESET_ENABLE, MPU6050FIFOEnable::GYRO_ACCEL), TAG, "Failed setup MPU FIFO");

    // FIFO Task
    BaseType_t taskCreated = xTaskCreatePinnedToCore( fifo_task_wrapper, "imu_fifo_task", 4096, this, configMAX_PRIORITIES - 2, &m_fifo_task_handle, 0);
    ESP_RETURN_ON_FALSE(taskCreated == pdPASS, ESP_FAIL, TAG, "Failed create IMU FIFO task"); ESP_LOGI(TAG, "IMU FIFO Task created on Core 0.");

    // Subscribe to Calibration Request
    m_eventBus.subscribe(EventType::START_CALIBRATION_REQUEST, [this](const BaseEvent& ev) { this->triggerCalibration(); }); ESP_LOGI(TAG, "Subscribed to START_CALIBRATION_REQUEST events.");
    ESP_LOGI(TAG, "IMUService Initialized successfully.");
    return ESP_OK;
}

esp_err_t IMUService::triggerCalibration() {
    ESP_LOGI(TAG, "Calibration triggered.");
    if (xSemaphoreTake(m_calibration_mutex, pdMS_TO_TICKS(100)) != pdTRUE) { ESP_LOGW(TAG, "Calibration busy"); CalibrationCompleteEvent e(ESP_ERR_TIMEOUT); m_eventBus.publish(e); return ESP_ERR_TIMEOUT; }
    esp_err_t ret = performCalibration();
    CalibrationCompleteEvent e(ret); m_eventBus.publish(e); ESP_LOGI(TAG,"Published CalibComplete (Status: %s)", esp_err_to_name(ret));
    xSemaphoreGive(m_calibration_mutex);
    return ret;
}

esp_err_t IMUService::performCalibration() {
    if (m_is_calibrating) { return ESP_ERR_INVALID_STATE; }
    m_is_calibrating = true;
    ESP_LOGI(TAG, "Starting Gyro calibration (X, Y, Z) for %d samples...", m_config.calibration_samples);
    float old_offset_x_dps = m_gyro_offset_dps[0]; float old_offset_y_dps = m_gyro_offset_dps[1]; float old_offset_z_dps = m_gyro_offset_dps[2];
    double gx_sum = 0, gy_sum = 0, gz_sum = 0; int successful_samples = 0; int attempts = 0; const int max_attempts = m_config.calibration_samples + 200;
    m_calib_gx_samples.clear(); m_calib_gy_samples.clear(); m_calib_gz_samples.clear();

    while (successful_samples < m_config.calibration_samples && attempts < max_attempts) {
        attempts++; float ax, ay, az, gx_dps, gy_dps, gz_dps;
        esp_err_t ret_a = m_sensor.getAcceleration(ax, ay, az); esp_err_t ret_g = m_sensor.getRotation(gx_dps, gy_dps, gz_dps);
        if (ret_a != ESP_OK || ret_g != ESP_OK) { ESP_LOGE(TAG, "Read fail calib attempt %d", attempts); vTaskDelay(pdMS_TO_TICKS(10)); continue; }
        gx_sum += gx_dps; gy_sum += gy_dps; gz_sum += gz_dps;
        m_calib_gx_samples.push_back(gx_dps); m_calib_gy_samples.push_back(gy_dps); m_calib_gz_samples.push_back(gz_dps); successful_samples++;
        if ((successful_samples % (m_config.calibration_samples / 5) == 0) && successful_samples > 0) { ESP_LOGI(TAG, "Calib progress: %d/%d", successful_samples, m_config.calibration_samples); } vTaskDelay(pdMS_TO_TICKS(5));
    }

    esp_err_t final_status = ESP_OK;
    if (successful_samples < m_config.calibration_samples / 2) {
        ESP_LOGE(TAG, "Insufficient calib samples (%d/%d). Restoring previous offsets.", successful_samples, m_config.calibration_samples);
        m_gyro_offset_dps[0] = old_offset_x_dps; m_gyro_offset_dps[1] = old_offset_y_dps; m_gyro_offset_dps[2] = old_offset_z_dps; final_status = ESP_FAIL;
    } else {
        float offset_gx_dps = gx_sum / successful_samples; float offset_gy_dps = gy_sum / successful_samples; float offset_gz_dps = gz_sum / successful_samples;
        m_gyro_offset_dps[0] = offset_gx_dps; m_gyro_offset_dps[1] = offset_gy_dps; m_gyro_offset_dps[2] = offset_gz_dps; // Store DPS
        auto calculate_stdev = [&](const std::vector<float>& samples, double mean) { if (samples.empty()) return 0.0; double sq_sum = std::inner_product(samples.begin(), samples.end(), samples.begin(), 0.0); return std::sqrt(std::max(0.0, sq_sum / samples.size() - mean * mean)); };
        double stdev_gx = calculate_stdev(m_calib_gx_samples, offset_gx_dps); double stdev_gy = calculate_stdev(m_calib_gy_samples, offset_gy_dps); double stdev_gz = calculate_stdev(m_calib_gz_samples, offset_gz_dps);
        ESP_LOGI(TAG, "Gyro Calib Complete (%d samples). Offsets(dps): X:%.4f Y:%.4f Z:%.4f | Stdev: X:%.4f Y:%.4f Z:%.4f", successful_samples, offset_gx_dps, offset_gy_dps, offset_gz_dps, stdev_gx, stdev_gy, stdev_gz);
        const float OFFSET_WARN_THRESHOLD = 2.0; const float STDEV_WARN_THRESHOLD = 1.0;
        if (std::abs(offset_gx_dps) > OFFSET_WARN_THRESHOLD || std::abs(offset_gy_dps) > OFFSET_WARN_THRESHOLD || std::abs(offset_gz_dps) > OFFSET_WARN_THRESHOLD || stdev_gx > STDEV_WARN_THRESHOLD || stdev_gy > STDEV_WARN_THRESHOLD || stdev_gz > STDEV_WARN_THRESHOLD) { ESP_LOGW(TAG, "Warning: Gyro offset/stdev high."); }
        if (final_status != ESP_FAIL) final_status = ESP_OK;
    }
    m_is_calibrating = false; ESP_LOGI(TAG, "Calibration finished: %s", esp_err_to_name(final_status)); return final_status;
}

// --- ISR and Task ---
// Static ISR handler
void IRAM_ATTR IMUService::isrHandler(void *arg) {
    IMUService *instance = static_cast<IMUService *>(arg);
    if(instance->m_is_calibrating) return; // Read volatile bool is acceptable here
    uint8_t current_count = instance->m_isr_data_counter.fetch_add(1, std::memory_order_relaxed) + 1;
    uint8_t threshold = instance->m_config.fifo_read_threshold;
    if (current_count >= threshold) {
        uint8_t expected = current_count;
        instance->m_isr_data_counter.compare_exchange_strong(expected, 0, std::memory_order_relaxed);
        BaseType_t woken = pdFALSE;
        if (instance->m_fifo_task_handle) { vTaskNotifyGiveFromISR(instance->m_fifo_task_handle, &woken); portYIELD_FROM_ISR(woken); }
    }
}
// Member function called by static handler (not used currently)
void IMUService::handleInterrupt() {}

// Task Wrapper
void IMUService::fifo_task_wrapper(void *arg) {
    IMUService *s = static_cast<IMUService *>(arg);
    if(s) s->fifo_task(); else { ESP_LOGE(TAG, "fifo_task_wrapper null arg!"); vTaskDelete(NULL); }
}

// FIFO Task - Corrected Version
void IMUService::fifo_task() {
    ESP_LOGI(TAG, "IMU FIFO Task started on Core %d.", xPortGetCoreID());
    uint8_t fifo_buffer[1024];
    const int bytes_per_sample = 12;

    float accel_scale = m_sensor.getAccelScale();
    float gyro_scale = m_sensor.getGyroScale();

    while (1) {
        // Wait for notification from ISR
        if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) > 0) {
             bool currently_calibrating = m_is_calibrating; // Read volatile bool

             // Read offsets directly in DPS (stored member is m_gyro_offset_dps)
             float current_offset_y_dps = m_gyro_offset_dps[1];
             float current_offset_z_dps = m_gyro_offset_dps[2];

             // ESP_LOGD(TAG, "FIFO notified. Cal:%s Off(DPS) Y:%.2f Z:%.2f", currently_calibrating?"T":"F", current_offset_y_dps, current_offset_z_dps);

             if (currently_calibrating) {
                 // Drain FIFO logic...
                 uint8_t count_buf_drain[2]; uint16_t fifo_count_drain = 0;
                 if(m_sensor.readRegisters(MPU6050Register::FIFO_COUNT_H, count_buf_drain, 2) == ESP_OK) {
                     fifo_count_drain = (count_buf_drain[0] << 8) | count_buf_drain[1];
                     if (fifo_count_drain >= 1024) {
                         ESP_LOGW(TAG, "FIFO overflow during calib drain.");
                         m_sensor.setupFIFO(MPU6050UserControl::FIFO_RESET_ENABLE, MPU6050FIFOEnable::GYRO_ACCEL);
                         m_isr_data_counter.store(0, std::memory_order_relaxed);
                     } else if (fifo_count_drain > 0) {
                         uint16_t bytes_to_drain = std::min((uint16_t)sizeof(fifo_buffer), fifo_count_drain);
                         m_sensor.readRegisters(MPU6050Register::FIFO_R_W, fifo_buffer, bytes_to_drain);
                     }
                 }
                 continue; // Skip processing
             }

            // Read FIFO Count
            uint8_t count_buf[2]; uint16_t fifo_count = 0;
            esp_err_t ret = m_sensor.readRegisters(MPU6050Register::FIFO_COUNT_H, count_buf, 2);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed read FIFO count: %s.", esp_err_to_name(ret));
                m_consecutive_i2c_failures++;
                if (m_consecutive_i2c_failures >= I2C_FAILURE_THRESHOLD) {
                    ESP_LOGE(TAG, "IMU failure threshold (%d). Publishing event.", m_consecutive_i2c_failures);
                    IMU_CommunicationErrorEvent error_event; m_eventBus.publish(error_event);
                    m_consecutive_i2c_failures = 0;
                }
                continue;
            } else {
                m_consecutive_i2c_failures = 0;
            }
            fifo_count = (count_buf[0] << 8) | count_buf[1];

            // Handle Overflow
            if (fifo_count >= 1024 || m_sensor.isFIFOOverflow() == ESP_OK) {
                ESP_LOGW(TAG, "FIFO overflow detected (count=%d or flag). Resetting.", fifo_count);
                m_sensor.setupFIFO(MPU6050UserControl::FIFO_RESET_ENABLE, MPU6050FIFOEnable::GYRO_ACCEL);
                m_isr_data_counter.store(0, std::memory_order_relaxed);
                continue;
            }

            // Check if enough data for full samples and Read Data
            if (fifo_count >= bytes_per_sample) {
                // Calculate bytes_to_read ensuring full samples
                uint16_t bytes_to_read = (fifo_count / bytes_per_sample) * bytes_per_sample;
                bytes_to_read = std::min((uint16_t)sizeof(fifo_buffer), bytes_to_read);

                // Read Data from HW FIFO
                ret = m_sensor.readRegisters(MPU6050Register::FIFO_R_W, fifo_buffer, bytes_to_read);
                if (ret != ESP_OK) {
                     ESP_LOGE(TAG, "FIFO read data failed: %s.", esp_err_to_name(ret));
                     m_consecutive_i2c_failures++;
                     if (m_consecutive_i2c_failures >= I2C_FAILURE_THRESHOLD) {
                         ESP_LOGE(TAG, "IMU failure threshold (%d). Publishing event.", m_consecutive_i2c_failures);
                         IMU_CommunicationErrorEvent error_event; m_eventBus.publish(error_event);
                         m_consecutive_i2c_failures = 0;
                     }
                     continue;
                } else {
                     m_consecutive_i2c_failures = 0;
                }

                // Process Data - Pass DPS offsets directly
                m_estimator.processFifoBatch(fifo_buffer, bytes_to_read, accel_scale, gyro_scale,
                                             current_offset_y_dps, // Already in DPS
                                             current_offset_z_dps); // Already in DPS

                // Publish Orientation Event (still just pitch)
                float pitch_deg = m_estimator.getPitchDeg();
                OrientationDataEvent orientation_event(pitch_deg * (M_PI / 180.0f), 0.0f);
                m_eventBus.publish(orientation_event);

            } else {
                 // ESP_LOGV(TAG, "FIFO count %d < bytes per sample %d. Waiting.", fifo_count, bytes_per_sample);
            }
        } else {
             ESP_LOGD(TAG, "FIFO task timeout - no INT notification received.");
             // Potentially add a check here: if timeout occurs X times, trigger comms error?
        }
    } // end while(1)
}


esp_err_t IMUService::resetSensor() {
    ESP_LOGI(TAG, "Attempting IMU sensor reset...");
    esp_err_t ret = m_sensor.writeRegister(MPU6050Register::PWR_MGMT_1, 0x80); // Set RESET bit
    if (ret == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(100));
        ret = m_sensor.writeRegister(MPU6050Register::PWR_MGMT_1, 0x00); // Wake up
        if (ret == ESP_OK) {
             // Set clock source explicitly after reset
             ret = m_sensor.writeRegister(MPU6050Register::PWR_MGMT_1, static_cast<uint8_t>(MPU6050PowerManagement::CLOCK_PLL_XGYRO));
        }
    }
    ESP_LOGI(TAG, "Sensor reset attempt finished: %s", esp_err_to_name(ret));
    return ret;
}

esp_err_t IMUService::reinitializeSensor() {
    ESP_LOGI(TAG, "Reinitializing IMU sensor configuration...");
    esp_err_t ret = ESP_OK;
    MPU6050AccelConfig ar = static_cast<MPU6050AccelConfig>(m_config.accel_range); MPU6050GyroConfig gr = static_cast<MPU6050GyroConfig>(m_config.gyro_range); MPU6050DLPFConfig dlpf = static_cast<MPU6050DLPFConfig>(m_config.dlpf_config); MPU6050SampleRateDiv sr = static_cast<MPU6050SampleRateDiv>(m_config.sample_rate_divisor);
    ret = m_sensor.setAccelRange(ar); if (ret != ESP_OK) { ESP_LOGE(TAG, "Reinit Accel failed: %s", esp_err_to_name(ret)); return ret; }
    ret = m_sensor.setGyroRange(gr); if (ret != ESP_OK) { ESP_LOGE(TAG, "Reinit Gyro failed: %s", esp_err_to_name(ret)); return ret; }
    ret = m_sensor.setDLPFConfig(dlpf); if (ret != ESP_OK) { ESP_LOGE(TAG, "Reinit DLPF failed: %s", esp_err_to_name(ret)); return ret; }
    ret = m_sensor.setSampleRate(sr); if (ret != ESP_OK) { ESP_LOGE(TAG, "Reinit Sample Rate failed: %s", esp_err_to_name(ret)); return ret; }
    if (m_config.int_pin >= 0 && m_config.int_pin < GPIO_NUM_MAX) { MPU6050InterruptPinConfig ipc = m_config.interrupt_active_high ? MPU6050InterruptPinConfig::ACTIVE_HIGH : MPU6050InterruptPinConfig::ACTIVE_LOW; ret = m_sensor.setupInterrupt(ipc, MPU6050Interrupt::DATA_READY); if (ret != ESP_OK) { ESP_LOGE(TAG, "Reinit Interrupt failed: %s", esp_err_to_name(ret)); return ret; } }
    ret = m_sensor.setupFIFO(MPU6050UserControl::FIFO_RESET_ENABLE, MPU6050FIFOEnable::GYRO_ACCEL); if (ret != ESP_OK) { ESP_LOGE(TAG, "Reinit FIFO failed: %s", esp_err_to_name(ret)); return ret; }
    m_isr_data_counter.store(0, std::memory_order_relaxed); m_consecutive_i2c_failures = 0;
    ESP_LOGI(TAG, "Sensor reinitialization finished: %s", esp_err_to_name(ret));
    return ret;
}

esp_err_t IMUService::verifyCommunication() {
    ESP_LOGI(TAG, "Verifying IMU communication...");
    uint8_t who = 0;
    esp_err_t ret = m_sensor.readRegisters(MPU6050Register::WHO_AM_I, &who, 1); // Read only 1 byte
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Communication verified (WHO_AM_I=0x%02X).", who);
        // Check against expected MPU6050 default or configured address (0x68 is common)
        if (who != m_config.device_address && who != 0x68) {
             ESP_LOGW(TAG, "WHO_AM_I value (0x%02X) unexpected (expected 0x%02X or 0x68).", who, m_config.device_address);
             // ret = ESP_FAIL; // Optionally fail if unexpected ID
        }
    } else {
        ESP_LOGE(TAG, "Communication verification failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

// Example event handler (can be replaced by lambda in init)
void IMUService::handleStartCalibrationRequest(const BaseEvent& event) {
    ESP_LOGW(TAG, "Handling StartCalibrationRequest via method (should use lambda).");
    triggerCalibration();
}