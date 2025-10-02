#include "FIFOProcessor.hpp"
#include "mpu6050.hpp"
#include "IMUHealthMonitor.hpp"
#include "IMU_CommunicationError.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <algorithm>

FIFOProcessor::FIFOProcessor(MPU6050Driver& driver,
                             std::shared_ptr<OrientationEstimator> estimator,
                             IMUHealthMonitor& healthMonitor,
                             EventBus& eventBus)
    : m_driver(driver),
      m_estimator(estimator),
      m_healthMonitor(healthMonitor),
      m_eventBus(eventBus),
      m_accel_lsb_per_g(8192.0f),  // Default value, will be updated by setScalingFactors
      m_gyro_lsb_per_dps(65.5f),   // Default value, will be updated by setScalingFactors
      m_isr_data_counter(0),
      m_interrupt_counter(0),
      m_isr_active(false),
      m_isr_handler_installed(false),
      m_interrupt_pin(-1),
      m_interrupt_active_high(true) {
    // Create binary semaphore for synchronization with FIFOTask
    m_dataReadySemaphore = xSemaphoreCreateBinary();
    if (m_dataReadySemaphore == NULL) {
        ESP_LOGE(TAG, "Failed to create data ready semaphore");
    }
}

FIFOProcessor::~FIFOProcessor() {
    // Clean up ISR if still installed
    if (m_isr_handler_installed && m_interrupt_pin >= 0 && m_interrupt_pin < GPIO_NUM_MAX) {
        unregisterInterrupt(m_interrupt_pin);
    }
    
    // Clean up semaphore
    if (m_dataReadySemaphore != NULL) {
        vSemaphoreDelete(m_dataReadySemaphore);
        m_dataReadySemaphore = NULL;
    }
    
    ESP_LOGI(TAG, "FIFOProcessor destroyed");
}

bool FIFOProcessor::processFIFO() {
    uint8_t isr_hint = getAndResetIsrDataCounter();
    if (isr_hint > 0) {
        ESP_LOGV(TAG, "ISR hint received (%d data-ready events), checking FIFO count.", isr_hint);
    }
    

    uint16_t fifo_count = 0;
    esp_err_t ret = m_driver.readFifoCount(fifo_count);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read FIFO count: %s. Publishing error.", esp_err_to_name(ret));
        m_eventBus.publish(IMU_CommunicationError(ret)); 
        return false; 
    }

    bool overflow_flag = false;
    ret = m_driver.isFIFOOverflow(overflow_flag); 
    if (ret != ESP_OK) {
         ESP_LOGW(TAG, "Failed to check FIFO overflow status from interrupt register: %s", esp_err_to_name(ret));
    }
    if (overflow_flag || fifo_count >= MAX_FIFO_BUFFER_SIZE) { 
         ESP_LOGW(TAG, "FIFO overflow detected (Flag: %d, Count: %u). Resetting FIFO.", overflow_flag, fifo_count);
         resetAndReEnableFIFO(); 
         return false; 
    }

    if (fifo_count < FIFO_PACKET_SIZE) {
        return true;  // No data to process, but not an error
    }

    uint16_t num_samples_available = fifo_count / FIFO_PACKET_SIZE;
    uint16_t num_samples_to_read = std::min(num_samples_available, MAX_SAMPLES_PER_PIPELINE_CALL);
    uint16_t bytes_to_read = num_samples_to_read * FIFO_PACKET_SIZE;

    if (bytes_to_read > MAX_FIFO_BUFFER_SIZE) { 
        ESP_LOGW(TAG, "Calculated bytes_to_read (%u) > MAX_FIFO_BUFFER_SIZE (%u). Clamping.",
                 bytes_to_read, MAX_FIFO_BUFFER_SIZE);
        bytes_to_read = MAX_FIFO_BUFFER_SIZE;
        num_samples_to_read = bytes_to_read / FIFO_PACKET_SIZE;
    }

    ESP_LOGV(TAG, "FIFO: %u bytes (%u avail). Reading: %u bytes (%u samples).",
             fifo_count, num_samples_available, bytes_to_read, num_samples_to_read);

    if (bytes_to_read == 0) {
        return true;  // No data to process, but not an error
    }
    
    ret = m_driver.readFifoBuffer(m_fifo_buffer, bytes_to_read);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read FIFO buffer: %s. Publishing error.", esp_err_to_name(ret));
        m_eventBus.publish(IMU_CommunicationError(ret));
        return false; 
    }
    
    if (!validateFIFOData(m_fifo_buffer, bytes_to_read)) {
        ESP_LOGW(TAG, "FIFO data validation failed. Resetting and re-enabling FIFO.");
        resetAndReEnableFIFO();
        return false; 
    }

    int processed_sample_count = 0;
    for (uint16_t i = 0; i < num_samples_to_read; ++i) {
        int offset = i * FIFO_PACKET_SIZE;
        if (offset + FIFO_PACKET_SIZE > bytes_to_read) {
            ESP_LOGE(TAG,"Buffer read boundary error (offset=%d, i=%u, bytes_read=%u). Stopping processing for this batch.", 
                    offset, i, bytes_to_read);
            break; 
        }

        int16_t ax_raw = (m_fifo_buffer[offset + 0] << 8) | m_fifo_buffer[offset + 1];
        int16_t ay_raw = (m_fifo_buffer[offset + 2] << 8) | m_fifo_buffer[offset + 3];
        int16_t az_raw = (m_fifo_buffer[offset + 4] << 8) | m_fifo_buffer[offset + 5];
        int16_t gx_raw = (m_fifo_buffer[offset + 6] << 8) | m_fifo_buffer[offset + 7];
        int16_t gy_raw = (m_fifo_buffer[offset + 8] << 8) | m_fifo_buffer[offset + 9];
        int16_t gz_raw = (m_fifo_buffer[offset + 10] << 8) | m_fifo_buffer[offset + 11];

        float ax_g = static_cast<float>(ax_raw) / m_accel_lsb_per_g;
        float ay_g = static_cast<float>(ay_raw) / m_accel_lsb_per_g;
        float az_g = static_cast<float>(az_raw) / m_accel_lsb_per_g;
        float gx_dps = static_cast<float>(gx_raw) / m_gyro_lsb_per_dps; // Scaled, but not offset-corrected here
        float gy_dps = static_cast<float>(gy_raw) / m_gyro_lsb_per_dps; // Scaled, but not offset-corrected here
        float gz_dps = static_cast<float>(gz_raw) / m_gyro_lsb_per_dps; // Scaled, but not offset-corrected here

        // Pass scaled (but not offset-corrected) gyro data to estimator. Estimator applies its configured offsets.
        m_estimator->processSample(ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps);
        processed_sample_count++;
    }

    if (processed_sample_count > 0) {
        ESP_LOGV(TAG, "Processed %d samples from FIFO.", processed_sample_count);
        m_healthMonitor.pet(); 
    } else if (bytes_to_read > 0) {
        ESP_LOGW(TAG, "Read %u bytes from FIFO but processed 0 samples. This might indicate an issue.", bytes_to_read);
    }
    
    return processed_sample_count > 0;
}

void FIFOProcessor::setScalingFactors(float accelLsbPerG, float gyroLsbPerDps) {
    m_accel_lsb_per_g = accelLsbPerG;
    m_gyro_lsb_per_dps = gyroLsbPerDps;
    
    ESP_LOGI(TAG, "Scaling factors updated: Accel=%.1f LSB/g, Gyro=%.1f LSB/dps", 
             m_accel_lsb_per_g, m_gyro_lsb_per_dps);
}

esp_err_t FIFOProcessor::resetFIFO() {
    ESP_LOGD(TAG, "Resetting FIFO (USER_CTRL.FIFO_RESET=1)");
    esp_err_t ret = m_driver.configureFIFOReg(MPU6050UserControl::FIFO_RESET, (MPU6050FIFOEnable)0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set FIFO_RESET bit: %s", esp_err_to_name(ret));
    }
    // Use direct ms conversion (1 tick = 1ms at default 100Hz tick rate)
    vTaskDelay(2); 
    return ret;
}

esp_err_t FIFOProcessor::enableFIFO() {
    ESP_LOGI(TAG, "Enabling FIFO (USER_CTRL.FIFO_ENABLE=1, FIFO_EN=ACCEL|GYRO)");
    
    esp_err_t ret = m_driver.configureFIFOReg(MPU6050UserControl::FIFO_ENABLE, MPU6050FIFOEnable::GYRO_ACCEL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable FIFO: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

esp_err_t FIFOProcessor::enableFIFO(int interruptPin, bool activeHigh) {
    // First enable the FIFO
    esp_err_t ret = enableFIFO();
    if (ret != ESP_OK) {
        return ret;
    }
    // Then register the ISR for the interrupt pin
    ret = registerInterrupt(interruptPin, activeHigh);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register ISR handler when enabling FIFO: %s", esp_err_to_name(ret));
        // If ISR registration fails, try to disable FIFO to leave in consistent state
        disableFIFO();
        return ret;
    }
    
    // Store the interrupt pin for later unregistration
    m_interrupt_pin = interruptPin;
    
    ESP_LOGI(TAG, "FIFO enabled and ISR registered on pin %d", interruptPin);
    return ESP_OK;
}

esp_err_t FIFOProcessor::disableFIFO() {
    ESP_LOGD(TAG, "Disabling FIFO (USER_CTRL.FIFO_ENABLE=0, FIFO_EN=0) and resetting.");
    esp_err_t ret;
    ret = m_driver.disableFIFO(); 
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to disable FIFO: %s", esp_err_to_name(ret));
    }
    esp_err_t reset_ret = resetFIFO();
    if (reset_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset FIFO: %s", esp_err_to_name(reset_ret));
    }   

    // If requested and ISR handler is installed, unregister it
    if (m_isr_handler_installed && m_interrupt_pin >= 0) {
        esp_err_t isr_ret = unregisterInterrupt(m_interrupt_pin);
        if (isr_ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to unregister ISR when disabling FIFO: %s", esp_err_to_name(isr_ret));
            // Don't return error here as FIFO has been disabled already
            // Just use the FIFO disable result
        } else {
            ESP_LOGI(TAG, "FIFO disabled and ISR unregistered from pin %d", m_interrupt_pin);
        }
    }
    
    return (ret == ESP_OK) ? reset_ret : ret; ;
}

esp_err_t FIFOProcessor::resetAndReEnableFIFO() {
    ESP_LOGI(TAG, "Performing FIFO Reset and Re-Enable sequence.");
    
    // Store the current interrupt state before reset
    bool isr_was_installed = m_isr_handler_installed;
    int interrupt_pin = m_interrupt_pin;
    
    // Reset and re-enable FIFO
    esp_err_t ret = resetFIFO();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset FIFO during re-enable sequence: %s. Publishing error.", esp_err_to_name(ret));
        m_eventBus.publish(IMU_CommunicationError(ret));
        return ret;
    }
    // Use direct ms conversion (1 tick = 1ms at default 100Hz tick rate)
    vTaskDelay(5); 
    if (isr_was_installed && interrupt_pin >= 0 && interrupt_pin < GPIO_NUM_MAX) {
        ret = enableFIFO(interrupt_pin, m_interrupt_active_high);
    } else {
        ret = enableFIFO();
    }
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to re-enable FIFO after reset: %s. Publishing error.", esp_err_to_name(ret));
        m_eventBus.publish(IMU_CommunicationError(ret));
    }
    
    return ret;
}

bool FIFOProcessor::validateFIFOData(const uint8_t* data, size_t length) {
    if (data == nullptr) {
        ESP_LOGE(TAG, "FIFO data validation failed: null data pointer.");
        return false;
    }
    if (length == 0 || length % FIFO_PACKET_SIZE != 0) {
        ESP_LOGE(TAG, "FIFO data validation failed: invalid length %zu (not multiple of %zu).", 
                 length, FIFO_PACKET_SIZE);
        return false;
    }
    
    const int numSamples = length / FIFO_PACKET_SIZE;
    for (int i = 0; i < numSamples; i++) {
        const uint8_t* sample = data + (i * FIFO_PACKET_SIZE);
        int16_t accelX = (sample[0] << 8) | sample[1];
        if ((accelX == 0 || accelX == -1) &&
            (((sample[2] << 8) | sample[3]) == accelX) && 
            (((sample[4] << 8) | sample[5]) == accelX)    
           ) {
            int16_t gyroX = (sample[6] << 8) | sample[7];
            if ((gyroX == 0 || gyroX == -1) &&
                (((sample[8] << 8) | sample[9]) == gyroX) && 
                (((sample[10] << 8) | sample[11]) == gyroX)  
               ) {
                ESP_LOGW(TAG, "FIFO data validation: detected suspicious all-same pattern (0x%04X or 0x0000) in sample %d.", 
                        static_cast<uint16_t>(accelX), i);
                return false; 
            }
        }
    }
    return true;
}

void FIFOProcessor::incrementIsrDataCounter() {
    // Prevent ISR reentrancy - if already active, just return
    bool expected = false;
    if (!m_isr_active.compare_exchange_strong(expected, true, std::memory_order_acquire)) {
        // ISR is already active, skip this interrupt to prevent race conditions
        return;
    }
    
    // Use acquire-release memory ordering for proper synchronization
    uint8_t count = m_interrupt_counter.fetch_add(1, std::memory_order_acq_rel) + 1;
    
    // Only signal the semaphore every INTERRUPTS_PER_PROCESS interrupts
    if (count >= INTERRUPTS_PER_PROCESS) {
        // Reset the counter with proper memory ordering
        m_interrupt_counter.store(0, std::memory_order_release);
        
        // Increment the data counter that will be read by processFIFO
        m_isr_data_counter.fetch_add(1, std::memory_order_acq_rel);
        
        // Validate semaphore before using it in ISR context
        if (m_dataReadySemaphore != NULL) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            BaseType_t result = xSemaphoreGiveFromISR(m_dataReadySemaphore, &xHigherPriorityTaskWoken);
            
            // Check if semaphore operation was successful
            if (result == pdTRUE && xHigherPriorityTaskWoken == pdTRUE) {
                portYIELD_FROM_ISR();
            } else if (result != pdTRUE) {
                // Semaphore give failed - this could indicate the semaphore is full
                // or corrupted. We'll continue but this should be logged if possible
                // Note: Can't use ESP_LOG in ISR context, so we'll just continue
            }
        }
    }
    
    // Clear the ISR active flag with release ordering
    m_isr_active.store(false, std::memory_order_release);
}

void IRAM_ATTR FIFOProcessor::isrHandler(void* arg) {
    // Validate the processor pointer before using it
    FIFOProcessor* processor = static_cast<FIFOProcessor*>(arg);
    if (processor != nullptr) {
        // Additional validation - check if the processor is still valid
        // by checking if the semaphore exists (basic sanity check)
        if (processor->m_dataReadySemaphore != NULL) {
            processor->incrementIsrDataCounter();
        }
    }
}

esp_err_t FIFOProcessor::registerInterrupt(int interruptPin, bool activeHigh) {
    if (interruptPin < 0 || interruptPin >= GPIO_NUM_MAX) {
        ESP_LOGE(TAG, "Invalid interrupt pin: %d", interruptPin);
        return ESP_ERR_INVALID_ARG;
    }
    
    // Validate semaphore exists before registering ISR
    if (m_dataReadySemaphore == NULL) {
        ESP_LOGE(TAG, "Data ready semaphore not initialized, cannot register ISR");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Reset atomic counters before registering ISR
    m_isr_data_counter.store(0, std::memory_order_release);
    m_interrupt_counter.store(0, std::memory_order_release);
    m_isr_active.store(false, std::memory_order_release);
    
    // Store interrupt pin
    m_interrupt_pin = interruptPin;
    
    // Configure GPIO for interrupt
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << interruptPin);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = activeHigh ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = activeHigh ? GPIO_INTR_POSEDGE : GPIO_INTR_NEGEDGE;
    
    m_interrupt_active_high = activeHigh;
    
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure interrupt pin %d: %s", interruptPin, esp_err_to_name(ret));
        return ret;
    }
    
    // Install ISR service if not already installed
    ret = gpio_install_isr_service(ESP_INTR_FLAG_LOWMED);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to install ISR service: %s", esp_err_to_name(ret));
        return ret;
    } else if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "ISR service already installed. Proceeding.");
    }
    
    // Remove handler if already installed
    gpio_isr_handler_remove(static_cast<gpio_num_t>(interruptPin));
    
    // Add handler
    ret = gpio_isr_handler_add(static_cast<gpio_num_t>(interruptPin), isrHandler, this);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add ISR handler for pin %d: %s", interruptPin, esp_err_to_name(ret));
        return ret;
    }
    
    m_isr_handler_installed = true;
    ESP_LOGI(TAG, "GPIO interrupt configured and handler added for INT pin %d", interruptPin);
    
    return ESP_OK;
}

esp_err_t FIFOProcessor::unregisterInterrupt(int interruptPin) {
    if (interruptPin < 0 || interruptPin >= GPIO_NUM_MAX) {
        ESP_LOGE(TAG, "Invalid interrupt pin: %d", interruptPin);
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!m_isr_handler_installed) {
        ESP_LOGW(TAG, "ISR handler not installed");
        return ESP_OK;
    }
    
    // Wait for any active ISR to complete before unregistering
    uint32_t timeout_ms = 10;
    uint32_t elapsed_ms = 0;
    while (m_isr_active.load(std::memory_order_acquire) && elapsed_ms < timeout_ms) {
        vTaskDelay(pdMS_TO_TICKS(1));
        elapsed_ms++;
    }
    
    if (m_isr_active.load(std::memory_order_acquire)) {
        ESP_LOGW(TAG, "ISR still active after timeout, proceeding with unregistration");
    }
    
    esp_err_t ret = gpio_isr_handler_remove(static_cast<gpio_num_t>(interruptPin));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error removing ISR handler for pin %d: %s", interruptPin, esp_err_to_name(ret));
        return ret;
    }
    
    // Reset counters after unregistering ISR
    m_isr_data_counter.store(0, std::memory_order_release);
    m_interrupt_counter.store(0, std::memory_order_release);
    m_isr_active.store(false, std::memory_order_release);
    
    m_isr_handler_installed = false;
    ESP_LOGI(TAG, "Removed ISR handler for pin %d", interruptPin);
    
    return ESP_OK;
}

uint8_t FIFOProcessor::getAndResetIsrDataCounter() {
    return m_isr_data_counter.exchange(0, std::memory_order_acq_rel);
}
