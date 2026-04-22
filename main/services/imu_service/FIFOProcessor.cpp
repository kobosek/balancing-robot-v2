#include "FIFOProcessor.hpp"

#include "IIMUFaultSink.hpp"
#include "IMUHealthMonitor.hpp"
#include "esp_log.h"
#include "freertos/task.h"
#include "mpu6050.hpp"
#include <algorithm>
#include <climits>

FIFOProcessor::FIFOProcessor(MPU6050Driver& driver,
                             std::shared_ptr<OrientationEstimator> estimator,
                             IMUHealthMonitor& healthMonitor,
                             IIMUFaultSink& faultSink)
    : m_driver(driver),
      m_estimator(estimator),
      m_healthMonitor(healthMonitor),
      m_faultSink(faultSink),
      m_accel_lsb_per_g(8192.0f),
      m_gyro_lsb_per_dps(65.5f),
      m_isr_active(false),
      m_dataReadySemaphore(xSemaphoreCreateBinary()),
      m_isr_handler_installed(false),
      m_interrupt_pin(-1),
      m_interrupt_active_high(true),
      m_fifo_read_threshold_bytes(FIFO_PACKET_SIZE),
      m_misalignment_strikes(0) {}

FIFOProcessor::~FIFOProcessor() {
    if (m_isr_handler_installed && m_interrupt_pin >= 0 && m_interrupt_pin < GPIO_NUM_MAX) {
        unregisterInterrupt(m_interrupt_pin);
    }
    if (m_dataReadySemaphore != nullptr) {
        vSemaphoreDelete(m_dataReadySemaphore);
        m_dataReadySemaphore = nullptr;
    }
}

void FIFOProcessor::configureReadout(float samplePeriodS, int fifoReadThresholdBytes) {
    const uint16_t maxThresholdBytes = static_cast<uint16_t>(MAX_SAMPLES_PER_PIPELINE_CALL * FIFO_PACKET_SIZE);
    const uint16_t requestedBytes = static_cast<uint16_t>(std::clamp(fifoReadThresholdBytes, 1, static_cast<int>(maxThresholdBytes)));
    const uint16_t roundedThresholdBytes = static_cast<uint16_t>(
        ((requestedBytes + FIFO_PACKET_SIZE - 1) / FIFO_PACKET_SIZE) * FIFO_PACKET_SIZE);
    uint16_t maxBatchBytesForLatency = maxThresholdBytes;
    if (samplePeriodS > 0.0f) {
        constexpr float MAX_BATCH_LATENCY_S = 0.008f;
        const int maxPacketsForLatency = std::clamp(
            static_cast<int>(MAX_BATCH_LATENCY_S / samplePeriodS),
            1,
            static_cast<int>(MAX_SAMPLES_PER_PIPELINE_CALL));
        maxBatchBytesForLatency = static_cast<uint16_t>(maxPacketsForLatency * FIFO_PACKET_SIZE);
    }

    m_fifo_read_threshold_bytes = std::clamp<uint16_t>(
        std::min<uint16_t>(roundedThresholdBytes, maxBatchBytesForLatency),
        FIFO_PACKET_SIZE,
        maxThresholdBytes);
}

bool FIFOProcessor::processFIFO() {
    uint16_t fifoCount = 0;
    esp_err_t ret = m_driver.readFifoCount(fifoCount);
    if (ret != ESP_OK) {
        m_faultSink.onIMUHardFault(ret);
        return false;
    }

    if (fifoCount >= MAX_FIFO_BUFFER_SIZE) {
        ret = recoverFifoHard("overflow or fifo_count >= buffer");
        if (ret != ESP_OK) {
            m_faultSink.onIMUHardFault(ret);
        }
        return false;
    }

    if (fifoCount < m_fifo_read_threshold_bytes) {
        return true;
    }

    uint16_t samplesAvailable = fifoCount / FIFO_PACKET_SIZE;
    uint16_t samplesToRead = std::min(samplesAvailable, MAX_SAMPLES_PER_PIPELINE_CALL);
    uint16_t bytesToRead = samplesToRead * FIFO_PACKET_SIZE;
    if (bytesToRead == 0) {
        return true;
    }

    ret = m_driver.readFifoBuffer(m_fifo_buffer, bytesToRead);
    if (ret != ESP_OK) {
        m_faultSink.onIMUHardFault(ret);
        return false;
    }

    if (!validateFIFOData(m_fifo_buffer, bytesToRead)) {
        ret = recoverFifoHard("validation failed");
        if (ret != ESP_OK) {
            m_faultSink.onIMUHardFault(ret);
        }
        return false;
    }

    int processedSampleCount = 0;
    for (uint16_t i = 0; i < samplesToRead; ++i) {
        const int offset = i * FIFO_PACKET_SIZE;
        if (offset + FIFO_PACKET_SIZE > bytesToRead) {
            break;
        }

        const int16_t axRaw = static_cast<int16_t>((m_fifo_buffer[offset + 0] << 8) | m_fifo_buffer[offset + 1]);
        const int16_t ayRaw = static_cast<int16_t>((m_fifo_buffer[offset + 2] << 8) | m_fifo_buffer[offset + 3]);
        const int16_t azRaw = static_cast<int16_t>((m_fifo_buffer[offset + 4] << 8) | m_fifo_buffer[offset + 5]);
        const int16_t gxRaw = static_cast<int16_t>((m_fifo_buffer[offset + 6] << 8) | m_fifo_buffer[offset + 7]);
        const int16_t gyRaw = static_cast<int16_t>((m_fifo_buffer[offset + 8] << 8) | m_fifo_buffer[offset + 9]);
        const int16_t gzRaw = static_cast<int16_t>((m_fifo_buffer[offset + 10] << 8) | m_fifo_buffer[offset + 11]);

        const float axG = static_cast<float>(axRaw) / m_accel_lsb_per_g;
        const float ayG = static_cast<float>(ayRaw) / m_accel_lsb_per_g;
        const float azG = static_cast<float>(azRaw) / m_accel_lsb_per_g;
        const float gxDps = static_cast<float>(gxRaw) / m_gyro_lsb_per_dps;
        const float gyDps = static_cast<float>(gyRaw) / m_gyro_lsb_per_dps;
        const float gzDps = static_cast<float>(gzRaw) / m_gyro_lsb_per_dps;

        m_estimator->processSample(axG, ayG, azG, gxDps, gyDps, gzDps);
        processedSampleCount++;
    }

    if (processedSampleCount > 0) {
        m_healthMonitor.pet();
    }

    return processedSampleCount > 0;
}

void FIFOProcessor::setScalingFactors(float accelLsbPerG, float gyroLsbPerDps) {
    m_accel_lsb_per_g = accelLsbPerG;
    m_gyro_lsb_per_dps = gyroLsbPerDps;
}

esp_err_t FIFOProcessor::resetFIFO() {
    esp_err_t ret = m_driver.configureFIFOReg(MPU6050UserControl::FIFO_RESET, static_cast<MPU6050FIFOEnable>(0));
    if (ret == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    return ret;
}

esp_err_t FIFOProcessor::enableFIFO() {
    return m_driver.configureFIFOReg(MPU6050UserControl::FIFO_ENABLE, MPU6050FIFOEnable::GYRO_ACCEL);
}

esp_err_t FIFOProcessor::enableFIFO(int interruptPin, bool activeHigh) {
    if (!m_isr_handler_installed || m_interrupt_pin != interruptPin || m_interrupt_active_high != activeHigh) {
        esp_err_t ret = registerInterrupt(interruptPin, activeHigh);
        if (ret != ESP_OK) {
            return ret;
        }
    }

    esp_err_t ret = enableFIFO();
    if (ret != ESP_OK) {
        disableFIFO();
        return ret;
    }

    uint8_t interruptStatus = 0;
    (void)m_driver.getInterruptStatus(interruptStatus);
    uint16_t fifoCount = 0;
    (void)m_driver.readFifoCount(fifoCount);

    m_interrupt_pin = interruptPin;
    return ESP_OK;
}

esp_err_t FIFOProcessor::disableFIFO() {
    esp_err_t ret = m_driver.disableFIFO();
    esp_err_t resetRet = resetFIFO();

    if (m_isr_handler_installed && m_interrupt_pin >= 0) {
        (void)unregisterInterrupt(m_interrupt_pin);
    }

    return ret == ESP_OK ? resetRet : ret;
}

esp_err_t FIFOProcessor::resetAndReEnableFIFO() {
    const bool isrWasInstalled = m_isr_handler_installed;
    const int interruptPin = m_interrupt_pin;

    esp_err_t ret = resetFIFO();
    if (ret != ESP_OK) {
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(5));
    if (isrWasInstalled && interruptPin >= 0 && interruptPin < GPIO_NUM_MAX) {
        return enableFIFO(interruptPin, m_interrupt_active_high);
    }

    return enableFIFO();
}

bool FIFOProcessor::validateFIFOData(const uint8_t* data, size_t length) {
    if (data == nullptr || length == 0 || length % FIFO_PACKET_SIZE != 0) {
        return false;
    }

    const int numSamples = static_cast<int>(length / FIFO_PACKET_SIZE);
    for (int i = 0; i < numSamples; ++i) {
        const uint8_t* sample = data + (i * FIFO_PACKET_SIZE);
        const int16_t ax = static_cast<int16_t>((sample[0] << 8) | sample[1]);
        const int16_t ay = static_cast<int16_t>((sample[2] << 8) | sample[3]);
        const int16_t az = static_cast<int16_t>((sample[4] << 8) | sample[5]);
        const int16_t gx = static_cast<int16_t>((sample[6] << 8) | sample[7]);
        const int16_t gy = static_cast<int16_t>((sample[8] << 8) | sample[9]);
        const int16_t gz = static_cast<int16_t>((sample[10] << 8) | sample[11]);

        const bool accelAllSame = (ax == ay) && (ay == az) && (ax == 0 || ax == -1);
        const bool gyroAllSame = (gx == gy) && (gy == gz) && (gx == 0 || gx == -1);
        if (accelAllSame && gyroAllSame) {
            return false;
        }

        const auto isSat = [](int16_t value) { return value == INT16_MAX || value == INT16_MIN; };
        if (isSat(ax) || isSat(ay) || isSat(az) || isSat(gx) || isSat(gy) || isSat(gz)) {
            return false;
        }

        const float axG = static_cast<float>(ax) / m_accel_lsb_per_g;
        const float ayG = static_cast<float>(ay) / m_accel_lsb_per_g;
        const float azG = static_cast<float>(az) / m_accel_lsb_per_g;
        const float fullScaleG = 32768.0f / m_accel_lsb_per_g;
        const float maxAllowedMag2 = fullScaleG * fullScaleG;
        const float mag2 = axG * axG + ayG * ayG + azG * azG;
        if (mag2 < 0.01f || mag2 > maxAllowedMag2) {
            return false;
        }
    }

    return true;
}

esp_err_t FIFOProcessor::recoverFifoHard(const char* reason) {
    ESP_LOGW(TAG, "FIFO hard recovery: %s", reason != nullptr ? reason : "unknown");

    esp_err_t ret = m_driver.disableFIFO();
    if (ret != ESP_OK) {
        return ret;
    }

    ret = m_driver.resetFIFO();
    if (ret != ESP_OK) {
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(2));

    if (m_isr_handler_installed && m_interrupt_pin >= 0 && m_interrupt_pin < GPIO_NUM_MAX) {
        ret = enableFIFO(m_interrupt_pin, m_interrupt_active_high);
    } else {
        ret = enableFIFO();
    }

    if (ret != ESP_OK) {
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(2));
    uint16_t fifoCount = 0;
    if (m_driver.readFifoCount(fifoCount) == ESP_OK && fifoCount >= FIFO_PACKET_SIZE) {
        const size_t bytesToDrain =
            std::min<size_t>((fifoCount / FIFO_PACKET_SIZE) * FIFO_PACKET_SIZE, sizeof(m_fifo_buffer));
        (void)m_driver.readFifoBuffer(m_fifo_buffer, bytesToDrain);
    }

    return ESP_OK;
}

void FIFOProcessor::notifyDataReady() {
    bool expected = false;
    if (!m_isr_active.compare_exchange_strong(expected, true, std::memory_order_acquire)) {
        return;
    }

    if (m_dataReadySemaphore != nullptr) {
        BaseType_t higherPriorityTaskWoken = pdFALSE;
        BaseType_t result = xSemaphoreGiveFromISR(m_dataReadySemaphore, &higherPriorityTaskWoken);
        if (result == pdTRUE && higherPriorityTaskWoken == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }

    m_isr_active.store(false, std::memory_order_release);
}

void IRAM_ATTR FIFOProcessor::isrHandler(void* arg) {
    FIFOProcessor* processor = static_cast<FIFOProcessor*>(arg);
    if (processor != nullptr && processor->m_dataReadySemaphore != nullptr) {
        processor->notifyDataReady();
    }
}

esp_err_t FIFOProcessor::registerInterrupt(int interruptPin, bool activeHigh) {
    if (interruptPin < 0 || interruptPin >= GPIO_NUM_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    if (m_dataReadySemaphore == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    m_isr_active.store(false, std::memory_order_release);
    m_interrupt_pin = interruptPin;
    m_interrupt_active_high = activeHigh;

    esp_err_t ret = m_irqHelper.init(static_cast<gpio_num_t>(interruptPin), activeHigh, &FIFOProcessor::isrHandler, this);
    if (ret != ESP_OK) {
        return ret;
    }

    m_isr_handler_installed = true;
    return ESP_OK;
}

esp_err_t FIFOProcessor::unregisterInterrupt(int interruptPin) {
    if (interruptPin < 0 || interruptPin >= GPIO_NUM_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!m_isr_handler_installed) {
        return ESP_OK;
    }

    uint32_t elapsedMs = 0;
    while (m_isr_active.load(std::memory_order_acquire) && elapsedMs < 10) {
        vTaskDelay(pdMS_TO_TICKS(1));
        elapsedMs++;
    }

    esp_err_t ret = m_irqHelper.deinit();
    if (ret != ESP_OK) {
        return ret;
    }

    m_isr_active.store(false, std::memory_order_release);
    m_isr_handler_installed = false;
    return ESP_OK;
}
