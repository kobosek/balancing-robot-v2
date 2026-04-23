#include "FIFOProcessor.hpp"

#include "IIMUFaultSink.hpp"
#include "IMUHealthMonitor.hpp"
#include "MPU6050Profile.hpp"
#include "esp_log.h"
#include "freertos/task.h"
#include "mpu6050.hpp"
#include <algorithm>

namespace {
struct FIFODecodedPacket {
    int16_t axRaw;
    int16_t ayRaw;
    int16_t azRaw;
    int16_t gxRaw;
    int16_t gyRaw;
    int16_t gzRaw;
};

struct FIFOScaledSample {
    float axG;
    float ayG;
    float azG;
    float gxDps;
    float gyDps;
    float gzDps;
};

constexpr int64_t ACCEL_AXIS_FULL_SCALE_RAW = 32768LL;
constexpr int64_t MAX_ALLOWED_ACCEL_MAG2_RAW =
    3LL * ACCEL_AXIS_FULL_SCALE_RAW * ACCEL_AXIS_FULL_SCALE_RAW;

inline int16_t readBigEndianInt16(const uint8_t* sample, size_t offset) {
    return static_cast<int16_t>((sample[offset] << 8) | sample[offset + 1]);
}

inline FIFODecodedPacket parseFifoPacket(const uint8_t* sample) {
    return {
        readBigEndianInt16(sample, 0),
        readBigEndianInt16(sample, 2),
        readBigEndianInt16(sample, 4),
        readBigEndianInt16(sample, 6),
        readBigEndianInt16(sample, 8),
        readBigEndianInt16(sample, 10),
    };
}

bool isTransportCorrupt(const FIFODecodedPacket& packet) {
    const bool accelAllSame = (packet.axRaw == packet.ayRaw) &&
                              (packet.ayRaw == packet.azRaw) &&
                              (packet.axRaw == 0 || packet.axRaw == -1);
    const bool gyroAllSame = (packet.gxRaw == packet.gyRaw) &&
                             (packet.gyRaw == packet.gzRaw) &&
                             (packet.gxRaw == 0 || packet.gxRaw == -1);
    if (accelAllSame && gyroAllSame) {
        return true;
    }

    return false;
}

bool isPhysicsOutlier(const FIFODecodedPacket& packet) {
    const int64_t mag2 = static_cast<int64_t>(packet.axRaw) * packet.axRaw +
                         static_cast<int64_t>(packet.ayRaw) * packet.ayRaw +
                         static_cast<int64_t>(packet.azRaw) * packet.azRaw;
    return mag2 > MAX_ALLOWED_ACCEL_MAG2_RAW;
}
} // namespace

FIFOProcessor::FIFOProcessor(MPU6050Driver& driver,
                             IIMUDataSink& dataSink,
                             IMUHealthMonitor& healthMonitor,
                             IIMUFaultSink& faultSink)
  : m_driver(driver),
      m_dataSink(dataSink),
      m_healthMonitor(healthMonitor),
      m_faultSink(faultSink),
      m_accel_scale_g_per_lsb(1.0f / MPU6050Profile::DEFAULT_ACCEL_LSB_PER_G),
      m_gyro_scale_dps_per_lsb(1.0f / MPU6050Profile::DEFAULT_GYRO_LSB_PER_DPS),
      m_isr_active(false),
      m_dataReadySemaphore(xSemaphoreCreateBinary()),
      m_isr_handler_installed(false),
      m_interrupt_pin(-1),
      m_interrupt_active_high(true),
      m_fifo_read_threshold_bytes(FIFO_PACKET_SIZE),
      m_physics_outlier_count(0) {}

FIFOProcessor::~FIFOProcessor() {
    if (m_isr_handler_installed && m_interrupt_pin >= 0 && m_interrupt_pin < GPIO_NUM_MAX) {
        (void)unregisterInterrupt();
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

    m_fifo_read_threshold_bytes.store(
        std::clamp<uint16_t>(
            std::min<uint16_t>(roundedThresholdBytes, maxBatchBytesForLatency),
            FIFO_PACKET_SIZE,
            maxThresholdBytes),
        std::memory_order_relaxed);
}

bool FIFOProcessor::processFIFO() {
    const float accelScale = m_accel_scale_g_per_lsb.load(std::memory_order_relaxed);
    const float gyroScale = m_gyro_scale_dps_per_lsb.load(std::memory_order_relaxed);
    const uint16_t fifoReadThresholdBytes = m_fifo_read_threshold_bytes.load(std::memory_order_relaxed);
    if (accelScale <= 0.0f || gyroScale <= 0.0f) {
        ESP_LOGE(TAG, "Invalid scaling factors configured for FIFO processing");
        return false;
    }

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

    if (fifoCount < fifoReadThresholdBytes) {
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

    FIFOScaledSample scaledSamples[MAX_SAMPLES_PER_PIPELINE_CALL];
    uint16_t acceptedSampleCount = 0;

    for (uint16_t i = 0; i < samplesToRead; ++i) {
        const size_t offset = static_cast<size_t>(i) * FIFO_PACKET_SIZE;
        const FIFODecodedPacket packet = parseFifoPacket(m_fifo_buffer + offset);

        if (isTransportCorrupt(packet)) {
            ret = recoverFifoHard("transport corruption");
            if (ret != ESP_OK) {
                m_faultSink.onIMUHardFault(ret);
            }
            return false;
        }

        if (isPhysicsOutlier(packet)) {
            m_physics_outlier_count++;
            if (m_physics_outlier_count == 1 ||
                (m_physics_outlier_count % OUTLIER_LOG_INTERVAL) == 0) {
                ESP_LOGW(TAG,
                         "Dropped FIFO physics outlier sample #%lu (ax=%d ay=%d az=%d)",
                         static_cast<unsigned long>(m_physics_outlier_count),
                         packet.axRaw,
                         packet.ayRaw,
                         packet.azRaw);
            }
            continue;
        }

        scaledSamples[acceptedSampleCount] = {
            static_cast<float>(packet.axRaw) * accelScale,
            static_cast<float>(packet.ayRaw) * accelScale,
            static_cast<float>(packet.azRaw) * accelScale,
            static_cast<float>(packet.gxRaw) * gyroScale,
            static_cast<float>(packet.gyRaw) * gyroScale,
            static_cast<float>(packet.gzRaw) * gyroScale
        };
        acceptedSampleCount++;
    }

    int processedSampleCount = 0;
    for (uint16_t i = 0; i < acceptedSampleCount; ++i) {
        const FIFOScaledSample& scaledSample = scaledSamples[i];
        m_dataSink.processSample(
            scaledSample.axG,
            scaledSample.ayG,
            scaledSample.azG,
            scaledSample.gxDps,
            scaledSample.gyDps,
            scaledSample.gzDps);
        processedSampleCount++;
    }

    if (processedSampleCount > 0) {
        m_healthMonitor.pet();
    }

    return processedSampleCount > 0;
}

void FIFOProcessor::setScalingFactors(float accelLsbPerG, float gyroLsbPerDps) {
    m_accel_scale_g_per_lsb.store(accelLsbPerG > 0.0f ? 1.0f / accelLsbPerG : 0.0f, std::memory_order_relaxed);
    m_gyro_scale_dps_per_lsb.store(gyroLsbPerDps > 0.0f ? 1.0f / gyroLsbPerDps : 0.0f, std::memory_order_relaxed);
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
    esp_err_t interruptRet = ESP_OK;

    if (m_isr_handler_installed && m_interrupt_pin >= 0) {
        interruptRet = unregisterInterrupt();
    }

    if (ret != ESP_OK) {
        return ret;
    }
    if (resetRet != ESP_OK) {
        return resetRet;
    }
    return interruptRet;
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
    if (!m_isr_active.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
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

void FIFOProcessor::clearPendingDataReadySignal() {
    if (m_dataReadySemaphore != nullptr) {
        (void)xSemaphoreTake(m_dataReadySemaphore, 0);
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
    esp_err_t ret = m_irqHelper.init(static_cast<gpio_num_t>(interruptPin), activeHigh, &FIFOProcessor::isrHandler, this);
    if (ret != ESP_OK) {
        return ret;
    }

    clearPendingDataReadySignal();
    m_interrupt_pin = interruptPin;
    m_interrupt_active_high = activeHigh;
    m_isr_handler_installed = true;
    return ESP_OK;
}

esp_err_t FIFOProcessor::unregisterInterrupt() {
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
    m_interrupt_pin = -1;
    clearPendingDataReadySignal();
    return ESP_OK;
}
