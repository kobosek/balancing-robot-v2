#include "FIFOProcessor.hpp"
#include "mpu6050.hpp"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <algorithm>

void FIFOProcessor::configure(const MPU6050Profile& profile, int threshold) {
    m_profile = profile;
    m_threshold = (threshold + 11) / 12;
    m_chunk = profile.maxReadPackets;
}
esp_err_t FIFOProcessor::resync() {
    esp_err_t ret = m_driver.disableFIFO();
    if (ret == ESP_OK) ret = m_driver.resetFIFO();
    if (ret != ESP_OK) return ret;
    vTaskDelay(std::max<TickType_t>(1, pdMS_TO_TICKS(2)));
    uint8_t status = 0;
    ret = m_driver.getInterruptStatus(status);
    if (ret == ESP_OK) ret = m_driver.configureFIFOReg(MPU6050UserControl::FIFO_ENABLE, MPU6050FIFOEnable::GYRO_ACCEL);
    return ret;
}
FIFOResult FIFOProcessor::processFIFO(uint32_t generation, int64_t retryDeadlineUs) {
    uint16_t count = 0;
    int64_t start = esp_timer_get_time();
    esp_err_t ret = m_driver.readFifoCount(count);
    bool retried = false;
    if (ret != ESP_OK && esp_timer_get_time() + 3000 < retryDeadlineUs) {
        retried = true;
        start = esp_timer_get_time();
        ret = m_driver.readFifoCount(count);
    }
    if (ret != ESP_OK) return {FIFOOutcome::READ_FAILURE, ret, IMUFaultReason::TRANSPORT, 0, retried};
    const unsigned available = count / 12;
    const auto discarded = [&](FIFOOutcome outcome, esp_err_t error, IMUFaultReason reason, unsigned accepted = 0) {
        // All observed packets not accepted will be discarded by recovery. A
        // failed read/overflow can hide additional packets, so never claim an exact count.
        m_sink.recordFifoLoss(available - accepted, true);
        return FIFOResult{outcome, error, reason, accepted, retried};
    };
    if (count >= 1024)
        return discarded(FIFOOutcome::LOST_ALIGNMENT, ESP_ERR_INVALID_SIZE, IMUFaultReason::OVERFLOW);
    if (available == 0) return {FIFOOutcome::NO_DATA, ESP_OK, IMUFaultReason::NONE, 0, retried};
    // Coalesce only while another sample period, the transfer and the next control
    // step still fit. A threshold must never force a fresh cached sample to expire.
    if (available < m_threshold && esp_timer_get_time() +
        (m_threshold - available) * static_cast<int64_t>(m_profile.samplePeriodUs) + 10000 < retryDeadlineUs)
        return {FIFOOutcome::NO_DATA, ESP_OK, IMUFaultReason::NONE, 0, retried};
    const unsigned samples = std::min(available, m_chunk);
    const unsigned timestampPackets = available + (count % 12 != 0 ? 1 : 0);
    if (!samples) return {FIFOOutcome::HARDWARE_FAILURE, ESP_ERR_INVALID_SIZE, IMUFaultReason::CONFIGURATION, 0, retried};
    uint8_t buffer[48]{};
    ret = m_driver.readFifoBuffer(buffer, samples * 12);
    // A failed destructive read may consume any prefix: never retry or decode it.
    if (ret != ESP_OK) return discarded(FIFOOutcome::LOST_ALIGNMENT, ret, IMUFaultReason::FIFO_ALIGNMENT);
    uint8_t status = 0;
    ret = m_driver.getInterruptStatus(status); // Read-to-clear: one attempt.
    if (ret != ESP_OK) return discarded(FIFOOutcome::HARDWARE_FAILURE, ret, IMUFaultReason::TRANSPORT);
    if (status & static_cast<uint8_t>(MPU6050Interrupt::FIFO_OVERFLOW))
        return discarded(FIFOOutcome::LOST_ALIGNMENT, ESP_ERR_INVALID_SIZE, IMUFaultReason::OVERFLOW);
    // Age is not loss of framing. Integrate correctly ordered old samples so
    // the estimator can catch up; motor commits independently reject stale data.
    // Preserve the old transport corruption guard. Reject the entire batch
    // before publishing any packet from an all-zero/all-ones bus response.
    for (unsigned i = 0; i < samples; ++i) {
        const auto* packet = buffer + i * 12;
        const bool badAccel = (packet[0] == 0 || packet[0] == 0xff) &&
            std::all_of(packet, packet + 6, [&](uint8_t byte) { return byte == packet[0]; });
        const bool badGyro = (packet[6] == 0 || packet[6] == 0xff) &&
            std::all_of(packet + 6, packet + 12, [&](uint8_t byte) { return byte == packet[6]; });
        if (badAccel && badGyro)
            return discarded(FIFOOutcome::LOST_ALIGNMENT, ESP_ERR_INVALID_RESPONSE, IMUFaultReason::FIFO_ALIGNMENT);
    }
    for (unsigned i = 0; i < samples; ++i) {
        float values[6];
        IMUSampleMetadata metadata;
        metadata.fifoRemainingPackets = available - i - 1;
        for (unsigned axis = 0; axis < 6; ++axis) {
            const unsigned offset = i * 12 + axis * 2;
            const auto raw = static_cast<int16_t>((buffer[offset] << 8) | buffer[offset + 1]);
            if (raw == INT16_MIN || raw == INT16_MAX) metadata.saturationMask |= 1U << axis;
            values[axis] = raw /
                (axis < 3 ? m_profile.accelLsbPerG : m_profile.gyroLsbPerDps);
        }
        if (!m_sink.processSample(values[0], values[1], values[2], values[3], values[4], values[5],
            sampleTimestamp(start, timestampPackets, i, m_profile.samplePeriodUs), generation, metadata))
            return discarded(FIFOOutcome::LOST_ALIGNMENT, ESP_ERR_INVALID_RESPONSE, IMUFaultReason::VALIDATION, i);
    }
    return {FIFOOutcome::ACCEPTED, ESP_OK, IMUFaultReason::NONE, samples, retried, available > samples};
}
