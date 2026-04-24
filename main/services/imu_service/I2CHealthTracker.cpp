#include "I2CHealthTracker.hpp"

#include "esp_timer.h"

void I2CHealthTracker::reset() {
    m_stats = I2CStats{};
}

void I2CHealthTracker::recordSuccess() {
    const uint64_t currentTimeUs = getCurrentTimestampUs();
    m_stats.total_operations++;
    m_stats.successful_operations++;
    m_stats.consecutive_failures = 0;

    if (m_stats.last_operation_timestamp > 0) {
        const uint32_t operationTimeUs = static_cast<uint32_t>(currentTimeUs - m_stats.last_operation_timestamp);
        if (operationTimeUs > m_stats.max_operation_time_us) {
            m_stats.max_operation_time_us = operationTimeUs;
        }
        m_stats.avg_operation_time_us = (m_stats.avg_operation_time_us * 9 + operationTimeUs) / 10;
    }
    m_stats.last_operation_timestamp = currentTimeUs;

    if (m_stats.total_operations > 0) {
        const uint32_t totalErrors = m_stats.total_operations - m_stats.successful_operations;
        m_stats.current_error_rate_percent =
            static_cast<float>(totalErrors) / static_cast<float>(m_stats.total_operations) * 100.0f;
    }
}

void I2CHealthTracker::recordFailure(esp_err_t error) {
    const uint64_t currentTimeUs = getCurrentTimestampUs();
    const I2CErrorType errorType = classifyError(error);

    m_stats.total_operations++;
    m_stats.consecutive_failures++;
    if (m_stats.consecutive_failures > m_stats.max_consecutive_failures) {
        m_stats.max_consecutive_failures = m_stats.consecutive_failures;
    }

    if (m_stats.last_operation_timestamp > 0) {
        const uint32_t operationTimeUs = static_cast<uint32_t>(currentTimeUs - m_stats.last_operation_timestamp);
        if (operationTimeUs > m_stats.max_operation_time_us) {
            m_stats.max_operation_time_us = operationTimeUs;
        }
        m_stats.avg_operation_time_us = (m_stats.avg_operation_time_us * 9 + operationTimeUs) / 10;
    }
    m_stats.last_operation_timestamp = currentTimeUs;

    switch (errorType) {
        case I2CErrorType::TIMEOUT:
            m_stats.timeout_errors++;
            break;
        case I2CErrorType::NACK:
            m_stats.nack_errors++;
            break;
        case I2CErrorType::BUS_ERROR:
            m_stats.bus_errors++;
            break;
        case I2CErrorType::ARBITRATION_LOST:
            m_stats.arbitration_lost_errors++;
            break;
        case I2CErrorType::FIFO_OVERFLOW:
            m_stats.fifo_overflow_errors++;
            break;
        case I2CErrorType::PERMANENT_FAILURE:
            m_stats.permanent_failure_errors++;
            break;
        case I2CErrorType::DEVICE_NOT_FOUND:
            m_stats.device_not_found_errors++;
            break;
        case I2CErrorType::INVALID_RESPONSE:
            m_stats.invalid_response_errors++;
            break;
        case I2CErrorType::HARDWARE_FAULT:
            m_stats.hardware_fault_errors++;
            break;
        case I2CErrorType::NONE:
        default:
            break;
    }

    if (m_stats.total_operations > 0) {
        const uint32_t totalErrors = m_stats.total_operations - m_stats.successful_operations;
        m_stats.current_error_rate_percent =
            static_cast<float>(totalErrors) / static_cast<float>(m_stats.total_operations) * 100.0f;
    }
}

I2CErrorType I2CHealthTracker::classifyError(esp_err_t error) const {
    switch (error) {
        case ESP_OK:
            return I2CErrorType::NONE;
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
            return I2CErrorType::HARDWARE_FAULT;
    }
}

I2CErrorSeverity I2CHealthTracker::classifySeverity(I2CErrorType errorType) const {
    switch (errorType) {
        case I2CErrorType::NONE:
            return I2CErrorSeverity::NONE;
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
        default:
            return I2CErrorSeverity::PERMANENT;
    }
}

uint64_t I2CHealthTracker::getCurrentTimestampUs() {
    return static_cast<uint64_t>(esp_timer_get_time());
}
