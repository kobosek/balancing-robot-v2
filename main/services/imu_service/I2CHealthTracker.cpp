#include "I2CHealthTracker.hpp"

#include "esp_timer.h"

void I2CHealthTracker::reset() {
    m_stats = I2CStats{};
    m_error_count_in_current_window = 0;
    m_window_start_time = 0;
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

    updateErrorTrend(errorType, currentTimeUs);

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
        case I2CErrorType::NONE:
        default:
            return I2CErrorSeverity::PERMANENT;
    }
}

void I2CHealthTracker::updateErrorTrend(I2CErrorType errorType, uint64_t currentTimeUs) {
    (void)errorType;

    I2CErrorTrend& trend = m_stats.error_trend;
    trend.error_count_window[trend.window_index]++;
    trend.total_window_errors++;

    if (trend.last_error_timestamp > 0) {
        const uint64_t timeDiffUs = currentTimeUs - trend.last_error_timestamp;
        if (timeDiffUs > 0) {
            const float timeDiffS = static_cast<float>(timeDiffUs) / 1000000.0f;
            trend.error_rate_per_second = trend.total_window_errors / timeDiffS;
        }
    }

    trend.last_error_timestamp = currentTimeUs;
    if (m_window_start_time == 0) {
        m_window_start_time = currentTimeUs;
    }

    m_error_count_in_current_window++;
    if (m_error_count_in_current_window < 100 &&
        (currentTimeUs - m_window_start_time) < 10000000ULL) {
        return;
    }

    uint32_t recentErrors = 0;
    uint32_t olderErrors = 0;
    for (int i = 0; i < 5; ++i) {
        recentErrors += trend.error_count_window[(trend.window_index + 6 + i) % 10];
        olderErrors += trend.error_count_window[(trend.window_index + 1 + i) % 10];
    }
    trend.trend_increasing = recentErrors > olderErrors;

    trend.window_index = (trend.window_index + 1) % 10;
    trend.total_window_errors -= trend.error_count_window[trend.window_index];
    trend.error_count_window[trend.window_index] = 0;

    m_error_count_in_current_window = 0;
    m_window_start_time = currentTimeUs;
}

uint64_t I2CHealthTracker::getCurrentTimestampUs() {
    return static_cast<uint64_t>(esp_timer_get_time());
}
