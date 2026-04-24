#include "LogBufferService.hpp"

#include "esp_log.h"
#include "esp_timer.h"
#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <sys/time.h>

namespace {
vprintf_like_t s_originalVprintf = nullptr;
LogBufferService* s_logBufferService = nullptr;

int log_capture_vprintf(const char* fmt, va_list args)
{
    if (s_logBufferService && s_logBufferService->isEnabled()) {
        va_list copy;
        va_copy(copy, args);
        s_logBufferService->appendVprintf(fmt, copy);
        va_end(copy);
    }

    if (s_originalVprintf) {
        return s_originalVprintf(fmt, args);
    }
    return 0;
}
}

LogBufferService::LogBufferService() = default;

LogBufferService::~LogBufferService()
{
    if (s_logBufferService == this) {
        esp_log_set_vprintf(s_originalVprintf);
        s_logBufferService = nullptr;
        s_originalVprintf = nullptr;
    }
    free(m_storage);
    free(m_lengths);
    free(m_sequences);
}

esp_err_t LogBufferService::init(bool enabled, size_t maxLines, size_t maxLineLength)
{
    m_enabled = enabled;
    m_maxLines = std::max<size_t>(1, maxLines);
    m_maxLineLength = std::max<size_t>(2, maxLineLength);
    m_nextIndex = 0;
    m_count = 0;

    free(m_storage);
    free(m_lengths);
    free(m_sequences);
    m_storage = static_cast<char*>(calloc(m_maxLines, m_maxLineLength));
    m_lengths = static_cast<size_t*>(calloc(m_maxLines, sizeof(size_t)));
    m_sequences = static_cast<uint64_t*>(calloc(m_maxLines, sizeof(uint64_t)));
    if (!m_storage || !m_lengths || !m_sequences) {
        free(m_storage);
        free(m_lengths);
        free(m_sequences);
        m_storage = nullptr;
        m_lengths = nullptr;
        m_sequences = nullptr;
        return ESP_ERR_NO_MEM;
    }

    s_logBufferService = this;
    s_originalVprintf = esp_log_set_vprintf(log_capture_vprintf);
    return ESP_OK;
}

void LogBufferService::append(const char* line)
{
    if (!m_enabled || !line || !m_storage || !m_lengths || !m_sequences) {
        return;
    }

    std::lock_guard<std::mutex> lock(m_mutex);
    char* slot = m_storage + (m_nextIndex * m_maxLineLength);
    const size_t copyLen = strnlen(line, m_maxLineLength - 1);
    memcpy(slot, line, copyLen);
    slot[copyLen] = '\0';
    m_lengths[m_nextIndex] = copyLen;
    m_sequences[m_nextIndex] = m_nextSequence++;

    m_nextIndex = (m_nextIndex + 1) % m_maxLines;
    if (m_count < m_maxLines) {
        ++m_count;
    }
}

void LogBufferService::appendVprintf(const char* fmt, va_list args)
{
    if (!m_enabled || !fmt || !m_storage || !m_lengths || !m_sequences) {
        return;
    }

    std::lock_guard<std::mutex> lock(m_mutex);
    char* slot = m_storage + (m_nextIndex * m_maxLineLength);
    const size_t prefixLen = buildTimestampPrefix(slot, m_maxLineLength);
    size_t offset = 0;
    if (prefixLen > 0 && prefixLen + 1 < m_maxLineLength) {
        slot[prefixLen] = ' ';
        offset = prefixLen + 1;
    }

    const int len = vsnprintf(slot + offset, m_maxLineLength - offset, fmt, args);
    if (len <= 0) {
        return;
    }

    slot[m_maxLineLength - 1] = '\0';
    m_lengths[m_nextIndex] = strnlen(slot, m_maxLineLength - 1);
    m_sequences[m_nextIndex] = m_nextSequence++;

    m_nextIndex = (m_nextIndex + 1) % m_maxLines;
    if (m_count < m_maxLines) {
        ++m_count;
    }
}

size_t LogBufferService::buildTimestampPrefix(char* buffer, size_t bufferSize) const
{
    if (!buffer || bufferSize == 0) {
        return 0;
    }

    timeval now = {};
    gettimeofday(&now, nullptr);

    tm timeinfo = {};
    if (now.tv_sec > 1700000000 && localtime_r(&now.tv_sec, &timeinfo)) {
        const int written = snprintf(
            buffer,
            bufferSize,
            "%04d-%02d-%02d %02d:%02d:%02d.%03ld",
            timeinfo.tm_year + 1900,
            timeinfo.tm_mon + 1,
            timeinfo.tm_mday,
            timeinfo.tm_hour,
            timeinfo.tm_min,
            timeinfo.tm_sec,
            now.tv_usec / 1000
        );
        return written > 0 ? strnlen(buffer, bufferSize) : 0;
    }

    const int64_t uptimeMs = esp_timer_get_time() / 1000;
    const int written = snprintf(buffer, bufferSize, "+%lld.%03llds", uptimeMs / 1000, uptimeMs % 1000);
    return written > 0 ? strnlen(buffer, bufferSize) : 0;
}

std::vector<std::string> LogBufferService::snapshot(size_t maxLines) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    const size_t available = m_count;
    const size_t requested = maxLines == 0 ? available : std::min(maxLines, available);
    std::vector<std::string> result;
    result.reserve(requested);

    if (requested == 0 || !m_storage) {
        return result;
    }

    const size_t oldestIndex = m_count == m_maxLines ? m_nextIndex : 0;
    const size_t skip = available - requested;
    for (size_t i = skip; i < available; ++i) {
        const size_t index = (oldestIndex + i) % m_maxLines;
        const char* slot = m_storage + (index * m_maxLineLength);
        result.emplace_back(slot, m_lengths[index]);
    }

    return result;
}

std::vector<BufferedLogLine> LogBufferService::snapshotSince(uint64_t sinceSequence, size_t maxLines, uint64_t& nextSequence) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    nextSequence = m_nextSequence;

    std::vector<BufferedLogLine> result;
    if (m_count == 0 || !m_storage || !m_lengths || !m_sequences) {
        return result;
    }

    const size_t oldestIndex = m_count == m_maxLines ? m_nextIndex : 0;
    size_t matchingCount = 0;
    for (size_t i = 0; i < m_count; ++i) {
        const size_t index = (oldestIndex + i) % m_maxLines;
        if (m_sequences[index] > sinceSequence) {
            ++matchingCount;
        }
    }

    const size_t requested = maxLines == 0 ? matchingCount : std::min(maxLines, matchingCount);
    result.reserve(requested);
    const size_t skip = matchingCount > requested ? matchingCount - requested : 0;
    size_t seen = 0;

    for (size_t i = 0; i < m_count; ++i) {
        const size_t index = (oldestIndex + i) % m_maxLines;
        if (m_sequences[index] <= sinceSequence) {
            continue;
        }
        if (seen++ < skip) {
            continue;
        }

        const char* slot = m_storage + (index * m_maxLineLength);
        result.push_back({m_sequences[index], std::string(slot, m_lengths[index])});
    }

    return result;
}

void LogBufferService::clear()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_lengths) {
        memset(m_lengths, 0, m_maxLines * sizeof(size_t));
    }
    if (m_sequences) {
        memset(m_sequences, 0, m_maxLines * sizeof(uint64_t));
    }
    if (m_storage) {
        memset(m_storage, 0, m_maxLines * m_maxLineLength);
    }
    m_nextIndex = 0;
    m_count = 0;
}
