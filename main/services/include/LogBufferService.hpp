#pragma once

#include "esp_err.h"
#include <cstdarg>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

struct BufferedLogLine {
    uint64_t sequence = 0;
    std::string text;
};

class LogBufferService {
public:
    static constexpr size_t TIMESTAMP_PREFIX_MAX_LEN = 32;

    LogBufferService();
    ~LogBufferService();

    esp_err_t init(bool enabled, size_t maxLines, size_t maxLineLength);

    void append(const char* line);
    void appendVprintf(const char* fmt, va_list args);
    std::vector<std::string> snapshot(size_t maxLines = 0) const;
    std::vector<BufferedLogLine> snapshotSince(uint64_t sinceSequence, size_t maxLines, uint64_t& nextSequence) const;
    void clear();

    bool isEnabled() const { return m_enabled; }
    size_t buildTimestampPrefix(char* buffer, size_t bufferSize) const;

private:
    bool m_enabled = false;
    size_t m_maxLines = 0;
    size_t m_maxLineLength = 0;
    size_t m_nextIndex = 0;
    size_t m_count = 0;
    uint64_t m_nextSequence = 1;

    char* m_storage = nullptr;
    size_t* m_lengths = nullptr;
    uint64_t* m_sequences = nullptr;
    mutable std::mutex m_mutex;
};
