#include "LogsApiHandler.hpp"

#include "LogBufferService.hpp"
#include "esp_log.h"
#include <cstdlib>
#include <string>

LogsApiHandler::LogsApiHandler(LogBufferService& logBuffer)
    : m_logBuffer(logBuffer)
{
    ESP_LOGI(TAG, "LogsApiHandler constructed.");
}

esp_err_t LogsApiHandler::handleGetRequest(httpd_req_t* req)
{
    uint64_t sinceSequence = 0;
    char query[64] = {};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        char sinceValue[24] = {};
        if (httpd_query_key_value(query, "since", sinceValue, sizeof(sinceValue)) == ESP_OK) {
            sinceSequence = strtoull(sinceValue, nullptr, 10);
        }
    }

    uint64_t nextSequence = 0;
    const size_t maxLines = sinceSequence == 0 ? 0 : 50;
    const auto logs = m_logBuffer.snapshotSince(sinceSequence, maxLines, nextSequence);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    char prefix[64];
    snprintf(prefix, sizeof(prefix), "{\"next_sequence\":%llu,\"logs\":[", static_cast<unsigned long long>(nextSequence));
    esp_err_t ret = httpd_resp_sendstr_chunk(req, prefix);
    if (ret != ESP_OK) {
        return ret;
    }

    bool first = true;
    for (const auto& log : logs) {
        if (!first) {
            ret = httpd_resp_sendstr_chunk(req, ",");
            if (ret != ESP_OK) {
                return ret;
            }
        }
        first = false;

        const std::string& line = log.text;
        std::string escaped;
        escaped.reserve(line.size() + 2);
        escaped.push_back('"');
        for (char ch : line) {
            switch (ch) {
                case '\\': escaped += "\\\\"; break;
                case '"': escaped += "\\\""; break;
                case '\b': escaped += "\\b"; break;
                case '\f': escaped += "\\f"; break;
                case '\n': escaped += "\\n"; break;
                case '\r': escaped += "\\r"; break;
                case '\t': escaped += "\\t"; break;
                default:
                    if (static_cast<unsigned char>(ch) < 0x20) {
                        char encoded[7];
                        snprintf(encoded, sizeof(encoded), "\\u%04x", static_cast<unsigned char>(ch));
                        escaped += encoded;
                    } else {
                        escaped.push_back(ch);
                    }
                    break;
            }
        }
        escaped.push_back('"');

        ret = httpd_resp_send_chunk(req, escaped.c_str(), escaped.size());
        if (ret != ESP_OK) {
            return ret;
        }
    }

    ret = httpd_resp_sendstr_chunk(req, "]}");
    if (ret != ESP_OK) {
        return ret;
    }
    return httpd_resp_send_chunk(req, nullptr, 0);
}

esp_err_t LogsApiHandler::handleDeleteRequest(httpd_req_t* req)
{
    m_logBuffer.clear();
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_sendstr(req, "{\"status\":\"ok\"}");
}
