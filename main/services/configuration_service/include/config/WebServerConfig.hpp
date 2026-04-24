#pragma once

struct WebServerConfig {
    int telemetry_buffer_size = 100;
    int max_config_post_size = 4096;
    bool web_logs_enabled = true;
    int log_buffer_lines = 250;
    int log_line_max_length = 256;

    bool operator!=(const WebServerConfig& other) const {
        return telemetry_buffer_size != other.telemetry_buffer_size ||
               max_config_post_size != other.max_config_post_size ||
               web_logs_enabled != other.web_logs_enabled ||
               log_buffer_lines != other.log_buffer_lines ||
               log_line_max_length != other.log_line_max_length;
    }

    bool operator==(const WebServerConfig& other) const {
        return !(*this != other);
    }
};
