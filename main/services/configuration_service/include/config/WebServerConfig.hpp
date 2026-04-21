#pragma once

struct WebServerConfig {
    int telemetry_buffer_size = 100;
    int max_config_post_size = 4096;

    bool operator!=(const WebServerConfig& other) const {
        return telemetry_buffer_size != other.telemetry_buffer_size ||
               max_config_post_size != other.max_config_post_size;
    }

    bool operator==(const WebServerConfig& other) const {
        return !(*this != other);
    }
};
