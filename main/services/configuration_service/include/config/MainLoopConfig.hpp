#pragma once

struct MainLoopConfig {
    int interval_ms = 5;

    bool operator!=(const MainLoopConfig& other) const {
        return interval_ms != other.interval_ms;
    }

    bool operator==(const MainLoopConfig& other) const {
        return !(*this != other);
    }
};
