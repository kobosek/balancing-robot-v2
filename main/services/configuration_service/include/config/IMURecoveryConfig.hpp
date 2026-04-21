#pragma once

struct IMURecoveryConfig {
    bool minimal_interruption = false;
    int max_attempts = 3;
    int retry_delay_ms = 1000;
};
