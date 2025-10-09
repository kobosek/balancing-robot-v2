#pragma once

namespace IMUScale {

// Returns LSB per g for given accel range code (0..3)
inline float computeAccelLSBPerG(int accel_range_code) {
    switch (accel_range_code) {
        case 0: return 16384.0f; // ±2g
        case 1: return 8192.0f;  // ±4g
        case 2: return 4096.0f;  // ±8g
        case 3: return 2048.0f;  // ±16g
        default: return 8192.0f; // safe default
    }
}

// Returns LSB per dps for given gyro range code (0..3)
inline float computeGyroLSBPerDPS(int gyro_range_code) {
    switch (gyro_range_code) {
        case 0: return 131.0f;  // ±250 dps
        case 1: return 65.5f;   // ±500 dps
        case 2: return 32.8f;   // ±1000 dps
        case 3: return 16.4f;   // ±2000 dps
        default: return 65.5f;  // safe default
    }
}

} // namespace IMUScale
