#pragma once

class IIMUDataSink {
public:
    virtual ~IIMUDataSink() = default;

    virtual void processSample(float accel_g_x, float accel_g_y, float accel_g_z,
                               float raw_gyro_dps_x, float raw_gyro_dps_y, float raw_gyro_dps_z) = 0;
};
