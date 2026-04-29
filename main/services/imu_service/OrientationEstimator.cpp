#include "OrientationEstimator.hpp"
#include <algorithm>
#include <cmath>
#include "esp_log.h"

// Constructor implementation
OrientationEstimator::OrientationEstimator() :
    m_alpha(0.98f),
    m_sample_period_s(MPU6050Profile::DEFAULT_SAMPLE_PERIOD_S),
    m_pitch_deg(0.0f),
    m_pitch_rate_dps(0.0f),
    m_yaw_deg(0.0f),
    m_yaw_rate_dps(0.0f),
    m_pitch_bias_dps(0.0f),
    m_p00(1.0f),
    m_p01(0.0f),
    m_p10(0.0f),
    m_p11(1.0f),
    m_has_estimate(false),
    m_gyro_offset_x_dps(0.0f),
    m_gyro_offset_y_dps(0.0f),
    m_gyro_offset_z_dps(0.0f)
{
    ESP_LOGI(TAG, "Orientation Estimator created.");
}

// Initialization method
void OrientationEstimator::init(float alpha, float sample_period_s,
                                float gyro_offset_x_dps,
                                float gyro_offset_y_dps,
                                float gyro_offset_z_dps) {
    m_alpha.store(alpha, std::memory_order_relaxed);
    m_sample_period_s.store(sample_period_s, std::memory_order_relaxed);
    m_gyro_offset_x_dps.store(gyro_offset_x_dps, std::memory_order_relaxed);
    m_gyro_offset_y_dps.store(gyro_offset_y_dps, std::memory_order_relaxed);
    m_gyro_offset_z_dps.store(gyro_offset_z_dps, std::memory_order_relaxed);
    m_pitch_deg.store(0.0f, std::memory_order_relaxed);
    m_pitch_rate_dps.store(0.0f, std::memory_order_relaxed);
    m_yaw_deg.store(0.0f, std::memory_order_relaxed);
    m_yaw_rate_dps.store(0.0f, std::memory_order_relaxed);
    m_pitch_bias_dps.store(0.0f, std::memory_order_relaxed);
    m_p00.store(1.0f, std::memory_order_relaxed);
    m_p01.store(0.0f, std::memory_order_relaxed);
    m_p10.store(0.0f, std::memory_order_relaxed);
    m_p11.store(1.0f, std::memory_order_relaxed);
    m_has_estimate.store(false, std::memory_order_relaxed);
    
    ESP_LOGI(TAG, "Estimator init: accelTrust=%.3f, dt=%.4fs, Offsets(X:%.3f Y:%.3f Z:%.3f)",
             alpha, sample_period_s, gyro_offset_x_dps, gyro_offset_y_dps, gyro_offset_z_dps);
}

void OrientationEstimator::updateGyroOffsets(float gyro_offset_x_dps,
                                             float gyro_offset_y_dps,
                                             float gyro_offset_z_dps) {
    m_gyro_offset_x_dps.store(gyro_offset_x_dps, std::memory_order_relaxed);
    m_gyro_offset_y_dps.store(gyro_offset_y_dps, std::memory_order_relaxed);
    m_gyro_offset_z_dps.store(gyro_offset_z_dps, std::memory_order_relaxed);
    m_yaw_deg.store(0.0f, std::memory_order_relaxed);
    m_pitch_bias_dps.store(0.0f, std::memory_order_relaxed);
    m_p00.store(1.0f, std::memory_order_relaxed);
    m_p01.store(0.0f, std::memory_order_relaxed);
    m_p10.store(0.0f, std::memory_order_relaxed);
    m_p11.store(1.0f, std::memory_order_relaxed);
    
    ESP_LOGI(TAG, "Estimator gyro offsets updated: X:%.3f Y:%.3f Z:%.3f",
             gyro_offset_x_dps, gyro_offset_y_dps, gyro_offset_z_dps);
}

void OrientationEstimator::reset() {
    m_pitch_deg.store(0.0f, std::memory_order_relaxed);
    m_pitch_rate_dps.store(0.0f, std::memory_order_relaxed);
    m_yaw_deg.store(0.0f, std::memory_order_relaxed);
    m_yaw_rate_dps.store(0.0f, std::memory_order_relaxed);
    m_pitch_bias_dps.store(0.0f, std::memory_order_relaxed);
    m_p00.store(1.0f, std::memory_order_relaxed);
    m_p01.store(0.0f, std::memory_order_relaxed);
    m_p10.store(0.0f, std::memory_order_relaxed);
    m_p11.store(1.0f, std::memory_order_relaxed);
    m_has_estimate.store(false, std::memory_order_relaxed);
    
    ESP_LOGI(TAG, "Orientation Estimator state reset.");
}

void OrientationEstimator::processSample(float ax_g, float ay_g, float az_g,
                                        float raw_gyro_dps_x, float raw_gyro_dps_y, float raw_gyro_dps_z)
{
    // Suppress unused parameter warning for raw_gyro_dps_x (kept for interface compatibility)
    (void)raw_gyro_dps_x;
    const float current_pitch_deg_local = m_pitch_deg.load(std::memory_order_relaxed);
    const float gy_offset_local = m_gyro_offset_y_dps.load(std::memory_order_relaxed);
    const float gz_offset_local = m_gyro_offset_z_dps.load(std::memory_order_relaxed);
    const float sample_period_s_local = m_sample_period_s.load(std::memory_order_relaxed);
    const float alpha_local = m_alpha.load(std::memory_order_relaxed);

    // Apply gyro offsets (only Y and Z are used in current implementation)
    float gyro_dps_y = raw_gyro_dps_y - gy_offset_local;
    float gyro_dps_z = raw_gyro_dps_z - gz_offset_local;

    // --- Bias-aware Kalman observer for pitch ---
    float accel_pitch_deg = current_pitch_deg_local;
    bool accel_pitch_valid = false;
    float yz_mag_sq = (ay_g * ay_g) + (az_g * az_g);
    if (yz_mag_sq > 1e-6) {
        accel_pitch_deg = std::atan2(-ax_g, std::sqrt(yz_mag_sq)) * RAD_TO_DEG;
        accel_pitch_valid = true;
    } else {
        ESP_LOGV(TAG, "Accel YZ magnitude too small for pitch calculation (ay=%.3f, az=%.3f)", ay_g, az_g);
    }

    float dt = sample_period_s_local;
    if (dt <= 0) {
         ESP_LOGW(TAG,
                  "Invalid sample period (%.4f) in estimator, using %.4fs",
                  dt,
                  MPU6050Profile::DEFAULT_SAMPLE_PERIOD_S);
         dt = MPU6050Profile::DEFAULT_SAMPLE_PERIOD_S;
     }

    if (accel_pitch_valid && !m_has_estimate.load(std::memory_order_relaxed)) {
        m_pitch_deg.store(accel_pitch_deg, std::memory_order_relaxed);
        m_has_estimate.store(true, std::memory_order_relaxed);
    }

    float angle_deg = m_pitch_deg.load(std::memory_order_relaxed);
    float bias_dps = m_pitch_bias_dps.load(std::memory_order_relaxed);
    float yaw_deg = m_yaw_deg.load(std::memory_order_relaxed);
    float p00 = m_p00.load(std::memory_order_relaxed);
    float p01 = m_p01.load(std::memory_order_relaxed);
    float p10 = m_p10.load(std::memory_order_relaxed);
    float p11 = m_p11.load(std::memory_order_relaxed);

    const float unbiased_rate_dps = gyro_dps_y - bias_dps;
    angle_deg += dt * unbiased_rate_dps;

    p00 += dt * ((dt * p11) - p01 - p10 + KALMAN_PROCESS_NOISE_ANGLE);
    p01 -= dt * p11;
    p10 -= dt * p11;
    p11 += KALMAN_PROCESS_NOISE_BIAS * dt;

    const float accel_mag_g = std::sqrt((ax_g * ax_g) + (ay_g * ay_g) + (az_g * az_g));
    const float accel_deviation_g = std::fabs(accel_mag_g - 1.0f);
    if (accel_pitch_valid && accel_deviation_g <= ACCEL_REJECTION_THRESHOLD_G) {
        const float alpha = std::clamp(alpha_local, 0.0f, 0.999f);
        const float trustRatio = std::max(0.01f, alpha / std::max(1.0f - alpha, 0.001f));
        const float dynamicScale = 1.0f +
            std::max(0.0f, accel_deviation_g - ACCEL_TRUST_DEADBAND_G) *
            (MAX_ACCEL_NOISE_SCALE - 1.0f) /
            std::max(ACCEL_REJECTION_THRESHOLD_G - ACCEL_TRUST_DEADBAND_G, 0.001f);
        const float measurementNoise =
            std::max(MIN_ACCEL_NOISE_DEG2, KALMAN_BASE_ACCEL_NOISE_DEG2 * trustRatio * dynamicScale);

        const float innovation = accel_pitch_deg - angle_deg;
        const float innovationCovariance = p00 + measurementNoise;
        if (innovationCovariance > 1e-6f) {
            const float k0 = p00 / innovationCovariance;
            const float k1 = p10 / innovationCovariance;

            angle_deg += k0 * innovation;
            bias_dps += k1 * innovation;

            const float p00_prior = p00;
            const float p01_prior = p01;
            p00 -= k0 * p00_prior;
            p01 -= k0 * p01_prior;
            p10 -= k1 * p00_prior;
            p11 -= k1 * p01_prior;
        }
    }

    const float latest_pitch_rate_dps = gyro_dps_y - bias_dps;

    // --- Store Yaw Rate ---
    // We use the offset-corrected Z-axis gyro reading for yaw rate
    float latest_yaw_rate_dps = gyro_dps_z;
    yaw_deg += dt * latest_yaw_rate_dps;

    m_pitch_deg.store(angle_deg, std::memory_order_relaxed);
    m_pitch_rate_dps.store(latest_pitch_rate_dps, std::memory_order_relaxed);
    m_yaw_deg.store(yaw_deg, std::memory_order_relaxed);
    m_yaw_rate_dps.store(latest_yaw_rate_dps, std::memory_order_relaxed);
    m_pitch_bias_dps.store(bias_dps, std::memory_order_relaxed);
    m_p00.store(p00, std::memory_order_relaxed);
    m_p01.store(p01, std::memory_order_relaxed);
    m_p10.store(p10, std::memory_order_relaxed);
    m_p11.store(p11, std::memory_order_relaxed);

    ESP_LOGV(TAG, "Processed Sample: Acc(%.2f,%.2f,%.2f) RawGyro(%.2f,%.2f,%.2f) -> P:%.2f PR:%.2f Bias:%.3f YawR:%.2f",
             ax_g, ay_g, az_g, raw_gyro_dps_x, raw_gyro_dps_y, raw_gyro_dps_z,
             angle_deg, latest_pitch_rate_dps, bias_dps, latest_yaw_rate_dps);
}

float OrientationEstimator::getPitchDeg() const {
    return m_pitch_deg.load(std::memory_order_relaxed);
}

float OrientationEstimator::getPitchRateDPS() const {
    return m_pitch_rate_dps.load(std::memory_order_relaxed);
}

float OrientationEstimator::getYawDeg() const {
    return m_yaw_deg.load(std::memory_order_relaxed);
}

float OrientationEstimator::getYawRateDPS() const {
    return m_yaw_rate_dps.load(std::memory_order_relaxed);
}

OrientationEstimate OrientationEstimator::getOrientation() const {
    return {
        m_pitch_deg.load(std::memory_order_relaxed),
        m_pitch_rate_dps.load(std::memory_order_relaxed),
        m_yaw_deg.load(std::memory_order_relaxed),
        m_yaw_rate_dps.load(std::memory_order_relaxed)
    };
}

std::pair<float, float> OrientationEstimator::getPitchAndYawRate() const {
    return {
        m_pitch_deg.load(std::memory_order_relaxed),
        m_yaw_rate_dps.load(std::memory_order_relaxed)
    };
}
