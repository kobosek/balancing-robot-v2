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
    m_alpha = alpha;
    m_sample_period_s = sample_period_s;
    m_gyro_offset_x_dps = gyro_offset_x_dps;
    m_gyro_offset_y_dps = gyro_offset_y_dps;
    m_gyro_offset_z_dps = gyro_offset_z_dps;
    m_pitch_deg = 0.0f;
    m_pitch_rate_dps = 0.0f;
    m_yaw_deg = 0.0f;
    m_yaw_rate_dps = 0.0f;
    m_pitch_bias_dps = 0.0f;
    m_p00 = 1.0f;
    m_p01 = 0.0f;
    m_p10 = 0.0f;
    m_p11 = 1.0f;
    m_has_estimate = false;
    
    reset();
    ESP_LOGD(TAG, "Estimator init: accelTrust=%.3f, dt=%.4fs, Offsets(X:%.3f Y:%.3f Z:%.3f)",
             alpha, sample_period_s, gyro_offset_x_dps, gyro_offset_y_dps, gyro_offset_z_dps);
}

void OrientationEstimator::reset() {
    portENTER_CRITICAL(&m_snapshotMux);
    const auto generation = m_snapshot.generation + 1;
    const auto sequence = m_snapshot.sample_sequence;
    const auto lost = m_snapshot.lostSamples;
    const bool uncertain = m_snapshot.lossCountUncertain || m_snapshot.sample_timestamp_us > 0;
    m_snapshot = {};
    m_snapshot.lostSamples = lost;
    m_snapshot.lossCountUncertain = uncertain;
    m_snapshot.generation = generation;
    m_snapshot.sample_sequence = sequence;
    portEXIT_CRITICAL(&m_snapshotMux);
    m_pitch_deg = 0.0f;
    m_pitch_rate_dps = 0.0f;
    m_yaw_deg = 0.0f;
    m_yaw_rate_dps = 0.0f;
    m_pitch_bias_dps = 0.0f;
    m_p00 = 1.0f;
    m_p01 = 0.0f;
    m_p10 = 0.0f;
    m_p11 = 1.0f;
    m_has_estimate = false;
    
    ESP_LOGD(TAG, "Orientation Estimator state reset.");
}

bool OrientationEstimator::processSample(float ax_g, float ay_g, float az_g,
                                        float raw_gyro_dps_x, float raw_gyro_dps_y, float raw_gyro_dps_z,
                                        int64_t sampleTimestampUs, uint32_t generation,
                                        const IMUSampleMetadata& metadata)
{
    if (!std::isfinite(ax_g) || !std::isfinite(ay_g) || !std::isfinite(az_g) ||
        !std::isfinite(raw_gyro_dps_x) || !std::isfinite(raw_gyro_dps_y) || !std::isfinite(raw_gyro_dps_z) ||
        sampleTimestampUs <= 0 || generation != getOrientation().generation) return false;
    // Suppress unused parameter warning for raw_gyro_dps_x (kept for interface compatibility)
    (void)raw_gyro_dps_x;
    const float current_pitch_deg_local = m_pitch_deg;
    const float gy_offset_local = m_gyro_offset_y_dps;
    const float gz_offset_local = m_gyro_offset_z_dps;
    const float sample_period_s_local = m_sample_period_s;
    const float alpha_local = m_alpha;

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

    if (accel_pitch_valid && !m_has_estimate) {
        m_pitch_deg = accel_pitch_deg;
        m_has_estimate = true;
    }

    float angle_deg = m_pitch_deg;
    float bias_dps = m_pitch_bias_dps;
    float yaw_deg = m_yaw_deg;
    float p00 = m_p00;
    float p01 = m_p01;
    float p10 = m_p10;
    float p11 = m_p11;

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

    m_pitch_deg = angle_deg;
    m_pitch_rate_dps = latest_pitch_rate_dps;
    m_yaw_deg = yaw_deg;
    m_yaw_rate_dps = latest_yaw_rate_dps;
    m_pitch_bias_dps = bias_dps;
    m_p00 = p00;
    m_p01 = p01;
    m_p10 = p10;
    m_p11 = p11;

    if (!m_has_estimate || !std::isfinite(angle_deg) || !std::isfinite(yaw_deg) ||
        !std::isfinite(latest_pitch_rate_dps) || !std::isfinite(latest_yaw_rate_dps)) return false;
    portENTER_CRITICAL(&m_snapshotMux);
    if (m_snapshot.generation != generation) {
        portEXIT_CRITICAL(&m_snapshotMux);
        return false;
    }
    m_snapshot.ax_g = ax_g;
    m_snapshot.ay_g = ay_g;
    m_snapshot.az_g = az_g;
    m_snapshot.saturationMask = metadata.saturationMask;
    m_snapshot.fifoRemainingPackets = metadata.fifoRemainingPackets;
    m_snapshot.fifoState = metadata.fifoRemainingPackets ? SensorFIFOState::BACKLOG : SensorFIFOState::STREAMING;
    m_snapshot.pitch_deg = angle_deg;
    m_snapshot.pitch_rate_dps = latest_pitch_rate_dps;
    m_snapshot.yaw_deg = yaw_deg;
    m_snapshot.yaw_rate_dps = latest_yaw_rate_dps;
    m_snapshot.sample_timestamp_us = std::max(sampleTimestampUs, m_snapshot.sample_timestamp_us + 1);
    ++m_snapshot.sample_sequence;
    portEXIT_CRITICAL(&m_snapshotMux);
    return true;
}

void OrientationEstimator::recordFifoLoss(uint64_t knownDiscarded, bool uncertain) {
    portENTER_CRITICAL(&m_snapshotMux);
    m_snapshot.lostSamples += knownDiscarded;
    m_snapshot.lossCountUncertain |= uncertain;
    m_snapshot.fifoState = SensorFIFOState::DISCONTINUITY;
    m_snapshot.valid = false;
    portEXIT_CRITICAL(&m_snapshotMux);
}

void OrientationEstimator::setValidated() {
    portENTER_CRITICAL(&m_snapshotMux);
    m_snapshot.valid = m_snapshot.sample_timestamp_us > 0;
    portEXIT_CRITICAL(&m_snapshotMux);
}

float OrientationEstimator::getPitchDeg() const { return getOrientation().pitch_deg; }

float OrientationEstimator::getPitchRateDPS() const { return getOrientation().pitch_rate_dps; }

float OrientationEstimator::getYawDeg() const { return getOrientation().yaw_deg; }

float OrientationEstimator::getYawRateDPS() const { return getOrientation().yaw_rate_dps; }

OrientationEstimate OrientationEstimator::getOrientation() const {
    portENTER_CRITICAL(&m_snapshotMux);
    const auto snapshot = m_snapshot;
    portEXIT_CRITICAL(&m_snapshotMux);
    return snapshot;
}

std::pair<float, float> OrientationEstimator::getPitchAndYawRate() const {
    const auto snapshot = getOrientation();
    return {snapshot.pitch_deg, snapshot.yaw_rate_dps};
}
