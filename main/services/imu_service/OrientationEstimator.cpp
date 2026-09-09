#include "OrientationEstimator.hpp"
#include <algorithm>
#include <cmath>
#include "esp_log.h"

// Constructor implementation
OrientationEstimator::OrientationEstimator() :
    m_sample_period_s(MPU6050Profile::DEFAULT_SAMPLE_PERIOD_S),
    m_pitch_deg(0.0f),
    m_yaw_deg(0.0f),
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
    m_sample_period_s = std::isfinite(sample_period_s) && sample_period_s > 0.0f ?
        sample_period_s : MPU6050Profile::DEFAULT_SAMPLE_PERIOD_S;
    const float boundedAlpha = std::clamp(alpha, 0.0f, 0.999f);
    const float trustRatio = std::max(0.01f, boundedAlpha / std::max(1.0f - boundedAlpha, 0.001f));
    // Preserve the original alpha-to-observation-noise response at every configured rate.
    m_accelNoise = KALMAN_BASE_ACCEL_NOISE_DEG2 * trustRatio;
    m_gyro_offset_x_dps = gyro_offset_x_dps;
    m_gyro_offset_y_dps = gyro_offset_y_dps;
    m_gyro_offset_z_dps = gyro_offset_z_dps;
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
    m_yaw_deg = 0.0f;
    m_pitch_bias_dps = 0.0f;
    m_p00 = 1.0f;
    m_p01 = 0.0f;
    m_p10 = 0.0f;
    m_p11 = 1.0f;
    m_has_estimate = false;
    m_initializationSamples = 0;
    m_gyroContinuityLost = false;
    
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
    const float current_pitch_deg_local = m_pitch_deg;
    const float gy_offset_local = m_gyro_offset_y_dps;
    const float gz_offset_local = m_gyro_offset_z_dps;
    const float sample_period_s_local = m_sample_period_s;

    // Keep the original sensor Y/Z rate convention.
    float gyro_dps_y = raw_gyro_dps_y - gy_offset_local;
    float gyro_dps_z = raw_gyro_dps_z - gz_offset_local;

    // --- Bias-aware Kalman observer for pitch ---
    float accel_pitch_deg = current_pitch_deg_local;
    bool accel_pitch_valid = false;
    const float yz_mag_sq = (ay_g * ay_g) + (az_g * az_g);
    const float accel_mag_g = std::sqrt(ax_g * ax_g + yz_mag_sq);
    const float accel_deviation_g = std::fabs(accel_mag_g - 1.0f);
    const bool accelUsable = std::isfinite(accel_mag_g) && accel_mag_g > 1e-6f &&
        !(metadata.saturationMask & 0x07);
    if (accelUsable) {
        accel_pitch_deg = std::atan2(-ax_g, std::sqrt(yz_mag_sq)) * RAD_TO_DEG;
        accel_pitch_valid = true;
    }

    float dt = sample_period_s_local;
    if (dt <= 0) {
         ESP_LOGW(TAG,
                  "Invalid sample period (%.4f) in estimator, using %.4fs",
                  dt,
                  MPU6050Profile::DEFAULT_SAMPLE_PERIOD_S);
         dt = MPU6050Profile::DEFAULT_SAMPLE_PERIOD_S;
     }

    const bool initializationUsable = accel_pitch_valid && accel_deviation_g <= ACCEL_REJECTION_THRESHOLD_G &&
        !(metadata.saturationMask & 0x38);
    m_gyroContinuityLost |= (metadata.saturationMask & 0x38) != 0;
    if (m_initializationSamples < 5) {
        m_initializationSamples = initializationUsable ? m_initializationSamples + 1 : 0;
    }
    const bool firstEstimate = initializationUsable && !m_has_estimate;
    if (firstEstimate) {
        m_pitch_deg = accel_pitch_deg;
        m_has_estimate = true;
    }

    float angle_deg = m_pitch_deg;
    float bias_dps = m_pitch_bias_dps;
    double yaw_deg = m_yaw_deg;
    float p00 = m_p00;
    float p01 = m_p01;
    float p10 = m_p10;
    float p11 = m_p11;

    // Preserve the established sensor-axis convention used by the controller.
    const float unbiased_rate_dps = gyro_dps_y - bias_dps;
    if (m_has_estimate && !firstEstimate && !m_gyroContinuityLost)
        angle_deg += dt * unbiased_rate_dps;

    p00 += dt * ((dt * p11) - p01 - p10 + KALMAN_PROCESS_NOISE_ANGLE);
    p01 -= dt * p11;
    p10 -= dt * p11;
    p11 += KALMAN_PROCESS_NOISE_BIAS * dt;

    const float innovation = accel_pitch_deg - angle_deg;
    if (m_has_estimate && !m_gyroContinuityLost && accel_pitch_valid &&
        accel_deviation_g <= ACCEL_REJECTION_THRESHOLD_G) {
        const float dynamicScale = 1.0f +
            std::max(0.0f, accel_deviation_g - ACCEL_TRUST_DEADBAND_G) *
            (MAX_ACCEL_NOISE_SCALE - 1.0f) /
            std::max(ACCEL_REJECTION_THRESHOLD_G - ACCEL_TRUST_DEADBAND_G, 0.001f);
        const float measurementNoise = std::max(MIN_ACCEL_NOISE_DEG2, m_accelNoise * dynamicScale);

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
    const float latest_yaw_rate_dps = gyro_dps_z;
    if (m_has_estimate && !m_gyroContinuityLost) yaw_deg += static_cast<double>(dt) * latest_yaw_rate_dps;

    m_pitch_deg = angle_deg;
    m_yaw_deg = yaw_deg;
    m_pitch_bias_dps = bias_dps;
    m_p00 = p00;
    m_p01 = p01;
    m_p10 = p10;
    m_p11 = p11;

    if (!std::isfinite(angle_deg) || !std::isfinite(yaw_deg) ||
        !std::isfinite(latest_pitch_rate_dps) || !std::isfinite(latest_yaw_rate_dps)) return false;
    // Sensor +Z inclination is diagnostic only: mounting relative to the robot
    // has not been calibrated, so it must not gate motion or change pitch signs.
    const float tiltDeg = accelUsable ?
        std::acos(std::clamp(az_g / accel_mag_g, -1.0f, 1.0f)) * RAD_TO_DEG : 180.0f;
    portENTER_CRITICAL(&m_snapshotMux);
    if (m_snapshot.generation != generation) {
        portEXIT_CRITICAL(&m_snapshotMux);
        return false;
    }
    m_snapshot.ax_g = ax_g;
    m_snapshot.ay_g = ay_g;
    m_snapshot.az_g = az_g;
    m_snapshot.saturationMask = metadata.saturationMask;
    m_snapshot.gyroContinuityLost = m_gyroContinuityLost;
    if (!m_has_estimate || m_gyroContinuityLost) m_snapshot.valid = false;
    m_snapshot.tilt_deg = tiltDeg;
    m_snapshot.gravityReferenceValid = initializationUsable;
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

bool OrientationEstimator::setValidated() {
    portENTER_CRITICAL(&m_snapshotMux);
    m_snapshot.valid = m_snapshot.sample_timestamp_us > 0 && m_has_estimate &&
        m_initializationSamples >= 5 && !m_gyroContinuityLost;
    const bool valid = m_snapshot.valid;
    portEXIT_CRITICAL(&m_snapshotMux);
    return valid;
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
