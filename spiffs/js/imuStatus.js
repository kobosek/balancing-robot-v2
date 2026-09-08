import { TELEMETRY_FORMAT_VERSION } from './constants.js';

export const IMU_GRAPH_KEYS = ['pitchDeg', 'desiredAngleDeg', 'yawAngleDeg', 'targetYawAngleDeg', 'yawRateDPS', 'targetYawRateDPS'];
export function normalizeImuStatus(value) {
    if (!value || typeof value.ready !== 'boolean') return { state: 'UNKNOWN', ready: null, sample_age_ms: null, generation: null };
    return { ...value, state: typeof value.state === 'string' ? value.state : 'UNKNOWN',
        sample_age_ms: Number.isFinite(value.sample_age_ms) && value.sample_age_ms >= 0 ? value.sample_age_ms : null };
}
export function imuStatusText(value) {
    const imu = normalizeImuStatus(value);
    if (imu.ready === null) return 'UNKNOWN / DISCONNECTED';
    const state = imu.ready ? 'READY' : imu.state;
    const age = imu.sample_age_ms === null ? '' : ' · ' + imu.sample_age_ms.toFixed(1) + ' ms';
    return state + age + (imu.configuration_pending ? ' · configuration pending' : '');
}
export function decodeTelemetryPoint(point, version) {
    if (!Array.isArray(point) || point.length < 12) return null;
    if (version !== undefined && version !== 2 && version !== TELEMETRY_FORMAT_VERSION) return null;
    if (version >= 2 && (point.length < 15 || typeof point[12] !== 'boolean' ||
        !Number.isInteger(point[14]) || point[14] < 0)) return null;
    if (version === 3 && (point.length < 18 ||
        [15, 16, 17].some(index => typeof point[index] !== 'boolean'))) return null;
    const data = Object.fromEntries(['pitchDeg', 'speedLDPS', 'speedRDPS', 'batteryVoltage', 'systemState',
        'speedSetpointLDPS', 'speedSetpointRDPS', 'desiredAngleDeg', 'yawAngleDeg', 'targetYawAngleDeg',
        'yawRateDPS', 'targetYawRateDPS'].map((key, index) => [key, Number.isFinite(point[index]) ? point[index] : null]));
    data.imuValid = version >= 2 ? point[12] : null;
    data.imuAgeMs = version >= 2 && Number.isFinite(point[13]) && point[13] >= 0 ? point[13] : null;
    data.imuGeneration = version >= 2 ? point[14] : null;
    if (data.imuValid === false) IMU_GRAPH_KEYS.forEach(key => { data[key] = null; });
    data.encoderLeftValid = version === 3 ? point[15] : null;
    data.encoderRightValid = version === 3 ? point[16] : null;
    data.imuSampleRepeated = version === 3 ? point[17] : null;
    if (data.encoderLeftValid === false) data.speedLDPS = null;
    if (data.encoderRightValid === false) data.speedRDPS = null;
    return data;
}
