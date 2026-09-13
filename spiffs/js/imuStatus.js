import { TELEMETRY_FORMAT_VERSION, TELEMETRY_V4, TELEMETRY_V4_MIN_LENGTH } from './constants.js';

export const IMU_GRAPH_KEYS = ['pitchDeg', 'targetPitchDeg', 'yawAngleDeg', 'targetYawAngleDeg', 'yawRateDPS', 'targetYawRateDPS'];
export const LONGITUDINAL_GRAPH_KEYS = [
    'positionM', 'holdPositionM', 'positionErrorM',
    'targetVelocityMps', 'measuredVelocityMps', 'commandVelocityMps',
    'distanceDifferenceM', 'distanceDifferenceTargetM', 'syncVelocityDifferenceMps',
    'requestedBalanceEffort', 'balanceEffort', 'requestedSyncEffort',
    'syncEffort', 'leftEffort', 'rightEffort'
];
export const TELEMETRY_BREAK_KEYS = [...new Set([...IMU_GRAPH_KEYS, ...LONGITUDINAL_GRAPH_KEYS])];

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

const LEGACY_KEYS = ['pitchDeg', 'speedLDPS', 'speedRDPS', 'batteryVoltage', 'systemState',
    'speedSetpointLDPS', 'speedSetpointRDPS', 'desiredAngleDeg', 'yawAngleDeg', 'targetYawAngleDeg',
    'yawRateDPS', 'targetYawRateDPS'];
const V4_BOOLEAN_FIELDS = [
    'CONTROL_VALID', 'TARGET_PITCH_VALID', 'TARGET_PITCH_CLAMPED', 'TARGET_PITCH_RATE_LIMITED',
    'POSITION_TARGET_VALID', 'POSITION_HOLD_ACTIVE', 'SYNCHRONIZATION_TARGET_VALID',
    'VELOCITY_LOOP_ENABLED', 'POSITION_LOOP_ENABLED', 'SYNCHRONIZATION_ENABLED',
    'MOTION_COMMAND_VALID', 'MOTION_COMMAND_FRESH', 'VELOCITY_FEEDBACK_VALID',
    'VELOCITY_TARGET_CLAMPED', 'VELOCITY_OUTPUT_SATURATED', 'VELOCITY_ANTI_WINDUP',
    'PITCH_PID_SATURATED', 'BALANCE_SATURATED', 'MIXER_SATURATED', 'SYNC_LIMITED',
    'MOTION_REQUEST_LIMITED', 'YAW_TARGET_VALID', 'YAW_CONTROL_AVAILABLE'
];
const V4_FLOAT_FIELDS = [
    'TARGET_PITCH_DEG', 'TARGET_VELOCITY_MPS', 'COMMAND_VELOCITY_MPS', 'MEASURED_VELOCITY_MPS',
    'HOLD_VELOCITY_REQUEST_MPS', 'HOLD_VELOCITY_TARGET_MPS', 'POSITION_M', 'HOLD_POSITION_M',
    'POSITION_ERROR_M', 'DISTANCE_DIFFERENCE_M', 'DISTANCE_DIFFERENCE_TARGET_M',
    'SYNC_VELOCITY_DIFFERENCE_MPS', 'REQUESTED_BALANCE_EFFORT', 'BALANCE_EFFORT',
    'REQUESTED_SYNC_EFFORT', 'SYNC_EFFORT', 'LEFT_EFFORT', 'RIGHT_EFFORT'
];

function isDecimalId(value) {
    return typeof value === 'string' && /^[0-9]+$/.test(value);
}

function finiteOrNull(value) {
    return Number.isFinite(value) ? value : null;
}

function appendV4Fields(data, point) {
    const v4 = TELEMETRY_V4;
    data.timestampUs = finiteOrNull(point[v4.TIMESTAMP_US]);
    data.strategyId = point[v4.STRATEGY_ID];
    data.strategyName = point[v4.STRATEGY_ID] === 0 ? 'nested_pid' :
        (point[v4.STRATEGY_ID] === 1 ? 'longitudinal_cascade' : 'unknown');
    data.loopMode = point[v4.LOOP_MODE];
    data.strategyRevision = point[v4.STRATEGY_REVISION];
    data.configRevision = point[v4.CONFIG_REVISION];
    data.commandSessionId = point[v4.COMMAND_SESSION_ID];
    data.controlGeneration = point[v4.CONTROL_GENERATION];
    data.odometryGeneration = point[v4.ODOMETRY_GENERATION];
    data.odometrySequence = point[v4.ODOMETRY_SEQUENCE];
    data.phase = point[v4.PHASE];
    data.controlValid = point[v4.CONTROL_VALID];
    data.targetPitchValid = point[v4.TARGET_PITCH_VALID];
    data.targetPitchClamped = point[v4.TARGET_PITCH_CLAMPED];
    data.targetPitchRateLimited = point[v4.TARGET_PITCH_RATE_LIMITED];
    data.positionTargetValid = point[v4.POSITION_TARGET_VALID];
    // Keep the operator-facing name from the plan while retaining the more
    // specific wire flag used by the strategy diagnostics.
    data.holdTargetValid = data.positionTargetValid;
    data.positionHoldActive = point[v4.POSITION_HOLD_ACTIVE];
    data.synchronizationTargetValid = point[v4.SYNCHRONIZATION_TARGET_VALID];
    data.velocityLoopEnabled = point[v4.VELOCITY_LOOP_ENABLED];
    data.positionLoopEnabled = point[v4.POSITION_LOOP_ENABLED];
    data.synchronizationEnabled = point[v4.SYNCHRONIZATION_ENABLED];
    data.motionCommandValid = point[v4.MOTION_COMMAND_VALID];
    data.motionCommandFresh = point[v4.MOTION_COMMAND_FRESH];
    data.velocityFeedbackValid = point[v4.VELOCITY_FEEDBACK_VALID];
    data.velocityTargetClamped = point[v4.VELOCITY_TARGET_CLAMPED];
    data.velocityOutputSaturated = point[v4.VELOCITY_OUTPUT_SATURATED];
    data.velocityAntiWindup = point[v4.VELOCITY_ANTI_WINDUP];
    data.pitchPidSaturated = point[v4.PITCH_PID_SATURATED];
    data.balanceSaturated = point[v4.BALANCE_SATURATED];
    data.mixerSaturated = point[v4.MIXER_SATURATED];
    data.syncLimited = point[v4.SYNC_LIMITED];
    data.motionRequestLimited = point[v4.MOTION_REQUEST_LIMITED];
    data.phaseReason = point[v4.PHASE_REASON];
    data.targetPitchDeg = finiteOrNull(point[v4.TARGET_PITCH_DEG]);
    data.targetVelocityMps = finiteOrNull(point[v4.TARGET_VELOCITY_MPS]);
    data.commandVelocityMps = finiteOrNull(point[v4.COMMAND_VELOCITY_MPS]);
    data.measuredVelocityMps = finiteOrNull(point[v4.MEASURED_VELOCITY_MPS]);
    data.holdVelocityRequestMps = finiteOrNull(point[v4.HOLD_VELOCITY_REQUEST_MPS]);
    data.holdVelocityTargetMps = finiteOrNull(point[v4.HOLD_VELOCITY_TARGET_MPS]);
    data.positionM = finiteOrNull(point[v4.POSITION_M]);
    data.holdPositionM = finiteOrNull(point[v4.HOLD_POSITION_M]);
    data.positionErrorM = finiteOrNull(point[v4.POSITION_ERROR_M]);
    data.distanceDifferenceM = finiteOrNull(point[v4.DISTANCE_DIFFERENCE_M]);
    data.distanceDifferenceTargetM = finiteOrNull(point[v4.DISTANCE_DIFFERENCE_TARGET_M]);
    data.syncVelocityDifferenceMps = finiteOrNull(point[v4.SYNC_VELOCITY_DIFFERENCE_MPS]);
    data.requestedBalanceEffort = finiteOrNull(point[v4.REQUESTED_BALANCE_EFFORT]);
    data.balanceEffort = finiteOrNull(point[v4.BALANCE_EFFORT]);
    data.requestedSyncEffort = finiteOrNull(point[v4.REQUESTED_SYNC_EFFORT]);
    data.syncEffort = finiteOrNull(point[v4.SYNC_EFFORT]);
    data.leftEffort = finiteOrNull(point[v4.LEFT_EFFORT]);
    data.rightEffort = finiteOrNull(point[v4.RIGHT_EFFORT]);
    data.yawTargetValid = point[v4.YAW_TARGET_VALID];
    data.yawControlAvailable = point[v4.YAW_CONTROL_AVAILABLE];
    // These fields were appended after the original 70-element v4 contract;
    // keep them optional so a mixed firmware/UI pair can still decode the
    // stable prefix while exposing the richer diagnostics when available.
    if (point.length > v4.MOTOR_COMMIT_ATTEMPTED &&
        typeof point[v4.MOTOR_COMMIT_ATTEMPTED] === 'boolean') {
        data.motorCommitAttempted = point[v4.MOTOR_COMMIT_ATTEMPTED];
    } else data.motorCommitAttempted = null;
    if (point.length > v4.MOTOR_COMMIT_SUCCEEDED &&
        typeof point[v4.MOTOR_COMMIT_SUCCEEDED] === 'boolean') {
        data.motorCommitSucceeded = point[v4.MOTOR_COMMIT_SUCCEEDED];
    } else data.motorCommitSucceeded = null;
    data.motorCommitResult = point.length > v4.MOTOR_COMMIT_RESULT &&
        Number.isInteger(point[v4.MOTOR_COMMIT_RESULT])
        ? point[v4.MOTOR_COMMIT_RESULT] : null;
    data.faultReason = point.length > v4.FAULT_REASON &&
        Number.isInteger(point[v4.FAULT_REASON])
        ? point[v4.FAULT_REASON] : null;
    data.faultLatched = point.length > v4.FAULT_LATCHED &&
        typeof point[v4.FAULT_LATCHED] === 'boolean'
        ? point[v4.FAULT_LATCHED] : null;
    data.imuSampleSequence = point.length > v4.IMU_SAMPLE_SEQUENCE &&
        isDecimalId(point[v4.IMU_SAMPLE_SEQUENCE])
        ? point[v4.IMU_SAMPLE_SEQUENCE] : null;
    data.controlStepCostUs = point.length > v4.CONTROL_STEP_COST_US &&
        Number.isInteger(point[v4.CONTROL_STEP_COST_US]) &&
        point[v4.CONTROL_STEP_COST_US] >= 0
        ? point[v4.CONTROL_STEP_COST_US] : null;
    data.controlStepLate = point.length > v4.CONTROL_STEP_LATE &&
        typeof point[v4.CONTROL_STEP_LATE] === 'boolean'
        ? point[v4.CONTROL_STEP_LATE] : null;
    // NestedPid owns wheel-speed targets; the longitudinal cascade owns a
    // body-velocity target and intentionally has no per-wheel target here.
    data.wheelTargetsValid = data.strategyName === 'nested_pid' && data.controlValid;
}

export function decodeTelemetryPoint(point, version) {
    if (!Array.isArray(point) || point.length < 12) return null;
    if (version !== undefined && version !== 2 && version !== 3 && version !== TELEMETRY_FORMAT_VERSION) return null;
    if (version >= 2 && (point.length < 15 || typeof point[12] !== 'boolean' ||
        !Number.isInteger(point[14]) || point[14] < 0)) return null;
    if (version === 3 && (point.length < 18 ||
        [15, 16, 17].some(index => typeof point[index] !== 'boolean'))) return null;
    if (version === TELEMETRY_FORMAT_VERSION) {
        const v4 = TELEMETRY_V4;
        if (point.length < TELEMETRY_V4_MIN_LENGTH ||
            !Number.isFinite(point[v4.TIMESTAMP_US]) ||
            !Number.isInteger(point[v4.STRATEGY_ID]) ||
            (point[v4.STRATEGY_ID] !== 0 && point[v4.STRATEGY_ID] !== 1) ||
            !Number.isInteger(point[v4.LOOP_MODE]) ||
            point[v4.LOOP_MODE] < -1 || point[v4.LOOP_MODE] > 2 ||
            !Number.isInteger(point[v4.STRATEGY_REVISION]) || point[v4.STRATEGY_REVISION] < 0 ||
            !Number.isInteger(point[v4.CONFIG_REVISION]) || point[v4.CONFIG_REVISION] < 0 ||
            !isDecimalId(point[v4.COMMAND_SESSION_ID]) ||
            !Number.isInteger(point[v4.CONTROL_GENERATION]) || point[v4.CONTROL_GENERATION] < 0 ||
            !Number.isInteger(point[v4.ODOMETRY_GENERATION]) || point[v4.ODOMETRY_GENERATION] < 0 ||
            !isDecimalId(point[v4.ODOMETRY_SEQUENCE]) ||
            !Number.isInteger(point[v4.PHASE]) || point[v4.PHASE] < 0 || point[v4.PHASE] > 8 ||
            !Number.isInteger(point[v4.PHASE_REASON]) || point[v4.PHASE_REASON] < 0 || point[v4.PHASE_REASON] > 10 ||
            V4_BOOLEAN_FIELDS.some(name => typeof point[v4[name]] !== 'boolean') ||
            V4_FLOAT_FIELDS.some(name => !Number.isFinite(point[v4[name]]))) return null;
    }
    const data = Object.fromEntries(LEGACY_KEYS.map((key, index) =>
        [key, Number.isFinite(point[index]) ? point[index] : null]));
    data.imuValid = version >= 2 ? point[12] : null;
    data.imuAgeMs = version >= 2 && Number.isFinite(point[13]) && point[13] >= 0 ? point[13] : null;
    data.imuGeneration = version >= 2 ? point[14] : null;
    if (data.imuValid === false) IMU_GRAPH_KEYS.forEach(key => { data[key] = null; });
    data.encoderLeftValid = version >= 3 ? point[15] : null;
    data.encoderRightValid = version >= 3 ? point[16] : null;
    data.imuSampleRepeated = version >= 3 ? point[17] : null;
    if (data.encoderLeftValid === false) data.speedLDPS = null;
    if (data.encoderRightValid === false) data.speedRDPS = null;

    if (version === TELEMETRY_FORMAT_VERSION) {
        appendV4Fields(data, point);
        if (data.imuValid === false) IMU_GRAPH_KEYS.forEach(key => { data[key] = null; });
    } else {
        data.targetPitchDeg = data.imuValid === false ? null : data.desiredAngleDeg;
        for (const key of ['timestampUs', 'strategyId', 'strategyName', 'loopMode',
            'strategyRevision', 'configRevision', 'commandSessionId', 'controlGeneration',
            'odometryGeneration', 'odometrySequence', 'phase', 'phaseReason',
            'controlValid', 'targetPitchValid', 'targetPitchClamped', 'targetPitchRateLimited',
            'positionTargetValid', 'positionHoldActive', 'synchronizationTargetValid',
            'holdTargetValid', 'wheelTargetsValid',
            'velocityLoopEnabled', 'positionLoopEnabled', 'synchronizationEnabled',
            'motionCommandValid', 'motionCommandFresh', 'velocityFeedbackValid',
            'velocityTargetClamped', 'velocityOutputSaturated', 'velocityAntiWindup',
            'pitchPidSaturated', 'balanceSaturated', 'mixerSaturated', 'syncLimited',
            'motionRequestLimited', 'targetVelocityMps', 'commandVelocityMps',
            'measuredVelocityMps', 'holdVelocityRequestMps', 'holdVelocityTargetMps',
            'positionM', 'holdPositionM', 'positionErrorM', 'distanceDifferenceM',
            'distanceDifferenceTargetM', 'syncVelocityDifferenceMps', 'requestedBalanceEffort',
            'balanceEffort', 'requestedSyncEffort', 'syncEffort', 'leftEffort', 'rightEffort',
            'yawTargetValid', 'yawControlAvailable']) data[key] = null;
        for (const key of ['motorCommitAttempted', 'motorCommitSucceeded',
            'motorCommitResult', 'faultReason', 'faultLatched',
            'imuSampleSequence', 'controlStepCostUs', 'controlStepLate']) {
            data[key] = null;
        }
    }
    return data;
}
