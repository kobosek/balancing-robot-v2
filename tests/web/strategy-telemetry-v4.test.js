import test from 'node:test';
import assert from 'node:assert/strict';
import { decodeTelemetryPoint } from '../../spiffs/js/imuStatus.js';
import { TELEMETRY_V4_MIN_LENGTH } from '../../spiffs/js/constants.js';
import { buildStrategySelectionConfig, canApplyStrategy } from '../../spiffs/js/strategyConfig.js';

function makeV4Point(overrides = {}) {
    const legacy = [10, 20, 30, 3.8, 1, 0, 0, 0, 12, 0, 1, 0,
        true, 4, 9, true, true, false];
    const point = legacy.concat([
        123456, 1, 2, 3, 4, '17', 5, 6, '42', 5,
        ...new Array(21).fill(false),
        0,
        ...new Array(18).fill(0.25),
        true, false
    ]);
    Object.entries(overrides).forEach(([index, value]) => { point[Number(index)] = value; });
    assert.equal(point.length, TELEMETRY_V4_MIN_LENGTH);
    return point;
}

test('v4 telemetry decodes strategy, longitudinal diagnostics and decimal IDs', () => {
    const data = decodeTelemetryPoint(makeV4Point(), 4);
    assert.equal(data.strategyName, 'longitudinal_cascade');
    assert.equal(data.loopMode, 2);
    assert.equal(data.commandSessionId, '17');
    assert.equal(data.odometrySequence, '42');
    assert.equal(data.targetPitchDeg, 0.25);
    assert.equal(data.positionM, 0.25);
    assert.equal(data.yawControlAvailable, false);
});

test('v4 decoder rejects malformed metadata and preserves v3 compatibility', () => {
    const point = makeV4Point();
    point[23] = 17;
    assert.equal(decodeTelemetryPoint(point, 4), null);
    const v3 = [10, 20, 30, 3.8, 1, 0, 0, 0, 12, 0, 1, 0, true, 4, 1, true, true, false];
    assert.equal(decodeTelemetryPoint(v3, 3).targetPitchDeg, 0);
});

test('strategy selection changes only the active identifier', () => {
    const config = { config_revision: 7, control: { balance_strategy: 'nested_pid', strategies: {
        active: 'nested_pid', nested_pid: { revision: 2, angle: { kp: 3 } },
        longitudinal_cascade: { revision: 4, configured: true }
    } } };
    const selected = buildStrategySelectionConfig(config, 'longitudinal_cascade');
    assert.equal(selected.control.strategies.active, 'longitudinal_cascade');
    assert.equal(selected.control.strategies.nested_pid.angle.kp, 3);
    assert.equal(selected.control.strategies.longitudinal_cascade.revision, 4);
    assert.equal(config.control.strategies.active, 'nested_pid');
    assert.equal(canApplyStrategy({ state_name: 'IDLE', active_balance_strategy: 'nested_pid', strategy_capabilities: {
        longitudinal_cascade: { can_activate: true }
    } }, 'longitudinal_cascade'), true);
});
