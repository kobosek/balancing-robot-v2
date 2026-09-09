import { decodeTelemetryPoint, imuStatusText, normalizeImuStatus } from '../../spiffs/js/imuStatus.js';
import { appState, updateCurrentSystemState, updateTelemetryArrays } from '../../spiffs/js/state.js';
import { updateTelemetryData } from '../../spiffs/js/telemetry.js';
import { fetchStateApi } from '../../spiffs/js/api.js';
import { uiElements, updateStatusSectionUI } from '../../spiffs/js/ui.js';

export async function runTests() {
    let count = 0;
    const check = (condition, message) => { if (!condition) throw new Error(message); ++count; };
    const oldDocument = globalThis.document;
    if (!oldDocument) globalThis.document = { getElementById: () => null };
    const oldFetch = globalThis.fetch;
    const oldError = console.error;
    const point = [10, 20, 30, 3.8, 1, 0, 0, 0, 12, 0, 1, 0];
    try {
        const diagnostics = normalizeImuStatus({ ready: true, sample_sequence: 123,
            fifo_remaining_packets: 4, gyro_clipping_resets: 2 });
        check(diagnostics.sample_sequence === 123 && diagnostics.fifo_remaining_packets === 4 &&
            diagnostics.gyro_clipping_resets === 2, 'acquisition diagnostics survive state normalization');
        check(decodeTelemetryPoint(point).imuValid === null, 'legacy validity must be unknown');
        check(decodeTelemetryPoint(point, 99) === null, 'future format rejected');
        check(decodeTelemetryPoint(point, 2) === null, 'short version 2 rejected');
        check(decodeTelemetryPoint([...point, true, 4, 1], 3) === null, 'short version 3 rejected');
        const encoderInvalid = decodeTelemetryPoint([...point, true, 4, 1, false, true, true], 3);
        check(encoderInvalid.speedLDPS === null && encoderInvalid.speedRDPS === 30, 'independent encoder validity gaps');
        check(encoderInvalid.pitchDeg === 10 && encoderInvalid.imuSampleRepeated === true, 'repeated sample is explicit and preserves valid IMU');
        check(decodeTelemetryPoint([...point, true, 4, 1, 0, true, false], 3) === null, 'malformed encoder flags rejected');
        const invalid = decodeTelemetryPoint([...point, false, null, 4], 2);
        check(invalid.pitchDeg === null && invalid.yawRateDPS === null, 'invalid orientation gaps');
        check(invalid.speedLDPS === 20 && invalid.batteryVoltage === 3.8, 'independent sensors preserved');
        check(imuStatusText(null).includes('UNKNOWN'), 'unknown status');
        check(imuStatusText({ ready: false, state: 'VALIDATING' }).includes('VALIDATING'), 'validation status');
        check(imuStatusText({ ready: true, state: 'OPERATIONAL', sample_age_ms: 4, configuration_pending: true }).includes('pending'), 'pending config visible');
        updateTelemetryArrays({ pitchDeg: [42, null, 3] }, true);
        check(appState.telemetryData.pitchDeg.at(-2) === null, 'explicit gaps do not reuse cached values');
        globalThis.fetch = async () => ({ ok: true, text: async () => JSON.stringify({ format_version: 2,
            data: [[...point, true, 4, 1], [...point, false, 8, 1], [...point, true, 3, 2]] }) });
        await updateTelemetryData();
        check(appState.telemetryData.pitchDeg.at(-1) === 10, 'new generation rendered');
        check(appState.telemetryData.pitchDeg.at(-2) === null, 'generation boundary stays gapped');
        check(appState.telemetryData.speedLDPS.at(-2) === 20, 'encoder history survives generation');
        globalThis.fetch = async () => ({ ok: true, text: async () => JSON.stringify({ format_version: 2,
            data: [[...point, true, 4, 2], [...point, true, 4, 2], [...point, true, 3, 3]] }) });
        await updateTelemetryData();
        check(appState.telemetryData.pitchDeg.at(-3) === 10, 'generation change preserves earlier valid history');
        check(appState.telemetryData.pitchDeg.at(-2) === null, 'only the connecting segment is gapped');
        check(appState.telemetryData.speedLDPS.at(-3) === 20, 'series sample positions stay aligned');
        globalThis.fetch = async () => ({ ok: true, text: async () => JSON.stringify({ format_version: 2,
            data: [[...point, true, 3, 4]] }) });
        await updateTelemetryData();
        check(appState.telemetryData.pitchDeg.at(-4) === 10, 'cross-response generation change preserves history');
        check(appState.telemetryData.pitchDeg.at(-2) === null, 'cross-response generation change breaks line');
        updateCurrentSystemState({ imu: { ready: true, state: 'OPERATIONAL', generation: 2 } });
        uiElements.imuStatusValue = { textContent: '' };
        updateStatusSectionUI();
        check(uiElements.imuStatusValue.textContent.includes('READY'), 'status row renders ready');
        console.error = () => {};
        globalThis.fetch = async () => { throw new Error('simulated disconnect'); };
        await fetchStateApi();
        check(appState.currentSystemState.imu.ready === null, 'failed fetch clears previous readiness');
        check(uiElements.imuStatusValue.textContent.includes('DISCONNECTED'), 'failed fetch renders unknown');
        return count;
    } finally {
        globalThis.fetch = oldFetch; console.error = oldError;
        if (!oldDocument) delete globalThis.document;
    }
}
