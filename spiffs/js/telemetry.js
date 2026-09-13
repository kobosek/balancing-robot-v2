import { decodeTelemetryPoint, TELEMETRY_BREAK_KEYS } from './imuStatus.js';
import { appState, updateTelemetryJsonCache, updateTelemetryArrays, updateBatteryState } from './state.js';
import { fetchDataApi } from './api.js';
import { drawAllGraphs } from './graph.js';
import { updateLegendUI, updateBatteryUI, updateLongitudinalTelemetryUI } from './ui.js';

let telemetryFetchInFlight = false;

export async function updateTelemetryData() {
    // setInterval can fire again while a slow WiFi request is still pending.
    // Overlapping requests reorder batches and make fast motion look stepped.
    if (telemetryFetchInFlight) return;
    telemetryFetchInFlight = true;
    let rawResponse;
    try {
        rawResponse = await fetchDataApi();
    } finally {
        telemetryFetchInFlight = false;
    }
    if (!rawResponse) return;

    updateTelemetryJsonCache(rawResponse);
    if (Number.isInteger(rawResponse.dropped_samples) && rawResponse.dropped_samples >= 0) {
        appState.telemetryDroppedSamples = rawResponse.dropped_samples;
    }
    if (Number.isInteger(rawResponse.remaining_samples) && rawResponse.remaining_samples >= 0) {
        appState.telemetryPendingSamples = rawResponse.remaining_samples;
    }

    if (!rawResponse.data || !Array.isArray(rawResponse.data)) {
        console.warn("Invalid telemetry response structure:", rawResponse);
        return;
    }

    const batchData = rawResponse.data;
    if (batchData.length === 0) return;

    let dataUpdated = false;
    let invalidPointSeen = false;
    let latestPointMap = null;
    const telemetryKeys = Object.keys(appState.telemetryData);
    const batchValuesByKey = Object.fromEntries(telemetryKeys.map(key => [key, []]));
    const breakConnectingSegment = (keys = TELEMETRY_BREAK_KEYS) => {
        keys.forEach(key => {
            const values = batchValuesByKey[key]?.length ? batchValuesByKey[key] : appState.telemetryData[key];
            if (values?.length) values[values.length - 1] = null;
        });
    };
    const appendInvalidPointGap = () => {
        telemetryKeys.forEach(key => batchValuesByKey[key].push(null));
        invalidPointSeen = true;
    };

    batchData.forEach((point) => {
        const currentDataMap = decodeTelemetryPoint(point, rawResponse.format_version);
        if (currentDataMap) {
            const generation = currentDataMap.imuGeneration;
            if (generation !== null && generation !== appState.telemetryImuGeneration) {
                if (appState.telemetryImuGeneration !== null) {
                    // Break the connecting segment, retaining all earlier history
                    // and the common sample positions of the other graph series.
                    breakConnectingSegment(['pitchDeg', 'targetPitchDeg', 'yawAngleDeg', 'targetYawAngleDeg', 'yawRateDPS', 'targetYawRateDPS']);
                }
                appState.telemetryImuGeneration = generation;
            }
            const strategyChanged = currentDataMap.strategyId !== null &&
                (appState.telemetryStrategyId !== null && currentDataMap.strategyId !== appState.telemetryStrategyId);
            const loopModeChanged = currentDataMap.loopMode !== null &&
                (appState.telemetryLoopMode !== null && currentDataMap.loopMode !== appState.telemetryLoopMode);
            const controlGenerationChanged = currentDataMap.controlGeneration !== null &&
                (appState.telemetryControlGeneration !== null && currentDataMap.controlGeneration !== appState.telemetryControlGeneration);
            const odometryGenerationChanged = currentDataMap.odometryGeneration !== null &&
                (appState.telemetryOdometryGeneration !== null && currentDataMap.odometryGeneration !== appState.telemetryOdometryGeneration);
            if (strategyChanged || loopModeChanged || controlGenerationChanged || odometryGenerationChanged) {
                breakConnectingSegment();
            }
            if (currentDataMap.strategyId !== null) appState.telemetryStrategyId = currentDataMap.strategyId;
            if (currentDataMap.loopMode !== null) appState.telemetryLoopMode = currentDataMap.loopMode;
            if (currentDataMap.controlGeneration !== null) appState.telemetryControlGeneration = currentDataMap.controlGeneration;
            if (currentDataMap.odometryGeneration !== null) appState.telemetryOdometryGeneration = currentDataMap.odometryGeneration;
            currentDataMap.joystickX = appState.joystick.currentData.x;
            currentDataMap.joystickY = appState.joystick.currentData.y;
            telemetryKeys.forEach(key => {
                if (currentDataMap.hasOwnProperty(key)) {
                     batchValuesByKey[key].push(currentDataMap[key]);
                }
            });

            latestPointMap = currentDataMap; // Store the latest mapped point
            dataUpdated = true;
        } else {
            console.warn(`Skipping invalid point array format or insufficient length (${point?.length || 'null'} < 12):`, point);
            appendInvalidPointGap();
        }
    });

    if (dataUpdated || invalidPointSeen) {
        updateTelemetryArrays(batchValuesByKey, true);
    }

    // Update Legend and Battery Status using the LATEST point's mapped data
    if (latestPointMap) {
        updateLegendUI(latestPointMap); // Update legends with the mapped data
        updateLongitudinalTelemetryUI(latestPointMap);

        const battVoltage = parseFloat(latestPointMap.batteryVoltage);
         if (!isNaN(battVoltage)) {
            const vMax = appState.configDataCache?.battery?.voltage_max || 4.2;
            const vMin = appState.configDataCache?.battery?.voltage_min || 3.3;
            const vRange = vMax - vMin;
            const percentage = vRange > 0.1 ? Math.max(0, Math.min(100, Math.round(((battVoltage - vMin) / vRange) * 100))) : (battVoltage >= vMax ? 100 : 0);
            updateBatteryState(battVoltage, percentage); updateBatteryUI(battVoltage, percentage);
        } else { updateBatteryState(0, 0); updateBatteryUI(NaN, NaN); }
    } else { updateBatteryState(0, 0); updateBatteryUI(NaN, NaN); }

    if (dataUpdated || invalidPointSeen) { drawAllGraphs(); }
}
