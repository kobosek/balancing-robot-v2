import { decodeTelemetryPoint, IMU_GRAPH_KEYS } from './imuStatus.js';
import { appState, updateTelemetryJsonCache, updateTelemetryArrays, updateBatteryState } from './state.js';
import { fetchDataApi } from './api.js';
import { drawAllGraphs } from './graph.js';
import { updateLegendUI, updateBatteryUI } from './ui.js';

export async function updateTelemetryData() {
    const rawResponse = await fetchDataApi();
    if (!rawResponse) return;

    updateTelemetryJsonCache(rawResponse);

    if (!rawResponse.data || !Array.isArray(rawResponse.data)) {
        console.warn("Invalid telemetry response structure:", rawResponse);
        return;
    }

    const batchData = rawResponse.data;
    if (batchData.length === 0) return;

    let dataUpdated = false;
    let latestPointMap = null;
    const telemetryKeys = Object.keys(appState.telemetryData);
    const batchValuesByKey = Object.fromEntries(telemetryKeys.map(key => [key, []]));

    batchData.forEach((point) => {
        const currentDataMap = decodeTelemetryPoint(point, rawResponse.format_version);
        if (currentDataMap) {
            const generation = currentDataMap.imuGeneration;
            if (generation !== null && generation !== appState.telemetryImuGeneration) {
                if (appState.telemetryImuGeneration !== null) {
                    // Break the connecting segment, retaining all earlier history
                    // and the common sample positions of the other graph series.
                    IMU_GRAPH_KEYS.forEach(key => {
                        const values = batchValuesByKey[key]?.length ? batchValuesByKey[key] : appState.telemetryData[key];
                        if (values?.length) values[values.length - 1] = null;
                    });
                }
                appState.telemetryImuGeneration = generation;
            }
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
        }
    });

    if (dataUpdated) {
        updateTelemetryArrays(batchValuesByKey, true);
    }

    // Update Legend and Battery Status using the LATEST point's mapped data
    if (latestPointMap) {
        updateLegendUI(latestPointMap); // Update legends with the mapped data

        const battVoltage = parseFloat(latestPointMap.batteryVoltage);
         if (!isNaN(battVoltage)) {
            const vMax = appState.configDataCache?.battery?.voltage_max || 4.2;
            const vMin = appState.configDataCache?.battery?.voltage_min || 3.3;
            const vRange = vMax - vMin;
            const percentage = vRange > 0.1 ? Math.max(0, Math.min(100, Math.round(((battVoltage - vMin) / vRange) * 100))) : (battVoltage >= vMax ? 100 : 0);
            updateBatteryState(battVoltage, percentage); updateBatteryUI(battVoltage, percentage);
        } else { updateBatteryState(0, 0); updateBatteryUI(NaN, NaN); }
    } else { updateBatteryState(0, 0); updateBatteryUI(NaN, NaN); }

    if (dataUpdated) { drawAllGraphs(); }
}
