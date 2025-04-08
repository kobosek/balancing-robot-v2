// js/telemetry.js
import { appState, updateTelemetryJsonCache, updateTelemetryArray, updateBatteryState } from './state.js';
import { fetchDataApi } from './api.js';
import { drawGraph } from './graph.js';
import { updateLegendUI, updateBatteryUI } from './ui.js';

export async function updateTelemetryData() {
    const rawResponse = await fetchDataApi();
    if (!rawResponse) return; // Exit if fetch failed

    updateTelemetryJsonCache(rawResponse); // Cache the raw response

    // Validate response structure
    if (!rawResponse.data || !Array.isArray(rawResponse.data) || !rawResponse.hasOwnProperty('interval_ms')) {
        console.warn("Invalid telemetry response structure:", rawResponse);
        return;
    }

    const batchData = rawResponse.data;
    if (batchData.length === 0) return; // No new data points

    let dataUpdated = false;
    let latestPoint = null;

    batchData.forEach((point) => {
        if (Array.isArray(point) && point.length >= 9) {
             // Push data into respective arrays using the state function
            const dataMap = [point[0], point[1], point[2], point[3], point[4], point[5], point[6]];
            appState.telemetryKeys.forEach((key, index) => {
                if (index < dataMap.length) {
                    updateTelemetryArray(key, dataMap[index]);
                } else {
                    updateTelemetryArray(key, null); // Handle potential length mismatch
                }
            });
            latestPoint = point; // Update latest point
            dataUpdated = true;
        } else {
            console.warn("Skipping invalid point array format:", point);
        }
    });

    // Update Legend and Battery Status using the LATEST point
    if (latestPoint) {
        updateLegendUI(latestPoint);

        // Update Battery State (Index 7)
        const battVoltage = parseFloat(latestPoint[7]);
        if (!isNaN(battVoltage)) {
            const vMax = appState.configDataCache?.battery?.voltage_max || 4.2;
            const vMin = appState.configDataCache?.battery?.voltage_min || 3.3;
            const vRange = vMax - vMin;
            const percentage = vRange > 0.1
                ? Math.max(0, Math.min(100, Math.round(((battVoltage - vMin) / vRange) * 100)))
                : (battVoltage >= vMax ? 100 : 0);

            updateBatteryState(battVoltage, percentage); // Update central state
            updateBatteryUI(battVoltage, percentage);   // Update UI display
        } else {
            // Handle case where battery data might be missing or invalid
             updateBatteryState(0, 0); // Update central state
             updateBatteryUI(NaN, '--'); // Update UI display
        }
    }

    // Redraw graph if data was updated
    if (dataUpdated) {
        drawGraph();
    }
}