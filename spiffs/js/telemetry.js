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
        // --- MODIFIED: Check length needed for new data format (9 elements) ---
        if (Array.isArray(point) && point.length >= 9) {
            // Map received array elements to named properties based on NEW order
            const currentDataMap = {
                pitchDeg:           point[0], // Index 0
                speedLDPS:          point[1], // Index 1 (Actual L)
                speedRDPS:          point[2], // Index 2 (Actual R)
                batteryVoltage:     point[3], // Index 3
                systemState:        point[4], // Index 4
                speedSetpointLDPS:  point[5], // Index 5 (Setpoint L)
                speedSetpointRDPS:  point[6], // Index 6 (Setpoint R)
                desiredAngleDeg:    point[7], // Index 7 (Desired Angle)
                yawRateDPS:         point[8], // Index 8 (Yaw Rate) <<< NEW
                // Add joystick data separately
                joystickX:          appState.joystick.currentData.x,
                joystickY:          appState.joystick.currentData.y,
            };

            telemetryKeys.forEach(key => {
                if (currentDataMap.hasOwnProperty(key)) {
                     batchValuesByKey[key].push(currentDataMap[key]);
                }
            });

            latestPointMap = currentDataMap; // Store the latest mapped point
            dataUpdated = true;
        } else {
            console.warn(`Skipping invalid point array format or insufficient length (${point?.length || 'null'} < 9):`, point);
        }
    });

    if (dataUpdated) {
        updateTelemetryArrays(batchValuesByKey);
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
