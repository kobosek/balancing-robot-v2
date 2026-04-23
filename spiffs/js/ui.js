import { appState } from './state.js';
import { GRAPH_COLORS, Y_ANGLE_RANGE_DEG, Y_EFFORT_RANGE, Y_SPEED_RANGE_DPS, Y_YAWRATE_RANGE_DPS } from './constants.js';

// --- DOM Elements Cache ---
export const uiElements = {};

// --- UI Initialization ---
export function assignElements() {
    // <<< Re-verified list includes yawRateConfigFormContainer and yawRateConfigBtn >>>
    const ids = [
        // Config & General
        'openConfigBtn', 'configMenu',
        'angleConfigBtn', 'speedLeftConfigBtn', 'speedRightConfigBtn', 'yawRateConfigBtn', 'generalConfigBtn',
        'angleConfigFormContainer', 'speedLeftConfigFormContainer', 'speedRightConfigFormContainer', 'yawRateConfigFormContainer', 'generalConfigFormContainer',
        // Status Display
        'systemStateValue', 'systemStateId', 'batteryVoltageValue', 'batteryPercentValue', 'batteryStateValue', 'batteryBarFill', 'batteryAdcPinVoltageValue', 'batteryAdcCalibrationValue', 'autoBalancingStatus', 'fallDetectStatus', 'criticalBatteryShutdownStatus', 'wsStatus',
        // Command Buttons
        'startBtn', 'stopBtn', 'calibrateBtn', 'toggleAutoBalancingBtn', 'toggleFallDetectBtn', 'toggleCriticalBatteryShutdownBtn',
        // Joystick
        'joystickZone',
        // Graphs & Legends
        'telemetryGraph1', 'legend1', 'graphContainer1',
        'telemetryGraph2', 'legend2', 'graphContainer2',
        'telemetryGraph3', 'legend3', 'graphContainer3',
    ];
    console.log("Assigning elements...");

    ids.forEach(id => {
        const el = document.getElementById(id);
        if (!el) {
            console.warn(`UI element with ID '${id}' NOT FOUND!`);
            // Assign null so checks later on will fail gracefully
            uiElements[id] = null;
        } else {
            uiElements[id] = el;
        }
    });

    // Specific setup for graph states (now includes graph 3)
    for (let i = 0; i < 3; i++) {
        const graphIndex = i + 1;
        // Ensure appState.graphs has enough placeholders
        if (!appState.graphs[i]) { appState.graphs[i] = { ctx: null, canvas: null, container: null, legendValueElements: [], dpr: 1, config: null }; }
        const graphState = appState.graphs[i];

        // Use the potentially null values from uiElements cache
        graphState.container = uiElements[`graphContainer${graphIndex}`];
        graphState.canvas = uiElements[`telemetryGraph${graphIndex}`];

        if (!graphState.container || !graphState.canvas) {
             console.warn(`Graph ${graphIndex} container or canvas element missing from uiElements cache.`);
             continue; // Skip setup for this graph if elements missing
        }

        try {
            graphState.ctx = graphState.canvas.getContext('2d');
            if (!graphState.ctx) throw new Error('Failed context');
            graphState.legendValueElements = [];
            const legendElement = uiElements[`legend${graphIndex}`]; // Use cache
            if (legendElement) {
                legendElement.querySelectorAll('.legend-value').forEach(span => {
                    graphState.legendValueElements.push(span);
                });
            } else { console.warn(`Legend element legend${graphIndex} not found in cache.`); }
        } catch (error) { console.error(`UI Init Error Graph ${graphIndex}:`, error); }
    }
    // Add checks for critical elements after assignment
    if (!uiElements.joystickZone) { console.error("CRITICAL: Joystick Zone element missing after assignment."); }
    if (!uiElements.wsStatus) { console.error("CRITICAL: WS Status element missing after assignment."); }

    console.log("UI Elements Assignment finished.");
}

// --- UI Update Functions ---
// (No changes needed in update functions, they should already check if elements exist)
export function updateWsStatusUI(text, color) { if (uiElements.wsStatus) { uiElements.wsStatus.textContent = text; uiElements.wsStatus.style.color = color; } }
export function updateStatusSectionUI() { const state = appState.currentSystemState; if (uiElements.systemStateValue) uiElements.systemStateValue.textContent = state.state_name || 'UNKNOWN'; if (uiElements.systemStateId) uiElements.systemStateId.textContent = (state.state_id !== undefined && state.state_id !== null) ? state.state_id : '?'; updateAutoBalancingButtonUI(); updateFallDetectButtonUI(); updateCriticalBatteryShutdownButtonUI(); const batt = appState.currentBattery; updateBatteryUI(batt.voltage, batt.percentage); }
export function updateAutoBalancingButtonUI() { const button = uiElements.toggleAutoBalancingBtn; const statusEl = uiElements.autoBalancingStatus; const isEnabled = !!(appState.currentSystemState?.auto_balancing_enabled); if (button) { button.textContent = isEnabled ? 'Disable Auto Balancing' : 'Enable Auto Balancing'; button.classList.toggle('enabled', isEnabled); button.disabled = false; } if (statusEl) statusEl.textContent = isEnabled ? 'ENABLED' : 'DISABLED'; }
export function updateFallDetectButtonUI() { const button = uiElements.toggleFallDetectBtn; const statusEl = uiElements.fallDetectStatus; const isEnabled = !!(appState.currentSystemState?.fall_detection_enabled); if (button) { button.textContent = isEnabled ? 'Disable Fall Detect' : 'Enable Fall Detect'; button.classList.toggle('enabled', isEnabled); button.disabled = false; } if (statusEl) statusEl.textContent = isEnabled ? 'ENABLED' : 'DISABLED'; }
export function updateCriticalBatteryShutdownButtonUI() {
    const button = uiElements.toggleCriticalBatteryShutdownBtn;
    const statusEl = uiElements.criticalBatteryShutdownStatus;
    const stateValue = appState.currentSystemState?.critical_battery_motor_shutdown_enabled;
    const hasStateValue = typeof stateValue === 'boolean';
    const isEnabled = hasStateValue ? stateValue : false;

    if (button) {
        button.textContent = hasStateValue
            ? (isEnabled ? 'Disable Crit Batt Shutdown' : 'Enable Crit Batt Shutdown')
            : 'Critical Battery Shutdown';
        button.classList.toggle('enabled', isEnabled);
        button.disabled = !hasStateValue;
    }

    if (statusEl) {
        statusEl.textContent = hasStateValue ? (isEnabled ? 'ENABLED' : 'DISABLED') : 'UNKNOWN';
    }
}
export function updateBatteryUI(voltage, percentage) {
    const stateBatteryVoltage = parseFloat(appState.currentSystemState?.battery_voltage);
    const stateBatteryPercentage = parseFloat(appState.currentSystemState?.battery_percentage);
    const stateAdcPinVoltage = parseFloat(appState.currentSystemState?.battery_adc_pin_voltage);
    const adcCalibrated = !!appState.currentSystemState?.battery_adc_calibrated;

    const numericVoltage = parseFloat(voltage);
    const numericPercentage = parseFloat(percentage);
    const hasTelemetryVoltage = !isNaN(numericVoltage) && numericVoltage > 0.05;
    const effectiveVoltage = hasTelemetryVoltage ? numericVoltage : stateBatteryVoltage;
    const effectivePercentage = hasTelemetryVoltage && !isNaN(numericPercentage) ? numericPercentage : stateBatteryPercentage;
    const hasValidVoltage = !isNaN(effectiveVoltage) && effectiveVoltage >= 0.0;
    const clampedPercentage = !isNaN(effectivePercentage) ? Math.max(0, Math.min(100, Math.round(effectivePercentage))) : 0;
    const vMin = appState.configDataCache?.battery?.voltage_min ?? 3.3;
    const vMax = appState.configDataCache?.battery?.voltage_max ?? 4.2;
    const lowWarningVoltage = vMin + (vMax - vMin) * 0.1;

    const displayVoltage = hasValidVoltage ? effectiveVoltage.toFixed(2) : '?.??';
    const displayPercentage = hasValidVoltage ? clampedPercentage : '??';
    const displayAdcPinVoltage = !isNaN(stateAdcPinVoltage) && stateAdcPinVoltage >= 0.0 ? stateAdcPinVoltage.toFixed(2) : '?.??';
    let batteryStateText = 'UNKNOWN';
    let batteryStateClass = 'unknown';

    if (hasValidVoltage) {
        if (effectiveVoltage <= vMin) {
            batteryStateText = 'CRITICAL';
            batteryStateClass = 'critical';
        } else if (effectiveVoltage <= lowWarningVoltage) {
            batteryStateText = 'LOW';
            batteryStateClass = 'low';
        } else {
            batteryStateText = 'OK';
            batteryStateClass = 'ok';
        }
    }

    if (uiElements.batteryVoltageValue) uiElements.batteryVoltageValue.textContent = displayVoltage;
    if (uiElements.batteryPercentValue) uiElements.batteryPercentValue.textContent = displayPercentage;
    if (uiElements.batteryStateValue) {
        uiElements.batteryStateValue.textContent = batteryStateText;
        uiElements.batteryStateValue.className = `battery-state ${batteryStateClass}`;
    }
    if (uiElements.batteryAdcPinVoltageValue) uiElements.batteryAdcPinVoltageValue.textContent = displayAdcPinVoltage;
    if (uiElements.batteryAdcCalibrationValue) {
        uiElements.batteryAdcCalibrationValue.textContent = adcCalibrated ? 'CAL ON' : 'CAL OFF';
        uiElements.batteryAdcCalibrationValue.className = `battery-state ${adcCalibrated ? 'ok' : 'unknown'}`;
    }
    if (uiElements.batteryBarFill) {
        uiElements.batteryBarFill.style.width = `${hasValidVoltage ? clampedPercentage : 0}%`;
        uiElements.batteryBarFill.className = `battery-bar-fill ${batteryStateClass}`;
    }
}
export function disableCommandButton(buttonElement, disable = true) { if (buttonElement) buttonElement.disabled = disable; }
export function updateLegendUI(latestPointMap) {
    const formatVal = (val, digits = 1) => { const num = parseFloat(val); return isNaN(num) ? 'N/A' : num.toFixed(digits); };
    // Graph 1
    if (appState.graphs[0]?.legendValueElements?.length >= 4) { const l = appState.graphs[0].legendValueElements; l[0].textContent = formatVal(latestPointMap?.pitchDeg, 1); l[1].textContent = formatVal(latestPointMap?.desiredAngleDeg, 1); l[2].textContent = formatVal(latestPointMap?.joystickX, 2); l[3].textContent = formatVal(latestPointMap?.joystickY, 2); }
    // Graph 2
    if (appState.graphs[1]?.legendValueElements?.length >= 4) { const l = appState.graphs[1].legendValueElements; l[0].textContent = formatVal(latestPointMap?.speedSetpointLDPS, 0); l[1].textContent = formatVal(latestPointMap?.speedSetpointRDPS, 0); l[2].textContent = formatVal(latestPointMap?.speedLDPS, 0); l[3].textContent = formatVal(latestPointMap?.speedRDPS, 0); }
    // Graph 3
    if (appState.graphs[2]?.legendValueElements?.length >= 1) { const l = appState.graphs[2].legendValueElements; l[0].textContent = formatVal(latestPointMap?.yawRateDPS, 1); }
}
