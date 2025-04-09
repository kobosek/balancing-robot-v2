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
        'systemStateValue', 'systemStateId', 'batteryVoltageValue', 'batteryPercentValue', 'autoRecoveryStatus', 'fallDetectStatus', 'wsStatus',
        // Command Buttons
        'startBtn', 'stopBtn', 'calibrateBtn', 'toggleRecoveryBtn', 'toggleFallDetectBtn',
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
export function updateStatusSectionUI() { const state = appState.currentSystemState; if (uiElements.systemStateValue) uiElements.systemStateValue.textContent = state.state_name || 'UNKNOWN'; if (uiElements.systemStateId) uiElements.systemStateId.textContent = (state.state_id !== undefined && state.state_id !== null) ? state.state_id : '?'; updateRecoveryButtonUI(); updateFallDetectButtonUI(); const batt = appState.currentBattery; updateBatteryUI(batt.voltage, batt.percentage); }
export function updateRecoveryButtonUI() { const button = uiElements.toggleRecoveryBtn; const statusEl = uiElements.autoRecoveryStatus; const isEnabled = !!(appState.currentSystemState?.auto_recovery_enabled); if (button) { button.textContent = isEnabled ? 'Disable Auto Recovery' : 'Enable Auto Recovery'; button.classList.toggle('enabled', isEnabled); button.disabled = false; } if (statusEl) statusEl.textContent = isEnabled ? 'ENABLED' : 'DISABLED'; }
export function updateFallDetectButtonUI() { const button = uiElements.toggleFallDetectBtn; const statusEl = uiElements.fallDetectStatus; const isEnabled = !!(appState.currentSystemState?.fall_detection_enabled); if (button) { button.textContent = isEnabled ? 'Disable Fall Detect' : 'Enable Fall Detect'; button.classList.toggle('enabled', isEnabled); button.disabled = false; } if (statusEl) statusEl.textContent = isEnabled ? 'ENABLED' : 'DISABLED'; }
export function updateBatteryUI(voltage, percentage) { const displayVoltage = !isNaN(parseFloat(voltage)) ? parseFloat(voltage).toFixed(2) : '?.??'; const displayPercentage = !isNaN(parseFloat(percentage)) ? Math.round(parseFloat(percentage)) : '??'; if (uiElements.batteryVoltageValue) uiElements.batteryVoltageValue.textContent = displayVoltage; if (uiElements.batteryPercentValue) uiElements.batteryPercentValue.textContent = displayPercentage; }
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