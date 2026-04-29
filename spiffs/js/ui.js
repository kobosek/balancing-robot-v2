import { appState } from './state.js';
import { GRAPH_COLORS, Y_ANGLE_RANGE_DEG, Y_EFFORT_RANGE, Y_SPEED_RANGE_DPS, Y_YAWRATE_RANGE_DPS } from './constants.js';
import { STATIC_UI_ELEMENT_IDS } from './uiElementsRegistry.js';

export const uiElements = {};

export function assignElements() {
    console.log('Assigning elements...');

    STATIC_UI_ELEMENT_IDS.forEach(id => {
        const el = document.getElementById(id);
        if (!el) {
            console.warn(`UI element with ID '${id}' NOT FOUND!`);
            uiElements[id] = null;
        } else {
            uiElements[id] = el;
        }
    });

    for (let i = 0; i < 3; i++) {
        const graphIndex = i + 1;
        if (!appState.graphs[i]) {
            appState.graphs[i] = { ctx: null, canvas: null, container: null, legendValueElements: [], dpr: 1, config: null };
        }
        const graphState = appState.graphs[i];
        graphState.container = uiElements[`graphContainer${graphIndex}`];
        graphState.canvas = uiElements[`telemetryGraph${graphIndex}`];

        if (!graphState.container || !graphState.canvas) {
            console.warn(`Graph ${graphIndex} container or canvas element missing from uiElements cache.`);
            continue;
        }

        try {
            graphState.ctx = graphState.canvas.getContext('2d');
            if (!graphState.ctx) {
                throw new Error('Failed context');
            }
            graphState.legendValueElements = [];
            const legendElement = uiElements[`legend${graphIndex}`];
            if (legendElement) {
                legendElement.querySelectorAll('.legend-value').forEach(span => {
                    graphState.legendValueElements.push(span);
                });
            } else {
                console.warn(`Legend element legend${graphIndex} not found in cache.`);
            }
        } catch (error) {
            console.error(`UI Init Error Graph ${graphIndex}:`, error);
        }
    }

    if (!uiElements.joystickZone) {
        console.error('CRITICAL: Joystick Zone element missing after assignment.');
    }
    if (!uiElements.wsStatus) {
        console.error('CRITICAL: WS Status element missing after assignment.');
    }

    console.log('UI Elements Assignment finished.');
}

export function updateWsStatusUI(text, color) { if (uiElements.wsStatus) { uiElements.wsStatus.textContent = text; uiElements.wsStatus.style.color = color; } }
export function updateStatusSectionUI() { const state = appState.currentSystemState; if (uiElements.systemStateValue) uiElements.systemStateValue.textContent = state.state_name || 'UNKNOWN'; if (uiElements.systemStateId) uiElements.systemStateId.textContent = (state.state_id !== undefined && state.state_id !== null) ? state.state_id : '?'; updateAutoBalancingButtonUI(); updateFallDetectButtonUI(); updateYawControlButtonUI(); updateCriticalBatteryShutdownButtonUI(); updateGuidedCalibrationUI(); updateOtaUI(); updatePidTuningUI(); const batt = appState.currentBattery; updateBatteryUI(batt.voltage, batt.percentage); }
export function updateAutoBalancingButtonUI() { const button = uiElements.toggleAutoBalancingBtn; const statusEl = uiElements.autoBalancingStatus; const isEnabled = !!(appState.currentSystemState?.auto_balancing_enabled); if (button) { button.textContent = isEnabled ? 'Disable Auto Balancing' : 'Enable Auto Balancing'; button.classList.toggle('enabled', isEnabled); button.disabled = false; } if (statusEl) statusEl.textContent = isEnabled ? 'ENABLED' : 'DISABLED'; }
export function updateFallDetectButtonUI() { const button = uiElements.toggleFallDetectBtn; const statusEl = uiElements.fallDetectStatus; const isEnabled = !!(appState.currentSystemState?.fall_detection_enabled); if (button) { button.textContent = isEnabled ? 'Disable Fall Detect' : 'Enable Fall Detect'; button.classList.toggle('enabled', isEnabled); button.disabled = false; } if (statusEl) statusEl.textContent = isEnabled ? 'ENABLED' : 'DISABLED'; }
export function updateYawControlButtonUI() { const button = uiElements.toggleYawControlBtn; const statusEl = uiElements.yawControlStatus; const hasStateValue = typeof appState.currentSystemState?.yaw_control_enabled === 'boolean'; const isEnabled = !!appState.currentSystemState?.yaw_control_enabled; if (button) { button.textContent = isEnabled ? 'Disable Yaw Control' : 'Enable Yaw Control'; button.classList.toggle('enabled', isEnabled); button.disabled = !hasStateValue; } if (statusEl) statusEl.textContent = hasStateValue ? (isEnabled ? 'ENABLED' : 'DISABLED') : 'UNKNOWN'; }
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
export function updateOtaUI() {
    const ota = appState.currentSystemState?.ota || {};
    const statusEl = uiElements.otaStatusValue;
    const uploadBtn = uiElements.otaUploadBtn;
    const bundleAvailable = !!ota.available && !!ota.spiffs_available;
    const updateAllowed = !!ota.update_allowed;
    const text = ota.reboot_required
        ? 'REBOOT REQUIRED'
        : (ota.update_in_progress
            ? `UPLOADING ${String(ota.active_target || '').toUpperCase()}`
            : (!updateAllowed ? 'IDLE REQUIRED' : (bundleAvailable ? (ota.message || 'READY') : 'UNAVAILABLE')));
    if (statusEl) statusEl.textContent = text;
    if (uploadBtn) {
        uploadBtn.textContent = 'Upload OTA Bundle';
        uploadBtn.disabled = !bundleAvailable || !updateAllowed || !!ota.update_in_progress;
    }
}
export function updateGuidedCalibrationUI() {
    const guided = appState.currentSystemState?.guided_calibration || {};
    const state = guided.state || 'IDLE';
    const phase = guided.phase || 'IDLE';
    const progress = Math.round(Math.max(0, Math.min(1, Number(guided.progress || 0))) * 100);
    const statusText = state === 'RUNNING'
        ? `${phase} ${progress}%`
        : (guided.message || state);
    if (uiElements.guidedCalibrationStatus) uiElements.guidedCalibrationStatus.textContent = statusText;
    const isIdle = appState.currentSystemState?.state_name === 'IDLE';
    const isRunning = state === 'RUNNING' || appState.currentSystemState?.state_name === 'GUIDED_CALIBRATION';
    if (uiElements.startGuidedCalibrationBtn) uiElements.startGuidedCalibrationBtn.disabled = !isIdle || isRunning;
    if (uiElements.cancelGuidedCalibrationBtn) uiElements.cancelGuidedCalibrationBtn.disabled = !isRunning;
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
export function updatePidTuningUI() {
    const tuning = appState.currentSystemState?.pid_tuning || {};
    const state = tuning.state || 'IDLE';
    const phase = tuning.phase || 'IDLE';
    const target = tuning.target || 'motor_speed_left';
    const progress = Math.max(0, Math.min(1, Number(tuning.progress || 0)));
    const hasCandidate = !!tuning.has_candidate;
    const isRunning = state === 'RUNNING';
    const isPreviewReady = state === 'PREVIEW_READY' && hasCandidate;
    const canStart = !isRunning && !isPreviewReady && appState.currentSystemState?.state_name === 'IDLE';
    const canCommit = state === 'PREVIEW_READY' && hasCandidate;
    const fmtGains = (pid) => pid
        ? `Kp ${Number(pid.kp || 0).toFixed(4)} / Ki ${Number(pid.ki || 0).toFixed(4)} / Kd ${Number(pid.kd || 0).toFixed(4)}`
        : 'N/A';
    const getEl = (id) => uiElements[id] || document.getElementById(id);
    const updateWheel = (side, label, candidateKey) => {
        const activeTarget = target === `motor_speed_${side}`;
        const prefix = side === 'left' ? 'leftPidTuning' : 'rightPidTuning';
        const buttonPrefix = side === 'left' ? 'Left' : 'Right';
        const displayedState = activeTarget || state === 'IDLE' ? state : `${label.toUpperCase()} INACTIVE`;
        const displayedPhase = activeTarget ? phase : 'IDLE';
        const displayedMessage = activeTarget ? (tuning.message || 'Idle') :
            (isRunning || isPreviewReady ? `${label} wheel tuning is not active` : 'Idle');
        const candidate = tuning.candidate?.[candidateKey];

        const stateEl = getEl(`${prefix}State`);
        const phaseEl = getEl(`${prefix}Phase`);
        const progressEl = getEl(`${prefix}ProgressBar`);
        const messageEl = getEl(`${prefix}Message`);
        const gainsEl = getEl(`${prefix}Gains`);
        if (stateEl) stateEl.textContent = displayedState;
        if (phaseEl) phaseEl.textContent = displayedPhase;
        if (progressEl) progressEl.style.width = `${activeTarget ? Math.round(progress * 100) : 0}%`;
        if (messageEl) messageEl.textContent = displayedMessage;
        if (gainsEl) gainsEl.textContent = activeTarget && hasCandidate ? fmtGains(candidate) : 'N/A';

        const tuneBtn = getEl(`tune${buttonPrefix}PidBtn`);
        const cancelBtn = getEl(`cancel${buttonPrefix}PidTuningBtn`);
        const saveBtn = getEl(`save${buttonPrefix}PidTuningBtn`);
        const discardBtn = getEl(`discard${buttonPrefix}PidTuningBtn`);
        if (tuneBtn) tuneBtn.disabled = !canStart;
        if (cancelBtn) cancelBtn.disabled = !(isRunning && activeTarget);
        if (saveBtn) saveBtn.disabled = !(canCommit && activeTarget);
        if (discardBtn) discardBtn.disabled = !(canCommit && activeTarget);
    };

    updateWheel('left', 'Left', 'speed_left');
    updateWheel('right', 'Right', 'speed_right');
}
export function disableCommandButton(buttonElement, disable = true) { if (buttonElement) buttonElement.disabled = disable; }
export function updateLegendUI(latestPointMap) {
    const formatVal = (val, digits = 1) => { const num = parseFloat(val); return isNaN(num) ? 'N/A' : num.toFixed(digits); };
    if (appState.graphs[0]?.legendValueElements?.length >= 4) { const l = appState.graphs[0].legendValueElements; l[0].textContent = formatVal(latestPointMap?.pitchDeg, 1); l[1].textContent = formatVal(latestPointMap?.desiredAngleDeg, 1); l[2].textContent = formatVal(latestPointMap?.joystickX, 2); l[3].textContent = formatVal(latestPointMap?.joystickY, 2); }
    if (appState.graphs[1]?.legendValueElements?.length >= 4) { const l = appState.graphs[1].legendValueElements; l[0].textContent = formatVal(latestPointMap?.speedSetpointLDPS, 0); l[1].textContent = formatVal(latestPointMap?.speedSetpointRDPS, 0); l[2].textContent = formatVal(latestPointMap?.speedLDPS, 0); l[3].textContent = formatVal(latestPointMap?.speedRDPS, 0); }
    if (appState.graphs[2]?.legendValueElements?.length >= 4) { const l = appState.graphs[2].legendValueElements; l[0].textContent = formatVal(latestPointMap?.targetYawAngleDeg, 1); l[1].textContent = formatVal(latestPointMap?.yawAngleDeg, 1); l[2].textContent = formatVal(latestPointMap?.targetYawRateDPS, 1); l[3].textContent = formatVal(latestPointMap?.yawRateDPS, 1); }
}
