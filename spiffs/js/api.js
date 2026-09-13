import { API_CONFIG_URL, API_CONFIG_OPERATION_URL, API_COMMAND_URL, API_STATE_URL, API_DATA_URL, API_OTA_URL, API_LOGS_URL } from './constants.js';
import { updateConfigCache, invalidateConfigCache, updateCurrentSystemState } from './state.js';
import { updateStatusSectionUI } from './ui.js'; // For state updates
import {
    beginConfigOperation,
    getConfigOperation,
    updateConfigOperation,
    clearConfigOperation,
    configOperationFingerprint,
    configClientInstance,
    listConfigOperations
} from './configOps.js';

async function parseJsonOrText(response) {
    const text = await response.text();
    if (!text) return {};
    try {
        return JSON.parse(text);
    } catch (error) {
        return { message: text };
    }
}

async function apiCall(url, options = {}) {
    const timeoutMs = Number(options.timeoutMs || 0);
    const requestOptions = { ...options };
    delete requestOptions.timeoutMs;
    let timeoutHandle = null;
    const timeout = timeoutMs > 0 ? new Promise((_, reject) => {
        timeoutHandle = setTimeout(() => {
            const error = new Error(`Request timed out after ${timeoutMs} ms`);
            error.code = 'ETIMEDOUT';
            reject(error);
        }, timeoutMs);
    }) : null;
    try {
        const response = await (timeout
            ? Promise.race([fetch(url, requestOptions), timeout])
            : fetch(url, requestOptions));
        const result = await parseJsonOrText(response);
        if (!response.ok) {
            const message = result && typeof result === 'object' ? result.message : null;
            const error = new Error(message || `HTTP error ${response.status}`);
            error.status = response.status;
            error.result = result;
            throw error;
        }
        return result;
    } finally {
        if (timeoutHandle !== null) clearTimeout(timeoutHandle);
    }
}

let configOperationSequence = 0;
let configFetchInFlight = null;
let configWriteGeneration = 0;
let stateFetchInFlight = null;

// Operation IDs are decimal strings so they survive JSON number rounding in
// the browser and can be replayed verbatim after a lost HTTP response.
export function createConfigOperationId() {
    configOperationSequence = (configOperationSequence + 1) % 1000;
    if (configOperationSequence === 0) configOperationSequence = 1;
    let randomPart = 0;
    try {
        randomPart = globalThis.crypto?.getRandomValues
            ? globalThis.crypto.getRandomValues(new Uint32Array(1))[0] % 10000000
            : Math.floor(Math.random() * 10000000);
    } catch (_) {
        randomPart = Math.floor(Math.random() * 10000000);
    }
    // Decimal text avoids IEEE-754 rounding.  Seconds (10 digits), a fresh
    // seven-digit cryptographic component and a two-digit counter stay below
    // UINT64_MAX while avoiding the same-time/same-tab collision of the old
    // clock-plus-reset-counter scheme.
    return `${Math.floor(Date.now() / 1000)}${String(randomPart).padStart(7, '0')}${String(configOperationSequence % 100).padStart(2, '0')}`;
}

export function configOperationScope(scope = 'global') {
    return `${configClientInstance()}:${String(scope)}`;
}

export function persistConfigOperation(scope, kind, operationId, payload, baseRevision = 0) {
    return beginConfigOperation({
        scope: configOperationScope(scope), kind, operationId, payload, baseRevision
    });
}

export function pendingConfigOperation(scope, payload = null) {
    return getConfigOperation(configOperationScope(scope),
        payload === null ? null : configOperationFingerprint(payload));
}

export function resolveConfigOperation(scope, operationId) {
    clearConfigOperation(configOperationScope(scope), operationId);
}

// --- Config API ---
export function fetchConfigApi() {
    const requestGeneration = configWriteGeneration;
    if (configFetchInFlight && configFetchInFlight.generation === requestGeneration) {
        return configFetchInFlight.promise;
    }
    const request = (async () => {
    console.log("Fetching config from server...");
    try {
        const data = await apiCall(API_CONFIG_URL, { cache: 'no-cache' });
        if (requestGeneration === configWriteGeneration) {
            updateConfigCache(data); // Update cache in state.js
        }
        return data;
    } catch (error) {
        console.error("Failed to fetch config:", error);
        alert("Error loading configuration from robot.");
        return null;
    }
    })();
    const promise = request.finally(() => {
        if (configFetchInFlight?.promise === promise) configFetchInFlight = null;
    });
    configFetchInFlight = { generation: requestGeneration, promise };
    return promise;
}

export async function postConfigApi(configData, operationId = null, operationMeta = {}) {
    console.log("Posting updated config...");
    const requestOperationId = operationId || createConfigOperationId();
    const operationScope = operationMeta.scope || 'global';
    const operationKind = operationMeta.kind || 'configuration';
    const baseRevision = Number(operationMeta.baseRevision || configData?.config_revision || 0);
    // Invalidate every GET already in flight. Its response may still be
    // useful to its original caller, but it cannot repopulate the cache after
    // this write has started.
    configWriteGeneration += 1;
    invalidateConfigCache();
    configFetchInFlight = null;
    // Keep operation metadata at the document root. ConfigurationService
    // removes it from the request fingerprint, so a retry with the same ID is
    // idempotent even though the persisted config remains unchanged.
    const payload = { ...configData, operation_id: String(requestOperationId) };
    persistConfigOperation(operationScope, operationKind, requestOperationId,
                           configData, baseRevision);
    try {
        const result = await apiCall(API_CONFIG_URL, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(payload),
            timeoutMs: Number(operationMeta.timeoutMs || 8000)
        });
        if (result && typeof result === 'object' && result.operation_id === undefined) {
            result.operation_id = String(requestOperationId);
        }
        console.log("Config saved successfully:", result);
        updateConfigOperation(configOperationScope(operationScope), requestOperationId,
                              'succeeded', { result });
        if (!operationMeta.deferResolve) {
            clearConfigOperation(configOperationScope(operationScope), requestOperationId);
        }
        invalidateConfigCache(); // Invalidate cache via state.js
        return result;
    } catch (error) {
        const terminal = Number(error?.status || 0) >= 400 &&
            Number(error?.status || 0) < 500;
        updateConfigOperation(configOperationScope(operationScope), requestOperationId,
                              terminal ? 'failed' : 'uncertain', {
                                  error: String(error?.message || 'unknown')
                              });
        console.error("Failed to save config:", error);
        alert(`Error saving configuration: ${error.message || 'Unknown error'}`);
        return null;
    }
}

// Reconcile records that survived a reload.  The firmware sidecar is the
// authority; an unresolved local record is never silently retried with a new
// operation ID.
export async function reconcilePendingConfigOperations() {
    const records = listConfigOperations().filter(record =>
        record.status === 'pending' || record.status === 'uncertain');
    if (!records.length) return [];
    const status = await fetchConfigOperationStatusApi();
    const statusId = String(status?.operation_id || '0');
    const phase = String(status?.operation_phase || '').toLowerCase();
    records.forEach(record => {
        if (statusId !== record.operationId) return;
        if (phase === 'succeeded' || phase === 'applied') {
            clearConfigOperation(record.scope, record.operationId);
        } else if (phase === 'failed' || phase === 'aborted') {
            updateConfigOperation(record.scope, record.operationId, 'failed', {
                result: status
            });
        } else {
            updateConfigOperation(record.scope, record.operationId, 'uncertain', {
                result: status
            });
        }
    });
    return listConfigOperations();
}

export async function fetchConfigOperationStatusApi() {
    try {
        return await apiCall(API_CONFIG_OPERATION_URL, { cache: 'no-cache' });
    } catch (error) {
        console.error('Error fetching configuration operation status:', error);
        return null;
    }
}

export async function acknowledgeConfigOperationApi(operationId) {
    if (!operationId || String(operationId) === '0') {
        throw new Error('A positive configuration operation ID is required');
    }
    return await apiCall(API_CONFIG_OPERATION_URL, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ operation_id: String(operationId) })
    });
}

// --- Command API ---
export async function sendCommandApi(commandName, extraPayload = {}) {
    console.log(`Sending HTTP command: ${commandName}`);
    const configChangingCommands = new Set([
        'start_pid_tuning',
        'cancel_pid_tuning',
        'save_pid_tuning',
        'discard_pid_tuning'
    ]);
    try {
        const result = await apiCall(API_COMMAND_URL, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ command: commandName, ...extraPayload })
        });
        console.log('HTTP Command response:', result);
        if (configChangingCommands.has(commandName)) {
            invalidateConfigCache();
        }
        // Optionally provide feedback based on commandName
        // Fetch state soon after sending a command to update UI faster
        setTimeout(fetchStateApi, 300);
        return true;
    } catch (error) {
        console.error(`Error sending HTTP command '${commandName}':`, error);
        alert(`Error sending command: ${error.message || 'Unknown error'}`);
        return false;
    }
}

// --- State API ---
export function fetchStateApi() {
    if (stateFetchInFlight) return stateFetchInFlight;
    const request = (async () => {
    try {
        const data = await apiCall(API_STATE_URL, { cache: 'no-cache' });
        if (data && typeof data === 'object') {
            updateCurrentSystemState(data); // Update the state in appState
            updateStatusSectionUI();
            return data;
        } else {
            throw new Error("Invalid state data format");
        }
    } catch (error) {
        console.error('Error fetching state:', error);
        updateCurrentSystemState({
            imu: { state: 'UNKNOWN', ready: null },
            state_id: -1,
            name: 'ERROR',
            state_name: 'ERROR',
            active_balance_strategy: 'unknown',
            configured_balance_strategy: 'unknown',
            balance_strategy_config_revision: 0,
            config_revision: 0,
            command_session_id: '0',
            command_input_enabled: false,
            control_generation: 0,
            strategy_change_in_progress: false,
            operation_recovery_pending: false,
            operation_known: false,
            operation_active: false,
            operation_id: '0',
            operation_kind: 'none',
            operation_phase: 'idle',
            operation_result_code: 0,
            operation_base_revision: 0,
            operation_target_revision: 0,
            longitudinal_cascade_available: false,
            strategy_capabilities: {
                nested_pid: { active: false, configured: false, can_activate: false, loop_mode: 'nested_pid', revision: 0, reason: 'state unavailable' },
                longitudinal_cascade: { active: false, configured: false, can_activate: false, loop_mode: 'pitch_only', revision: 0, reason: 'state unavailable' }
            },
            auto_balancing_enabled: false,
            fall_detection_enabled: false,
            yaw_control_enabled: false,
            critical_battery_motor_shutdown_enabled: false,
            battery_voltage: 0,
            battery_adc_pin_voltage: 0,
            battery_percentage: 0,
            battery_is_low: false,
            battery_is_critical: false,
            battery_adc_calibrated: false,
            pid_tuning: {
                state: 'IDLE',
                target: 'motor_speed_left',
                phase: 'IDLE',
                progress: 0,
                message: 'State unavailable',
                has_candidate: false
            },
            guided_calibration: {
                state: 'IDLE',
                phase: 'IDLE',
                progress: 0,
                message: 'State unavailable'
            },
            ota: {
                available: false,
                spiffs_available: false,
                update_allowed: false,
                update_in_progress: false,
                reboot_required: false,
                bundle_id: '0',
                bundle_stage: 'none',
                bundle_recovery_pending: false,
                active_target: 'none',
                message: 'State unavailable'
            }
        });
        updateStatusSectionUI(); // Update UI to show error
        return null;
    }
    })();
    stateFetchInFlight = request.finally(() => { stateFetchInFlight = null; });
    return stateFetchInFlight;
}

// --- Telemetry Data API ---
export async function fetchDataApi() {
    try {
        return await apiCall(API_DATA_URL, { cache: 'no-cache' }); // Return the raw JSON data
    } catch (error) {
        console.error('Telemetry Update Error:', error);
        return null; // Indicate failure
    }
}

export async function uploadOtaImage(file, target) {
    if (!file) {
        alert("Select a .bin file first.");
        return null;
    }

    try {
        const uploadUrl = `${API_OTA_URL}?target=${encodeURIComponent(target)}`;
        const result = await apiCall(uploadUrl, {
            method: 'POST',
            headers: { 'Content-Type': 'application/octet-stream' },
            body: file
        });
        setTimeout(fetchStateApi, 300);
        return result;
    } catch (error) {
        console.error("OTA upload failed:", error);
        alert(`OTA upload failed: ${error.message || 'Unknown error'}`);
        return null;
    }
}

export const uploadOtaFirmware = uploadOtaImage;

export async function fetchLogsApi(sinceSequence = 0) {
    try {
        const url = `${API_LOGS_URL}?since=${encodeURIComponent(sinceSequence || 0)}`;
        return await apiCall(url, { cache: 'no-cache' });
    } catch (error) {
        console.error('Error fetching logs:', error);
        return null;
    }
}

export async function clearLogsApi() {
    try {
        return await apiCall(API_LOGS_URL, { method: 'DELETE' });
    } catch (error) {
        console.error('Error clearing logs:', error);
        alert(`Error clearing logs: ${error.message || 'Unknown error'}`);
        return null;
    }
}
