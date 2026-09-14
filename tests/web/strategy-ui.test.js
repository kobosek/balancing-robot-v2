import { decodeTelemetryPoint } from '../../spiffs/js/imuStatus.js';
import { TELEMETRY_V4_MIN_LENGTH } from '../../spiffs/js/constants.js';
import {
    appState,
    invalidateConfigCache,
    updateConfigCache,
    updateCurrentSystemState
} from '../../spiffs/js/state.js';
import {
    fetchConfigApi,
    fetchConfigOperationStatusApi,
    fetchStateApi,
    postConfigApi
} from '../../spiffs/js/api.js';
import {
    uiElements,
    updateStrategyUI,
    updateYawControlButtonUI,
    updatePidTuningUI,
    updateLongitudinalTelemetryUI
} from '../../spiffs/js/ui.js';
import { applySelectedBalanceStrategy } from '../../spiffs/js/configUI.js';
import { loadPIDConfigSection, savePIDConfigSection, loadStrategyConfig,
    saveStrategyConfig, rebaseAndSaveStrategyDraft } from '../../spiffs/js/configPersistence.js';
import { createConfigForms } from '../../spiffs/js/configRenderer.js';
import { buildStrategySelectionConfig, canApplyStrategy, capabilityForState } from '../../spiffs/js/strategyConfig.js';
import { LONGITUDINAL_CONFIG_FIELDS, NESTED_PID_CONFIG_FIELDS, PID_FIELDS } from '../../spiffs/js/configSchema.js';

function response(body, ok = true, status = 200) {
    return {
        ok,
        status,
        text: async () => typeof body === 'string' ? body : JSON.stringify(body)
    };
}

function makeConfig(revision = 7) {
    return {
        config_version: 3,
        config_revision: revision,
        web: { max_config_post_size: 8192 },
        behavior: { max_target_angular_velocity_dps: 60 },
        control: {
            balance_strategy: 'nested_pid',
            yaw_control_enabled: false,
            max_target_pitch_offset_deg: 5,
            strategies_revision: 2,
            strategies: {
                active: 'nested_pid',
                revision: 2,
                nested_pid: {
                    revision: 2,
                    max_target_pitch_offset_deg: 5,
                    max_target_angular_velocity_dps: 60,
                    yaw_control_enabled: false,
                    angle: {
                        kp: 3, ki: 0, kd: 0,
                        output_min: -720, output_max: 720,
                        iterm_min: -10, iterm_max: 10
                    },
                    speed_left: {
                        kp: 0.01, ki: 0.05, kd: 0.0001,
                        output_min: -1, output_max: 1,
                        iterm_min: -10, iterm_max: 10
                    },
                    speed_right: {
                        kp: 0.02, ki: 0.06, kd: 0.0002,
                        output_min: -1, output_max: 1,
                        iterm_min: -10, iterm_max: 10
                    },
                    yaw_angle: {
                        kp: 2, ki: 0, kd: 0.05,
                        output_min: -60, output_max: 60,
                        iterm_min: -20, iterm_max: 20
                    },
                    yaw_rate: {
                        kp: 0.1, ki: 0.01, kd: 0.005,
                        output_min: -100, output_max: 100,
                        iterm_min: -50, iterm_max: 50
                    }
                },
                longitudinal_cascade: {
                    revision: 4,
                    configured: true,
                    loop_mode: 'pitch_only',
                    position_kp: 0,
                    pitch_trim_deg: 0,
                    max_pitch_offset_deg: 8,
                    max_pitch_rate_dps: 120,
                    max_velocity_mps: 0.5,
                    max_hold_velocity_mps: 0.2,
                    max_acceleration_mps2: 1,
                    max_deceleration_mps2: 1,
                    hold_position_deadband_m: 0.01,
                    hold_velocity_deadband_mps: 0.01,
                    sync_enabled: false,
                    sync_kp: 0,
                    sync_kd: 0,
                    sync_position_deadband_m: 0,
                    sync_velocity_deadband_mps: 0,
                    sync_max_effort: 0,
                    max_effort: 1,
                    left_encoder_forward_sign: 1,
                    right_encoder_forward_sign: -1,
                    left_output_sign: 1,
                    right_output_sign: 1,
                    velocity_to_pitch_sign: 1,
                    pitch_to_effort_sign: 1,
                    hold_enter_velocity_mps: 0.02,
                    hold_exit_velocity_mps: 0.04,
                    hold_pitch_error_deadband_deg: 1,
                    hold_pitch_rate_deadband_dps: 15,
                    hold_settle_time_ms: 100,
                    motion_request_limit_enabled: false,
                    motion_request_limit_pitch_start_deg: 4,
                    motion_request_limit_pitch_full_deg: 8,
                    motion_request_limit_pitch_release_deg: 3,
                    motion_request_limit_effort_start: 0.6,
                    motion_request_limit_effort_full: 0.9,
                    motion_request_limit_effort_release: 0.5,
                    motion_request_limit_min_scale: 0.2,
                    pitch: {
                        kp: 0.4, ki: 0.2, kd: 0.02,
                        output_min: -1, output_max: 1,
                        iterm_min: -1, iterm_max: 1
                    },
                    velocity: {
                        kp: 0.3, ki: 0.1, kd: 0.3,
                        output_min: -1, output_max: 1,
                        iterm_min: -1, iterm_max: 1
                    }
                }
            }
        }
    };
}

function installFixture() {
    document.body.innerHTML = `
        <select id="balanceStrategySelect">
            <option value="nested_pid">Nested PID</option>
            <option value="longitudinal_cascade">Longitudinal Cascade</option>
        </select>
        <button id="applyBalanceStrategyBtn"></button>
        <div id="strategySelectionStatus"></div>
        <div id="activeBalanceStrategyValue"></div>
        <div id="configuredBalanceStrategyValue"></div>
        <div id="editedBalanceStrategyValue"></div>
        <div id="balanceStrategyCapabilityValue"></div>
        <button id="toggleYawControlBtn"></button>
        <div id="yawControlStatus"></div>
        <button id="tuneLeftPidBtn"></button>
        <button id="cancelLeftPidTuningBtn"></button>
        <button id="saveLeftPidTuningBtn"></button>
        <button id="discardLeftPidTuningBtn"></button>
        <button id="tuneRightPidBtn"></button>
        <button id="cancelRightPidTuningBtn"></button>
        <button id="saveRightPidTuningBtn"></button>
        <button id="discardRightPidTuningBtn"></button>
        <div id="longitudinalTelemetryStatus"></div>
        <div id="nestedPidConfigFormContainer"></div>
        <div id="longitudinalConfigFormContainer"></div>
        <div id="generalConfigFormContainer"></div>`;
    for (const id of [
        'balanceStrategySelect', 'applyBalanceStrategyBtn', 'strategySelectionStatus',
        'activeBalanceStrategyValue', 'configuredBalanceStrategyValue',
        'editedBalanceStrategyValue', 'balanceStrategyCapabilityValue',
        'toggleYawControlBtn', 'yawControlStatus',
        'tuneLeftPidBtn', 'cancelLeftPidTuningBtn', 'saveLeftPidTuningBtn',
        'discardLeftPidTuningBtn', 'tuneRightPidBtn', 'cancelRightPidTuningBtn',
        'saveRightPidTuningBtn', 'discardRightPidTuningBtn',
        'longitudinalTelemetryStatus',
        'nestedPidConfigFormContainer', 'longitudinalConfigFormContainer',
        'generalConfigFormContainer'
    ]) uiElements[id] = document.getElementById(id);
}

function setIdleState(active = 'nested_pid', configured = active) {
    updateCurrentSystemState({
        state_name: 'IDLE',
        state_id: 1,
        active_balance_strategy: active,
        configured_balance_strategy: configured,
        command_session_id: '17',
        strategy_change_in_progress: false,
        operation_active: false,
        operation_recovery_pending: false,
        operation_id: '0',
        operation_phase: 'idle',
        strategy_capabilities: {
            nested_pid: {
                active: active === 'nested_pid', configured: true,
                can_activate: true, loop_mode: 'nested_pid', revision: 2, reason: 'ready'
            },
            longitudinal_cascade: {
                active: active === 'longitudinal_cascade', configured: true,
                can_activate: true, loop_mode: 'pitch_only', revision: 4, reason: 'ready'
            }
        },
        yaw_control_enabled: false,
        imu: { ready: true, state: 'OPERATIONAL' }
    });
}

function makeV4Point() {
    const prefix = [10, 20, 30, 3.8, 1, 0, 0, 0, 12, 0, 1, 0,
        true, 4, 9, true, true, false];
    const point = prefix.concat([
        123456, 0, -1, 2, 7, '17', 3, 4, '42', 5,
        ...new Array(21).fill(false),
        0,
        ...new Array(18).fill(0.25),
        true, true
    ]);
    if (point.length !== TELEMETRY_V4_MIN_LENGTH) throw new Error('invalid v4 fixture');
    return point;
}

export async function runTests() {
    let checks = 0;
    const check = (condition, message) => {
        if (!condition) throw new Error(message);
        ++checks;
    };
    const oldFetch = globalThis.fetch;
    const oldAlert = globalThis.alert;
    const oldHtml = document.body.innerHTML;
    const originalConfig = appState.configDataCache;
    const originalState = appState.currentSystemState;
    const draftKeys = ['balancingRobot.configDraft.pid_angle',
        'balancingRobot.configDraft.nested_pid',
        'balancingRobot.configDraft.longitudinal_cascade'];
    const operationKey = 'balancingRobot.configOperations.v1';
    globalThis.alert = () => {};

    try {
        installFixture();
        setIdleState();

        // Each strategy is one revision-bound editor.  PID sections remain
        // visual groups inside the card, while the card owns the only
        // production Save action and the explicit conflict actions.
        createConfigForms();
        const nestedStrategyForm = document.querySelector('[data-strategy-form="nested_pid"]');
        const longitudinalStrategyForm = document.querySelector('[data-strategy-form="longitudinal_cascade"]');
        check(nestedStrategyForm?.querySelectorAll('.config-save-btn').length === 1,
            'NestedPid card does not have exactly one Save action');
        check(longitudinalStrategyForm?.querySelectorAll('.config-save-btn').length === 1,
            'longitudinal card does not have exactly one Save action');
        check(nestedStrategyForm?.querySelectorAll('[data-pid-section] .config-save-btn').length === 0 &&
            longitudinalStrategyForm?.querySelectorAll('[data-pid-section] .config-save-btn').length === 0,
            'a child PID section still owns a second Save action');
        check(longitudinalStrategyForm?.querySelector('#longitudinal_pid_pitch_ki')?.disabled === true &&
            longitudinalStrategyForm?.querySelector('#longitudinal_pid_velocity_kd')?.disabled === true,
            'longitudinal PD/PI fixed terms are editable');
        await loadStrategyConfig(longitudinalStrategyForm);
        check(longitudinalStrategyForm.querySelector('#longitudinal_pid_pitch_ki')?.value === '0' &&
            longitudinalStrategyForm.querySelector('#longitudinal_pid_velocity_kd')?.value === '0',
            'longitudinal fixed PID terms were restored from the server payload');
        check(longitudinalStrategyForm.querySelector('#longitudinal_pid_pitch_ki')?.disabled === true &&
            longitudinalStrategyForm.querySelector('#longitudinal_pid_velocity_kd')?.disabled === true,
            'longitudinal fixed PID terms remain disabled after loading');

        // A strategy Save carries its settings and all PID sections from one
        // loaded document, preserving the other strategy and compatibility
        // mirrors in the payload.
        let strategyPost = null;
        globalThis.fetch = async (url, options = {}) => {
            if (url === '/api/config' && options.method === 'POST') {
                strategyPost = JSON.parse(options.body);
                return response({ config_revision: 13, strategies_revision: 5,
                    nested_pid_revision: 5, longitudinal_cascade_revision: 4 });
            }
            if (url === '/api/config') return response(makeConfig(12));
            return response({});
        };
        await loadStrategyConfig(nestedStrategyForm);
        nestedStrategyForm.querySelector('#nested_max_target_pitch_offset_deg').value = '7';
        nestedStrategyForm.querySelector('#pid_speed_left_kp').value = '0.7';
        nestedStrategyForm.querySelector('#nested_max_target_pitch_offset_deg')
            .dispatchEvent(new Event('input', { bubbles: true }));
        await saveStrategyConfig('nested_pid', nestedStrategyForm,
            NESTED_PID_CONFIG_FIELDS,
            ['pid_angle', 'pid_speed_left', 'pid_speed_right',
                'pid_yaw_angle', 'pid_yaw_rate']);
        check(strategyPost?.control?.strategies?.nested_pid?.max_target_pitch_offset_deg === 7,
            'strategy Save omitted NestedPid settings');
        check(strategyPost?.control?.strategies?.nested_pid?.speed_left?.kp === 0.7,
            'strategy Save omitted a nested PID edit');
        check(strategyPost?.behavior?.max_target_angular_velocity_dps ===
            strategyPost?.control?.strategies?.nested_pid?.max_target_angular_velocity_dps,
            'strategy Save left the yaw compatibility mirror divergent');
        check(strategyPost?.control?.strategies?.longitudinal_cascade?.left_encoder_forward_sign === 1,
            'strategy Save dropped the other strategy record');
        check(nestedStrategyForm.querySelector('[data-draft-status]')?.dataset.state === 'clean',
            'successful strategy Save left a dirty draft status');

        // A newer document leaves the local strategy draft visible with the
        // two explicit conflict actions.  Rebase is an operator action: it
        // adopts the latest server revision only after preserving the local
        // values, then reuses the same one-card Save transaction.
        nestedStrategyForm.querySelector('#nested_max_target_pitch_offset_deg').value = '8';
        nestedStrategyForm.querySelector('#nested_max_target_pitch_offset_deg')
            .dispatchEvent(new Event('input', { bubbles: true }));
        globalThis.fetch = async (url, options = {}) => {
            if (url === '/api/config' && options.method === 'POST') {
                strategyPost = JSON.parse(options.body);
                return response({ config_revision: 16, strategies_revision: 6,
                    nested_pid_revision: 6, longitudinal_cascade_revision: 4 });
            }
            if (url === '/api/config') return response(makeConfig(15));
            return response({});
        };
        await loadStrategyConfig(nestedStrategyForm);
        check(nestedStrategyForm.dataset.draftConflict === 'true' &&
            nestedStrategyForm.querySelector('[data-draft-status]')?.dataset.state === 'conflict',
            'newer strategy revision did not expose a draft conflict');
        check(nestedStrategyForm.querySelector('[data-draft-action="retry"]')?.hidden === false &&
            nestedStrategyForm.querySelector('[data-draft-action="discard"]')?.hidden === false,
            'strategy conflict actions were not exposed');
        await rebaseAndSaveStrategyDraft(nestedStrategyForm);
        check(strategyPost?.config_revision === 15 &&
            strategyPost?.control?.strategies?.nested_pid?.max_target_pitch_offset_deg === 8,
            'explicit strategy rebase did not preserve the local value');
        check(nestedStrategyForm.querySelector('[data-draft-status]')?.dataset.state === 'clean',
            'rebased strategy draft was not cleared after Save');

        // The selector accepts only advertised strategy IDs and keeps both
        // complete strategy records when producing the full v3 document.
        const unknownCapability = capabilityForState(appState.currentSystemState, 'future_strategy');
        check(!unknownCapability.can_activate && !canApplyStrategy(appState.currentSystemState, 'future_strategy'),
            'unknown strategy was treated as activatable');
        check(buildStrategySelectionConfig(makeConfig(), 'future_strategy') === null,
            'unknown strategy produced a writable selection document');
        const selectedConfig = buildStrategySelectionConfig(makeConfig(), 'longitudinal_cascade');
        check(selectedConfig?.control?.strategies?.nested_pid?.angle?.kp === 3,
            'strategy selection dropped the NestedPid record');
        check(selectedConfig?.config_version === 3 && LONGITUDINAL_CONFIG_FIELDS.some(field => field.label.includes('(m/s)')),
            'strategy configuration units/version contract is incomplete');
        const cacheBeforeVersionError = appState.configDataCache;
        let rejectedVersionPayload = null;
        globalThis.fetch = async (url, options = {}) => {
            rejectedVersionPayload = JSON.parse(options.body);
            return response({ message: 'unsupported config_version' }, false, 400);
        };
        const rejectedVersion = await postConfigApi({ ...makeConfig(), config_version: 2 }, 'legacy-version');
        check(rejectedVersion === null && rejectedVersionPayload?.config_version === 2 &&
            appState.configDataCache === cacheBeforeVersionError,
            'rejected legacy config version changed the local configuration');

        // A state refresh from another tab must not replace a locally selected
        // strategy while the selector is still being edited.
        const select = uiElements.balanceStrategySelect;
        select.value = 'longitudinal_cascade';
        select.dataset.userEditing = 'true';
        updateStrategyUI();
        updateCurrentSystemState({ configured_balance_strategy: 'nested_pid' });
        updateStrategyUI();
        check(select.value === 'longitudinal_cascade', 'late state refresh replaced local strategy selection');
        check(select.dataset.userEditing === 'true', 'late state refresh cleared selector edit marker');

        delete select.dataset.userEditing;
        updateCurrentSystemState({ configured_balance_strategy: 'longitudinal_cascade' });
        updateStrategyUI();
        check(select.value === 'longitudinal_cascade', 'confirmed configured strategy was not selected');

        // Both GET entry points are single-flight, so timer callbacks cannot
        // reorder identical snapshots or issue duplicate reads.
        invalidateConfigCache();
        let configResolve;
        let configCalls = 0;
        globalThis.fetch = (url) => {
            if (url !== '/api/config') throw new Error(`unexpected config URL ${url}`);
            ++configCalls;
            return new Promise(resolve => { configResolve = resolve; });
        };
        const configFirst = fetchConfigApi();
        const configSecond = fetchConfigApi();
        check(configFirst === configSecond, 'config GETs were not coalesced');
        configResolve(response(makeConfig()));
        await Promise.all([configFirst, configSecond]);
        check(configCalls === 1, 'config GET was issued more than once');

        let stateResolve;
        let stateCalls = 0;
        globalThis.fetch = (url) => {
            if (url !== '/api/state') throw new Error(`unexpected state URL ${url}`);
            ++stateCalls;
            return new Promise(resolve => { stateResolve = resolve; });
        };
        const stateFirst = fetchStateApi();
        const stateSecond = fetchStateApi();
        check(stateFirst === stateSecond, 'state GETs were not coalesced');
        stateResolve(response({ state_name: 'IDLE', state_id: 1,
            active_balance_strategy: 'nested_pid', configured_balance_strategy: 'nested_pid',
            strategy_capabilities: appState.currentSystemState.strategy_capabilities,
            imu: { ready: true, state: 'OPERATIONAL' } }));
        await Promise.all([stateFirst, stateSecond]);
        check(stateCalls === 1, 'state GET was issued more than once');

        // A lost POST response is recovered only by matching operationId and
        // terminal success; it must not produce a second POST.
        const baseConfig = makeConfig();
        updateConfigCache(baseConfig);
        setIdleState('nested_pid', 'nested_pid');
        select.value = 'longitudinal_cascade';
        select.dataset.userEditing = 'true';
        let postCalls = 0;
        let observedOperationId = null;
        let recoveryConfigGets = 0;
        globalThis.fetch = async (url, options = {}) => {
            if (url === '/api/config' && options.method === 'POST') {
                ++postCalls;
                observedOperationId = JSON.parse(options.body).operation_id;
                throw new Error('simulated lost response');
            }
            if (url === '/api/config/operation') {
                return response({ operation_id: observedOperationId, operation_phase: 'succeeded' });
            }
            if (url === '/api/state') {
                return response({ state_name: 'IDLE', state_id: 1,
                    active_balance_strategy: 'longitudinal_cascade',
                    configured_balance_strategy: 'longitudinal_cascade',
                    operation_id: observedOperationId, operation_phase: 'succeeded',
                    strategy_capabilities: appState.currentSystemState.strategy_capabilities,
                    imu: { ready: true, state: 'OPERATIONAL' } });
            }
            if (url === '/api/config') {
                ++recoveryConfigGets;
                const recovered = makeConfig(8);
                recovered.control.balance_strategy = 'longitudinal_cascade';
                recovered.control.strategies.active = 'longitudinal_cascade';
                return response(recovered);
            }
            throw new Error(`unexpected recovery URL ${url}`);
        };
        const recovered = await applySelectedBalanceStrategy();
        check(recovered === true, 'matching successful operation was not recovered');
        check(postCalls === 1, 'lost response caused a duplicate strategy POST');
        check(recoveryConfigGets >= 1, 'recovery did not reload configuration');
        check(!select.dataset.userEditing, 'successful recovery kept stale selector edit marker');

        // Longitudinal mode disables yaw controls without changing the stored
        // NestedPid value; returning to NestedPid exposes the control again.
        updateCurrentSystemState({ active_balance_strategy: 'longitudinal_cascade', yaw_control_enabled: true });
        updateYawControlButtonUI();
        check(uiElements.toggleYawControlBtn.disabled === true, 'yaw control stayed enabled for longitudinal mode');
        check(uiElements.yawControlStatus.textContent.includes('UNAVAILABLE'), 'longitudinal yaw status is missing');
        updateCurrentSystemState({ active_balance_strategy: 'nested_pid', yaw_control_enabled: false });
        updateYawControlButtonUI();
        check(uiElements.toggleYawControlBtn.disabled === false, 'NestedPid yaw control was not restored');

        // Candidate actions are scoped to the active wheel and remain
        // available until an explicit Save or Discard command.
        updateCurrentSystemState({
            state_name: 'IDLE',
            pid_tuning: {
                state: 'PREVIEW_READY', target: 'motor_speed_left', phase: 'READY',
                progress: 1, has_candidate: true, save_in_progress: false,
                candidate: { speed_left: { kp: 0.2, ki: 0.1, kd: 0 } }
            }
        });
        updatePidTuningUI();
        check(uiElements.saveLeftPidTuningBtn.disabled === false && uiElements.discardLeftPidTuningBtn.disabled === false,
            'left tuning candidate actions were not enabled');
        check(uiElements.saveRightPidTuningBtn.disabled === true && uiElements.discardRightPidTuningBtn.disabled === true,
            'right tuning candidate actions were enabled for the wrong wheel');

        // Drafts survive reopening the same PID section at the same revision.
        // A newer server revision keeps the stale values and marks an explicit
        // conflict so the operator can compare/reapply them.
        const makePidForm = () => {
            const form = document.createElement('div');
            form.dataset.pidSection = 'pid_angle';
            for (const key of ['kp', 'ki', 'kd', 'output_min', 'output_max', 'iterm_min', 'iterm_max']) {
                const input = document.createElement('input');
                input.type = 'number';
                input.id = `pid_angle_${key}`;
                form.appendChild(input);
            }
            document.body.appendChild(form);
            return form;
        };
        const loadedForm = makePidForm();
        globalThis.fetch = async (url) => url === '/api/config' ? response(makeConfig(8)) : response({});
        await loadPIDConfigSection('pid_angle', loadedForm);
        loadedForm.querySelector('#pid_angle_kp').value = '9';
        loadedForm.querySelector('#pid_angle_kp').dispatchEvent(new Event('input', { bubbles: true }));
        const reloadedForm = makePidForm();
        await loadPIDConfigSection('pid_angle', reloadedForm);
        check(reloadedForm.querySelector('#pid_angle_kp').value === '9', 'same-revision PID draft was not restored');
        const newerForm = makePidForm();
        globalThis.fetch = async (url) => url === '/api/config' ? response(makeConfig(9)) : response({});
        await loadPIDConfigSection('pid_angle', newerForm);
        check(newerForm.querySelector('#pid_angle_kp').value === '9', 'stale PID draft was not retained for review');
        check(newerForm.dataset.draftConflict === 'true', 'stale PID draft was not marked as a conflict');

        // A syntactically valid but unsupported document version is unknown to
        // the editor. It must remain read-only instead of being rewritten by
        // a form that only understands the v3 schema.
        const unsupportedForm = makePidForm();
        globalThis.fetch = async (url) => url === '/api/config'
            ? response({ ...makeConfig(8), config_version: 2 }) : response({});
        await loadPIDConfigSection('pid_angle', unsupportedForm);
        check(unsupportedForm.dataset.configUnknown === 'true', 'unsupported config version was editable');
        check(unsupportedForm.querySelector('#pid_angle_kp').disabled === true,
            'unsupported config version did not disable its editor');

        // A rejected save leaves the local draft and never reports success.
        newerForm.querySelector('#pid_angle_kp').value = '11';
        newerForm.querySelector('#pid_angle_kp').dispatchEvent(new Event('input', { bubbles: true }));
        globalThis.fetch = async (url, options = {}) => url === '/api/config' && options.method === 'POST'
            ? response({ message: 'config_revision conflict' }, false, 409)
            : response(makeConfig(9));
        await savePIDConfigSection('pid_angle', newerForm);
        check(sessionStorage.getItem(draftKeys[0]) !== null, 'rejected save removed the local draft');

        // The v4 metadata contract and the full selection document fit the
        // configured HTTP limit while preserving both strategy records.
        check(decodeTelemetryPoint(makeV4Point(), 4)?.strategyName === 'nested_pid', 'browser harness v4 decoder failed');
        const faultPoint = makeV4Point();
        faultPoint[19] = 1;
        faultPoint[73] = 4;
        faultPoint[74] = true;
        const decodedFault = decodeTelemetryPoint(faultPoint.concat([
            true, true, 0, 4, true, '5', 20, false
        ]), 4);
        updateLongitudinalTelemetryUI(decodedFault);
        check(uiElements.longitudinalTelemetryStatus.textContent.includes('fault (odometry)'),
            'telemetry UI did not expose the structured odometry fault reason');
        check(JSON.stringify(makeConfig()).length < makeConfig().web.max_config_post_size,
            'full v3 configuration exceeds the configured POST headroom');

        return checks;
    } finally {
        for (const key of draftKeys) sessionStorage.removeItem(key);
        try { localStorage.removeItem(operationKey); } catch (_) { /* optional */ }
        appState.configDataCache = originalConfig;
        appState.currentSystemState = originalState;
        document.body.innerHTML = oldHtml;
        globalThis.fetch = oldFetch;
        globalThis.alert = oldAlert;
    }
}
