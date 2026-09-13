import { uiElements } from './ui.js';
import { fetchConfigApi, fetchStateApi, fetchConfigOperationStatusApi,
    createConfigOperationId, postConfigApi, pendingConfigOperation,
    persistConfigOperation, resolveConfigOperation } from './api.js';
import { createConfigForms as renderConfigForms } from './configRenderer.js';
import { loadGeneralConfig, loadPIDConfigSection } from './configPersistence.js';
import { appState } from './state.js';
import { buildStrategySelectionConfig, canApplyStrategy, capabilityForState } from './strategyConfig.js';
import { updateStrategyUI } from './ui.js';

export function createConfigForms() {
    renderConfigForms();
}

export function toggleConfigMenu() {
    const menu = uiElements.configMenu;
    if (!menu) {
        return;
    }
    const isHidden = menu.style.display === 'none' || menu.style.display === '';
    menu.style.display = isHidden ? 'block' : 'none';
    if (isHidden && !appState.configDataCache) {
        console.log('Config menu opened, fetching config data...');
        fetchConfigApi();
    }
    if (!isHidden) {
        hideAllConfigForms();
    }
}

export function hideAllConfigForms() {
    [
        uiElements.nestedPidConfigFormContainer,
        uiElements.generalConfigFormContainer,
        uiElements.longitudinalConfigFormContainer
    ].forEach(el => {
        if (el) {
            el.style.display = 'none';
        }
    });
}

export async function showConfigForm(sectionKey, container) {
    if (!container) {
        console.error(`Container for section ${sectionKey} not found.`);
        return;
    }
    hideAllConfigForms();
    container.style.display = 'block';
    const strategyForms = Array.from(container.querySelectorAll('[data-strategy-section]'));
    const pidForms = Array.from(container.querySelectorAll('[data-pid-section]'));
    if (strategyForms.length === 0 && pidForms.length === 0) {
        console.error(`Form div not found within container for section ${sectionKey}.`);
        return;
    }
    await Promise.all([
        ...strategyForms.map(formDiv => loadGeneralConfig(formDiv)),
        ...pidForms.map(formDiv => loadPIDConfigSection(formDiv.dataset.pidSection, formDiv))
    ]);
}

export async function showGeneralConfigForm(container) {
    if (!container) {
        console.error('General config form container not found.');
        return;
    }
    hideAllConfigForms();
    container.style.display = 'block';
    const formDiv = container.querySelector('div');
    if (formDiv) {
        await loadGeneralConfig(formDiv);
    } else {
        console.error('Form div not found within general config container.');
    }
}

export async function showLongitudinalConfigForm(container) {
    return showConfigForm('longitudinal_cascade', container);
}

export async function applySelectedBalanceStrategy() {
    const select = uiElements.balanceStrategySelect;
    const button = uiElements.applyBalanceStrategyBtn;
    const strategyId = select?.value;
    const selectionVersion = Number(select?.dataset.selectionVersion || 0);
    const selectionStillCurrent = () => select && select.value === strategyId &&
        Number(select.dataset.selectionVersion || 0) === selectionVersion;
    const state = appState.currentSystemState || {};
    const capability = capabilityForState(state, strategyId);
    if (!canApplyStrategy(state, strategyId)) {
        const reason = state.state_name !== 'IDLE'
            ? 'Strategy changes require the robot to be IDLE.'
            : (capability.reason || 'The selected strategy is not ready.');
        if (uiElements.strategySelectionStatus) uiElements.strategySelectionStatus.textContent = reason;
        alert(reason);
        return false;
    }

    const currentConfig = appState.configDataCache || await fetchConfigApi();
    if (!selectionStillCurrent()) {
        if (uiElements.strategySelectionStatus) {
            uiElements.strategySelectionStatus.textContent = 'Selection changed; apply the newest choice.';
        }
        return false;
    }
    const configToSend = buildStrategySelectionConfig(currentConfig, strategyId);
    if (!configToSend) {
        alert('The current configuration is unavailable or incompatible with strategy selection.');
        return false;
    }
    const operationScope = 'strategy-selection';
    const pending = pendingConfigOperation(operationScope, configToSend);
    const otherPending = pendingConfigOperation(operationScope);
    if (otherPending && !pending) {
        const reason = 'A previous strategy operation is still unresolved. Reconcile it before starting another selection.';
        if (uiElements.strategySelectionStatus) uiElements.strategySelectionStatus.textContent = reason;
        alert(reason);
        return false;
    }
    const operationId = pending?.operationId || createConfigOperationId();
    persistConfigOperation(operationScope, 'strategy-selection', operationId,
                           configToSend, Number(configToSend.config_revision || 0));
    if (button) button.disabled = true;
    if (uiElements.strategySelectionStatus) uiElements.strategySelectionStatus.textContent = `Applying ${strategyId} (#${operationId})...`;
    try {
        const result = await postConfigApi(configToSend, operationId, {
            scope: operationScope,
            kind: 'strategy-selection',
            baseRevision: Number(configToSend.config_revision || 0),
            timeoutMs: 8000,
            deferResolve: true
        });
        if (!selectionStillCurrent()) {
            if (uiElements.strategySelectionStatus) {
                uiElements.strategySelectionStatus.textContent = 'Selection changed while applying; the newest choice was kept.';
            }
            return false;
        }
        if (!result) {
            // A lost HTTP response does not imply a failed operation. Confirm
            // the same operationId before offering a retry, so a later state
            // update cannot be mistaken for a second configuration write.
            for (let attempt = 0; attempt < 7; ++attempt) {
                const [status, stateAfter, configAfter] = await Promise.all([
                    fetchConfigOperationStatusApi(), fetchStateApi(), fetchConfigApi()
                ]);
                const statusId = String(status?.operation_id || stateAfter?.operation_id || '0');
                const phase = String(status?.operation_phase || stateAfter?.operation_phase || '').toLowerCase();
                const configuredAfter = stateAfter?.configured_balance_strategy ||
                    configAfter?.control?.strategies?.active || configAfter?.control?.balance_strategy;
                const activeAfter = stateAfter?.active_balance_strategy;
                const revisionAfter = Number(stateAfter?.config_revision ?? configAfter?.config_revision ?? 0);
                const revisionConfirmed = revisionAfter >= Number(configToSend.config_revision || 0);
                if (statusId === String(operationId) && phase === 'succeeded' &&
                    configuredAfter === strategyId && activeAfter === strategyId && revisionConfirmed) {
                    resolveConfigOperation(operationScope, operationId);
                    if (selectionStillCurrent()) delete select.dataset.userEditing;
                    updateStrategyUI();
                    return !!configAfter;
                }
                if (attempt < 6) {
                    const waitMs = Math.min(3000, 300 * (2 ** attempt));
                    await new Promise(resolve => setTimeout(resolve, waitMs));
                }
            }
            return false;
        }
        const [confirmedConfig, confirmedState] = await Promise.all([
            fetchConfigApi(), fetchStateApi()
        ]);
        const configuredAfter = confirmedState?.configured_balance_strategy ||
            confirmedConfig?.control?.strategies?.active ||
            confirmedConfig?.control?.balance_strategy;
        const activeAfter = confirmedState?.active_balance_strategy;
        const operationConfirmed = String(confirmedState?.operation_id ||
            result?.operation_id || operationId) === String(operationId);
        const revisionConfirmed = Number(confirmedState?.config_revision ??
            confirmedConfig?.config_revision ?? 0) >= Number(configToSend.config_revision || 0);
        if (!operationConfirmed || configuredAfter !== strategyId ||
            activeAfter !== strategyId || !revisionConfirmed) {
            // The write was accepted but runtime confirmation is incomplete.
            // Keep the same ID/payload durable; a later state refresh can finish
            // the confirmation without creating a second write.
            persistConfigOperation(operationScope, 'strategy-selection', operationId,
                                   configToSend, Number(configToSend.config_revision || 0));
            if (uiElements.strategySelectionStatus) {
                uiElements.strategySelectionStatus.textContent = `Operation accepted (#${operationId}); waiting for matching active/configured strategy and revision.`;
            }
            return false;
        }
        resolveConfigOperation(operationScope, operationId);
        if (selectionStillCurrent()) delete select.dataset.userEditing;
        updateStrategyUI();
        const active = appState.currentSystemState?.active_balance_strategy;
        if (active !== strategyId && uiElements.strategySelectionStatus) {
            uiElements.strategySelectionStatus.textContent = `Operation accepted (#${operationId}); runtime reports ${active || 'unknown'} until the next safe apply point.`;
        }
        return true;
    } finally {
        if (button) button.disabled = false;
        updateStrategyUI();
    }
}
