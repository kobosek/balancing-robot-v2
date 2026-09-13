import {
    fetchConfigApi,
    postConfigApi,
    createConfigOperationId,
    pendingConfigOperation,
    persistConfigOperation
} from './api.js';
import { SUPPORTED_CONFIG_VERSION } from './configSchema.js';

function cloneConfig(config) {
    return config ? JSON.parse(JSON.stringify(config)) : null;
}

function setConfigFormAvailability(formElement, available, reason = '') {
    if (!formElement) return;
    if (available) {
        delete formElement.dataset.configUnknown;
    } else {
        formElement.dataset.configUnknown = 'true';
        if (reason) formElement.dataset.configUnknownReason = reason;
    }
    formElement.querySelectorAll?.('input, select').forEach(input => {
        input.disabled = !available;
    });
    formElement.querySelectorAll?.('.config-save-btn').forEach(button => {
        button.disabled = !available;
        if (!available && reason) button.title = reason;
    });
}

function isEditableConfig(fullConfig, formElement) {
    const version = Number(fullConfig?.config_version);
    if (version !== SUPPORTED_CONFIG_VERSION) {
        const reason = `Configuration version ${fullConfig?.config_version ?? 'unknown'} is unsupported; reload or repair the v3 document before editing.`;
        setConfigFormAvailability(formElement, false, reason);
        alert(reason);
        return false;
    }
    setConfigFormAvailability(formElement, true);
    return true;
}

function formDraftKey(formElement) {
    const identity = formElement?.dataset?.pidSection ||
        formElement?.dataset?.strategySection || formElement?.id;
    return identity ? `balancingRobot.configDraft.${identity}` : null;
}

function readDraft(formElement) {
    const key = formDraftKey(formElement);
    if (!key || !globalThis.sessionStorage) return null;
    try {
        const raw = globalThis.sessionStorage.getItem(key);
        return raw ? JSON.parse(raw) : null;
    } catch (error) {
        console.warn('Unable to read configuration draft:', error);
        return null;
    }
}

function captureDraft(formElement) {
    const values = {};
    formElement?.querySelectorAll?.('input, select').forEach(input => {
        values[input.id] = input.type === 'checkbox'
            ? { checked: !!input.checked }
            : { value: input.value };
    });
    return values;
}

function persistDraft(formElement) {
    const key = formDraftKey(formElement);
    if (!key || !globalThis.sessionStorage || !formElement?._loadedConfigSnapshot) return;
    try {
        globalThis.sessionStorage.setItem(key, JSON.stringify({
            baseRevision: Number(formElement.dataset.draftBaseRevision ??
                formElement.dataset.configRevision ?? 0),
            editVersion: Number(formElement.dataset.editVersion || 0),
            values: captureDraft(formElement)
        }));
    } catch (error) {
        console.warn('Unable to persist configuration draft:', error);
    }
}

function attachDraftTracking(formElement) {
    if (!formElement || formElement.dataset.draftTracking === 'true') return;
    const markEdited = () => {
        if (formElement.dataset.draftBaseRevision === undefined) {
            formElement.dataset.draftBaseRevision = String(
                Number(formElement.dataset.configRevision || 0));
        }
        formElement.dataset.editVersion = String(
            Number(formElement.dataset.editVersion || 0) + 1);
        persistDraft(formElement);
    };
    formElement.addEventListener('input', markEdited);
    formElement.addEventListener('change', markEdited);
    formElement.dataset.draftTracking = 'true';
}

function restoreDraft(formElement) {
    const draft = readDraft(formElement);
    if (!draft) return;
    // Preserve a draft based on an older document. The firmware will reject
    // the stale base revision on save, giving the operator an explicit
    // conflict instead of silently losing edits.
    formElement.dataset.draftBaseRevision = String(Number(draft.baseRevision || 0));
    formElement.dataset.editVersion = String(Number(draft.editVersion || 1));
    if (Number(draft.baseRevision) !== Number(formElement.dataset.configRevision || 0)) {
        formElement.dataset.draftConflict = 'true';
    } else {
        delete formElement.dataset.draftConflict;
    }
    Object.entries(draft.values || {}).forEach(([id, value]) => {
        const input = formElement.querySelector(`#${id}`);
        if (!input) return;
        if (input.type === 'checkbox') input.checked = !!value.checked;
        else if (value.value !== undefined) input.value = value.value;
    });
}

function clearFormDraft(formElement) {
    const key = formDraftKey(formElement);
    if (!key || !globalThis.sessionStorage) return;
    try { globalThis.sessionStorage.removeItem(key); } catch (_) { /* storage is optional */ }
}

function rememberLoadedConfig(formElement, config) {
    const snapshot = cloneConfig(config);
    formElement._loadedConfigSnapshot = snapshot;
    formElement.dataset.configRevision = String(Number(snapshot?.config_revision ?? 0));
    if (!readDraft(formElement)) {
        delete formElement.dataset.draftBaseRevision;
        delete formElement.dataset.draftConflict;
        formElement.dataset.editVersion = '0';
    }
    const operationScope = formDraftKey(formElement) || formElement.id || 'global';
    const pending = pendingConfigOperation(operationScope);
    if (pending) {
        formElement.dataset.pendingOperationId = pending.operationId;
        formElement.dataset.pendingOperationFingerprint = pending.fingerprint || '';
        formElement.dataset.pendingOperationStatus = pending.status || 'pending';
    } else {
        delete formElement.dataset.pendingOperationId;
        delete formElement.dataset.pendingOperationFingerprint;
        delete formElement.dataset.pendingOperationStatus;
    }
    attachDraftTracking(formElement);
    return snapshot;
}

function revisionForSave(formElement, config) {
    const draft = readDraft(formElement);
    if (draft && formElement.dataset.draftBaseRevision !== undefined) {
        return Number(formElement.dataset.draftBaseRevision);
    }
    return Number(formElement.dataset.configRevision ?? config?.config_revision ?? 0);
}

function configOperationForForm(formElement, config) {
    const fingerprint = JSON.stringify(config);
    const operationScope = formDraftKey(formElement) || formElement.id || 'global';
    const pending = pendingConfigOperation(operationScope, config);
    if (pending) {
        formElement.dataset.pendingOperationId = pending.operationId;
        formElement.dataset.pendingOperationFingerprint = pending.fingerprint || fingerprint;
        formElement.dataset.pendingOperationStatus = pending.status || 'pending';
        return pending.operationId;
    }
    const otherPending = pendingConfigOperation(operationScope);
    if (otherPending && otherPending.fingerprint !== fingerprint) {
        formElement.dataset.draftConflict = 'true';
        formElement.dataset.pendingOperationStatus = otherPending.status || 'uncertain';
        return null;
    }
    const operationId = createConfigOperationId();
    formElement.dataset.pendingOperationId = operationId;
    formElement.dataset.pendingOperationFingerprint = fingerprint;
    formElement.dataset.pendingOperationStatus = 'pending';
    persistConfigOperation(operationScope, 'configuration', operationId, config,
                          Number(formElement.dataset.configRevision || config?.config_revision || 0));
    return operationId;
}

function applySavedRevisions(config, saveResult) {
    if (!config || !saveResult) return config;
    if (saveResult.config_revision !== undefined) {
        config.config_revision = Number(saveResult.config_revision);
    }
    const strategies = config.control?.strategies;
    if (!strategies) return config;
    if (saveResult.strategies_revision !== undefined) {
        strategies.revision = Number(saveResult.strategies_revision);
    }
    if (strategies.nested_pid && saveResult.nested_pid_revision !== undefined) {
        strategies.nested_pid.revision = Number(saveResult.nested_pid_revision);
    }
    if (strategies.longitudinal_cascade &&
        saveResult.longitudinal_cascade_revision !== undefined) {
        strategies.longitudinal_cascade.revision = Number(
            saveResult.longitudinal_cascade_revision);
    }
    return config;
}

function finishSuccessfulSave(formElement, config, saveResult,
                              saveEditVersion, saveBaseRevision) {
    const lateEdit = Number(formElement.dataset.editVersion || 0) !==
        Number(saveEditVersion || 0);
    const savedConfig = applySavedRevisions(config, saveResult);
    rememberLoadedConfig(formElement, savedConfig);
    if (lateEdit) {
        // Do not let a late response overwrite controls edited while the
        // request was in flight. Keep the original base for an explicit
        // conflict on the next save.
        formElement.dataset.draftBaseRevision = String(saveBaseRevision);
        persistDraft(formElement);
    } else {
        clearFormDraft(formElement);
        delete formElement.dataset.draftBaseRevision;
        delete formElement.dataset.draftConflict;
        formElement.dataset.editVersion = '0';
    }
    return savedConfig;
}

function getConfigValue(config, section, key, path = []) {
    let value = config?.[section];
    for (const part of path) value = value?.[part];
    return value?.[key];
}

function setConfigValue(config, section, key, value, path = []) {
    let target = config[section] || (config[section] = {});
    for (const part of path) target = target[part] || (target[part] = {});
    target[key] = value;
}

const NESTED_PID_PATHS = {
    pid_angle: ['control', 'strategies', 'nested_pid', 'angle'],
    pid_speed_left: ['control', 'strategies', 'nested_pid', 'speed_left'],
    pid_speed_right: ['control', 'strategies', 'nested_pid', 'speed_right'],
    pid_yaw_angle: ['control', 'strategies', 'nested_pid', 'yaw_angle'],
    pid_yaw_rate: ['control', 'strategies', 'nested_pid', 'yaw_rate']
};
const LONGITUDINAL_PID_PATHS = {
    longitudinal_pid_pitch: ['control', 'strategies', 'longitudinal_cascade', 'pitch'],
    longitudinal_pid_velocity: ['control', 'strategies', 'longitudinal_cascade', 'velocity']
};
const PID_PATHS = { ...NESTED_PID_PATHS, ...LONGITUDINAL_PID_PATHS };

function getNestedPid(config, sectionKey) {
    const path = PID_PATHS[sectionKey];
    if (!path) return config?.[sectionKey];
    return path.slice(1).reduce((value, part) => value?.[part], config?.[path[0]]);
}

function setNestedPid(config, sectionKey, value) {
    const path = PID_PATHS[sectionKey];
    if (!path) config[sectionKey] = value;
    else setConfigValue(config, path[0], path[path.length - 1], value, path.slice(1, -1));
}

export async function loadPIDConfigSection(sectionKey, formElement) {
    if (!formElement) {
        console.warn(`Form element for ${sectionKey} not found.`);
        return;
    }
    const fullConfig = await fetchConfigApi();
    if (!fullConfig) {
        console.warn(`Config data for section ${sectionKey} not available.`);
        alert(`Failed to load config for ${sectionKey}. Please refresh or check connection.`);
        return;
    }
    if (!isEditableConfig(fullConfig, formElement)) return;
    const section = getNestedPid(fullConfig, sectionKey);
    if (!section) {
        console.warn(`Config data for section ${sectionKey} not available.`);
        alert(`Failed to load config for ${sectionKey}. Please refresh or check connection.`);
        return;
    }
    rememberLoadedConfig(formElement, fullConfig);

    Object.keys(section).forEach(configKey => {
        const inputId = `${sectionKey}_${configKey}`;
        const input = formElement.querySelector(`#${inputId}`);
        if (input) {
            input.value = section[configKey];
        } else {
            console.warn(`Input field #${inputId} not found in form for ${sectionKey}.`);
        }
    });
    restoreDraft(formElement);
}

export async function loadGeneralConfig(formElement) {
    if (!formElement) {
        console.warn('General config form element not found.');
        return;
    }
    const fullConfig = await fetchConfigApi();
    if (!fullConfig) {
        console.warn('Full config data not available.');
        alert('Failed to load general config. Please refresh or check connection.');
        return;
    }
    if (!isEditableConfig(fullConfig, formElement)) return;
    rememberLoadedConfig(formElement, fullConfig);

    formElement.querySelectorAll('input, select').forEach(input => {
        const idParts = input.id.split('_');
        if (idParts.length < 2 && (!input.dataset.section || !input.dataset.key)) {
            console.warn(`Could not parse section/key from input ID: ${input.id}`);
            return;
        }

        const sectionKey = input.dataset.section || idParts[0];
        const valueKey = input.dataset.key || idParts.slice(1).join('_');
        try {
            const value = getConfigValue(fullConfig, sectionKey, valueKey, input.dataset.path ? input.dataset.path.split('.') : []);
            if (value !== undefined) {
                if (input.type === 'checkbox') {
                    input.checked = !!value;
                } else {
                    input.value = value;
                }
            } else {
                console.warn(`Value for ${sectionKey}.${valueKey} (ID: ${input.id}) is undefined in config.`);
            }
        } catch (error) {
            console.warn(`Error accessing ${sectionKey}.${valueKey} (ID: ${input.id}) in config data:`, error);
        }
    });
    restoreDraft(formElement);
}

export async function savePIDConfigSection(sectionKey, formElement) {
    let isValid = true;
    const currentConfig = formElement._loadedConfigSnapshot || await fetchConfigApi();
    if (!currentConfig) {
        alert('Could not load current config. Save aborted.');
        return;
    }
    if (!isEditableConfig(currentConfig, formElement)) return;

    // Start from the complete canonical section so fields that are not
    // rendered in this form, especially its strategy revision, survive a
    // read/modify/write.  Sending only the visible PID numbers would make the
    // firmware see revision zero and reject the otherwise valid update as a
    // stale document.
    const loadedSection = getNestedPid(currentConfig, sectionKey);
    const newSectionData = cloneConfig(loadedSection) || {};

    formElement.querySelectorAll('input[type="number"]').forEach(input => {
        if (!isValid) {
            return;
        }
        const key = input.id.substring(sectionKey.length + 1);
        const value = parseFloat(input.value);
        if (isNaN(value)) {
            alert(`Invalid number for: ${input.previousElementSibling?.textContent || input.id}`);
            input.focus();
            isValid = false;
        } else {
            newSectionData[key] = value;
        }
    });

    if (!isValid) {
        return;
    }

    const configToSend = cloneConfig(currentConfig);
    const saveBaseRevision = revisionForSave(formElement, configToSend);
    configToSend.config_revision = saveBaseRevision;
    setNestedPid(configToSend, sectionKey, newSectionData);
    const operationId = configOperationForForm(formElement, configToSend);
    if (!operationId) {
        alert(`An unresolved ${sectionKey.replace(/_/g, ' ')} operation exists. Reconcile it or discard the local draft before starting another save.`);
        return;
    }
    const saveEditVersion = Number(formElement.dataset.editVersion || 0);
    const saveResult = await postConfigApi(configToSend, operationId, {
        scope: formDraftKey(formElement) || formElement.id || 'global',
        kind: 'configuration',
        baseRevision: saveBaseRevision
    });
    if (saveResult) {
        finishSuccessfulSave(formElement, configToSend, saveResult,
                             saveEditVersion, saveBaseRevision);
        alert(`${sectionKey.replace(/_/g, ' ')} config saved!`);
    } else {
        alert(`Failed to save ${sectionKey.replace(/_/g, ' ')} config.`);
    }
}

export async function saveGeneralConfig(formElement, fieldMapping) {
    let isValid = true;
    const currentConfig = formElement._loadedConfigSnapshot || await fetchConfigApi();
    if (!currentConfig) {
        alert('Could not load current config. Save aborted.');
        return;
    }
    if (!isEditableConfig(currentConfig, formElement)) return;

    const configToSend = cloneConfig(currentConfig);
    const saveBaseRevision = revisionForSave(formElement, configToSend);
    configToSend.config_revision = saveBaseRevision;
    fieldMapping.forEach(field => {
        if (!isValid) {
            return;
        }

        const input = formElement.querySelector(`#${field.id}`);
        if (!input) {
            console.warn(`Input field ID ${field.id} not found.`);
            return;
        }

        let value;
        if (input.type === 'checkbox') {
            value = input.checked;
        } else if (input.tagName === 'SELECT') {
            value = input.value;
        } else {
            value = parseFloat(input.value);
            if (isNaN(value)) {
                alert(`Invalid number for ${field.label}`);
                input.focus();
                isValid = false;
                return;
            }
        }

        setConfigValue(configToSend, field.section, field.key, value, field.path || []);
        // Keep the old control fields synchronized while the firmware still
        // exposes them as compatibility mirrors for existing clients.
        if (field.section === 'control' && (field.key === 'max_target_pitch_offset_deg' || field.key === 'yaw_control_enabled')) {
            configToSend.control[field.key] = value;
        }
    });

    if (!isValid) {
        return;
    }

    const operationId = configOperationForForm(formElement, configToSend);
    if (!operationId) {
        alert('An unresolved configuration operation exists. Reconcile it or discard the local draft before starting another save.');
        return;
    }
    const saveEditVersion = Number(formElement.dataset.editVersion || 0);
    const saveResult = await postConfigApi(configToSend, operationId, {
        scope: formDraftKey(formElement) || formElement.id || 'global',
        kind: 'configuration',
        baseRevision: saveBaseRevision
    });
    if (saveResult) {
        finishSuccessfulSave(formElement, configToSend, saveResult,
                             saveEditVersion, saveBaseRevision);
        alert('General config saved!');
    } else {
        alert('Failed to save general config.');
    }
}
