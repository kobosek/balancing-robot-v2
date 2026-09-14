import { uiElements } from './ui.js';
import { GENERAL_CONFIG_FIELDS, LONGITUDINAL_CONFIG_FIELDS,
    NESTED_PID_CONFIG_FIELDS, PID_FIELDS } from './configSchema.js';
import { saveGeneralConfig, saveStrategyConfig, discardConfigDraft,
    rebaseAndSaveStrategyDraft } from './configPersistence.js';

function registerDynamicElement(id, element) {
    uiElements[id] = element;
    return element;
}

function addFormField(parent, fieldConfig) {
    const isCheckbox = fieldConfig.type === 'checkbox';
    const group = document.createElement('div');
    group.className = isCheckbox ? 'form-group form-group-checkbox' : 'form-group';

    const label = document.createElement('label');
    label.htmlFor = fieldConfig.id;
    label.textContent = isCheckbox ? fieldConfig.label : `${fieldConfig.label}:`;

    const isSelect = fieldConfig.type === 'select';
    const input = isSelect ? document.createElement('select') : document.createElement('input');
    // HTMLSelectElement.type is a read-only property in browsers. Set the
    // input type only for actual <input> elements; assigning "select-one"
    // would abort the complete configuration-form initialization.
    if (!isSelect) {
        input.type = isCheckbox ? 'checkbox' : 'number';
    }
    input.id = fieldConfig.id;
    input.name = fieldConfig.id;
    if (fieldConfig.section) {
        input.dataset.section = fieldConfig.section;
    }
    if (fieldConfig.key) {
        input.dataset.key = fieldConfig.key;
    }
    if (fieldConfig.path) {
        input.dataset.path = fieldConfig.path.join('.');
    }

    if (isSelect) {
        (fieldConfig.options || []).forEach(([value, label]) => {
            const option = document.createElement('option');
            option.value = value;
            option.textContent = label;
            input.appendChild(option);
        });
    } else if (!isCheckbox) {
        if (fieldConfig.step) {
            input.step = fieldConfig.step;
        }
        if (fieldConfig.min !== undefined) {
            input.min = fieldConfig.min;
        }
        if (fieldConfig.max !== undefined) {
            input.max = fieldConfig.max;
        }
    }

    if (isCheckbox) {
        group.appendChild(input);
        group.appendChild(label);
    } else {
        group.appendChild(label);
        group.appendChild(input);
    }

    parent.appendChild(group);
}

function createPIDFormDiv(idPrefix, options = {}) {
    const formDiv = document.createElement('div');
    formDiv.dataset.pidSection = idPrefix;
    formDiv.className = 'config-pid-form';
    PID_FIELDS.forEach(field => {
        addFormField(formDiv, {
            label: field.label,
            id: `${idPrefix}_${field.suffix}`,
            step: field.step
        });
        if (options.forcedZero?.includes(field.suffix)) {
            const input = formDiv.querySelector(`#${idPrefix}_${field.suffix}`);
            input.value = '0';
            input.disabled = true;
            input.dataset.forcedZero = 'true';
            input.title = 'Fixed at zero for this loop type';
        }
    });
    return formDiv;
}

function createConfigSubsection(title, description = '') {
    const section = document.createElement('section');
    section.className = 'config-subsection';

    const heading = document.createElement('h3');
    heading.textContent = title;
    section.appendChild(heading);

    if (description) {
        const note = document.createElement('p');
        note.className = 'config-help';
        note.textContent = description;
        section.appendChild(note);
    }

    const body = document.createElement('div');
    body.className = 'config-subsection-body';
    section.appendChild(body);
    return { section, body };
}

function addStrategyDraftControls(strategyForm) {
    const status = document.createElement('div');
    status.className = 'config-draft-status';
    status.dataset.draftStatus = 'true';
    status.textContent = 'No unsaved local changes.';
    strategyForm.appendChild(status);

    const comparison = document.createElement('div');
    comparison.className = 'config-draft-comparison';
    comparison.dataset.draftComparison = 'true';
    comparison.hidden = true;
    strategyForm.appendChild(comparison);

    const actions = document.createElement('div');
    actions.className = 'config-draft-actions';
    const retry = document.createElement('button');
    retry.type = 'button';
    retry.className = 'config-draft-retry';
    retry.dataset.draftAction = 'retry';
    retry.textContent = 'Rebase and retry draft';
    retry.hidden = true;
    actions.appendChild(retry);
    const discard = document.createElement('button');
    discard.type = 'button';
    discard.className = 'config-draft-discard';
    discard.dataset.draftAction = 'discard';
    discard.textContent = 'Discard local draft';
    discard.hidden = true;
    actions.appendChild(discard);
    strategyForm.appendChild(actions);

    retry.addEventListener('click', () => rebaseAndSaveStrategyDraft(strategyForm));
    discard.addEventListener('click', () => discardConfigDraft(strategyForm));
}

function createPidBlock(title, pidForm) {
    const block = document.createElement('section');
    block.className = 'config-pid-block';
    const heading = document.createElement('h4');
    heading.textContent = title;
    block.appendChild(heading);
    block.appendChild(pidForm);
    return block;
}

function createPidTuningPanel(side, title) {
    const prefix = side === 'left' ? 'leftPidTuning' : 'rightPidTuning';
    const buttonPrefix = side === 'left' ? 'Left' : 'Right';
    const panel = document.createElement('div');
    panel.className = 'pid-tuning-panel config-pid-tuning-panel';

    const heading = document.createElement('h3');
    heading.textContent = title;
    panel.appendChild(heading);

    const controls = document.createElement('div');
    controls.className = 'pid-tuning-controls';

    const tuneBtn = document.createElement('button');
    tuneBtn.id = `tune${buttonPrefix}PidBtn`;
    tuneBtn.className = 'tune';
    tuneBtn.textContent = `Tune ${buttonPrefix}`;
    controls.appendChild(registerDynamicElement(tuneBtn.id, tuneBtn));

    const cancelBtn = document.createElement('button');
    cancelBtn.id = `cancel${buttonPrefix}PidTuningBtn`;
    cancelBtn.className = 'tune-cancel';
    cancelBtn.textContent = 'Cancel';
    controls.appendChild(registerDynamicElement(cancelBtn.id, cancelBtn));

    const saveBtn = document.createElement('button');
    saveBtn.id = `save${buttonPrefix}PidTuningBtn`;
    saveBtn.className = 'tune-save';
    saveBtn.textContent = 'Save Tuned PID';
    controls.appendChild(registerDynamicElement(saveBtn.id, saveBtn));

    const discardBtn = document.createElement('button');
    discardBtn.id = `discard${buttonPrefix}PidTuningBtn`;
    discardBtn.className = 'tune-discard';
    discardBtn.textContent = 'Discard';
    controls.appendChild(registerDynamicElement(discardBtn.id, discardBtn));

    panel.appendChild(controls);

    const stateItem = document.createElement('div');
    stateItem.className = 'status-item';
    stateItem.innerHTML = `<span class="status-label">Tuning:</span> <span id="${prefix}State" class="status-value">IDLE</span>`;
    panel.appendChild(stateItem);

    const phaseItem = document.createElement('div');
    phaseItem.className = 'status-item';
    phaseItem.innerHTML = `<span class="status-label">Phase:</span> <span id="${prefix}Phase" class="status-value">IDLE</span>`;
    panel.appendChild(phaseItem);

    const progress = document.createElement('div');
    progress.className = 'pid-tuning-progress';
    progress.setAttribute('aria-hidden', 'true');
    progress.innerHTML = `<div id="${prefix}ProgressBar" class="pid-tuning-progress-fill"></div>`;
    panel.appendChild(progress);

    const message = document.createElement('div');
    message.id = `${prefix}Message`;
    message.className = 'pid-tuning-message';
    message.textContent = 'Idle';
    panel.appendChild(registerDynamicElement(message.id, message));

    const gains = document.createElement('div');
    gains.className = 'pid-tuning-gains';
    gains.innerHTML = `Candidate: <span id="${prefix}Gains">N/A</span>`;
    panel.appendChild(gains);

    registerDynamicElement(`${prefix}State`, stateItem.querySelector(`#${prefix}State`));
    registerDynamicElement(`${prefix}Phase`, phaseItem.querySelector(`#${prefix}Phase`));
    registerDynamicElement(`${prefix}ProgressBar`, progress.querySelector(`#${prefix}ProgressBar`));
    registerDynamicElement(`${prefix}Gains`, gains.querySelector(`#${prefix}Gains`));

    return panel;
}

function createGeneralConfigForm(container) {
    const formDiv = document.createElement('div');
    formDiv.dataset.strategySection = 'general';
    GENERAL_CONFIG_FIELDS.forEach(field => addFormField(formDiv, field));

    const button = document.createElement('button');
    button.textContent = 'Save General Settings';
    button.className = 'config-save-btn';
    button.addEventListener('click', () => saveGeneralConfig(formDiv, GENERAL_CONFIG_FIELDS));
    formDiv.appendChild(button);
    container.appendChild(formDiv);
}

function createLongitudinalConfigForm(container) {
    container.classList.add('strategy-form-card');

    const strategyForm = document.createElement('div');
    strategyForm.dataset.strategyForm = 'longitudinal_cascade';
    strategyForm.className = 'strategy-form';

    const header = document.createElement('div');
    header.className = 'strategy-form-header';
    const heading = document.createElement('h3');
    heading.textContent = 'Longitudinal Cascade';
    header.appendChild(heading);
    const note = document.createElement('p');
    note.className = 'config-help';
    note.textContent = 'Values use SI units. Editing this set does not activate the strategy; apply it from the selector while the robot is IDLE.';
    header.appendChild(note);
    strategyForm.appendChild(header);
    addStrategyDraftControls(strategyForm);

    const formDiv = document.createElement('div');
    formDiv.className = 'config-field-form';
    LONGITUDINAL_CONFIG_FIELDS.forEach(field => addFormField(formDiv, field));

    const parameters = createConfigSubsection('Cascade parameters', 'Motion profile, HOLD, synchronization and request limiter.');
    parameters.body.appendChild(formDiv);
    strategyForm.appendChild(parameters.section);

    const loops = createConfigSubsection('Loop PIDs', 'The pitch loop is shared with the velocity cascade; the velocity loop is used in velocity and position-hold modes.');
    const pidGrid = document.createElement('div');
    pidGrid.className = 'config-pid-grid';
    pidGrid.appendChild(createPidBlock('Pitch PD', createPIDFormDiv(
        'longitudinal_pid_pitch', { forcedZero: ['ki'] })));
    pidGrid.appendChild(createPidBlock('Velocity PI', createPIDFormDiv(
        'longitudinal_pid_velocity', { forcedZero: ['kd'] })));
    loops.body.appendChild(pidGrid);
    strategyForm.appendChild(loops.section);

    const saveAction = () => saveStrategyConfig(
        'longitudinal_cascade', strategyForm, LONGITUDINAL_CONFIG_FIELDS,
        ['longitudinal_pid_pitch', 'longitudinal_pid_velocity']);
    strategyForm._saveAction = saveAction;
    const button = document.createElement('button');
    button.textContent = 'Save Longitudinal Cascade';
    button.className = 'config-save-btn';
    button.addEventListener('click', saveAction);
    strategyForm.appendChild(button);
    container.appendChild(strategyForm);
}

function createNestedPidConfigForm(container) {
    container.classList.add('strategy-form-card');

    const strategyForm = document.createElement('div');
    strategyForm.dataset.strategyForm = 'nested_pid';
    strategyForm.className = 'strategy-form';

    const header = document.createElement('div');
    header.className = 'strategy-form-header';
    const heading = document.createElement('h3');
    heading.textContent = 'Nested PID';
    header.appendChild(heading);
    const note = document.createElement('p');
    note.className = 'config-help';
    note.textContent = 'Existing pitch, wheel-speed and optional yaw loops. Save applies the complete NestedPid set in one revision-bound operation.';
    header.appendChild(note);
    strategyForm.appendChild(header);
    addStrategyDraftControls(strategyForm);

    const settings = createConfigSubsection('Strategy settings', 'NestedPid pitch and yaw command limits and yaw enable state.');
    const settingsForm = document.createElement('div');
    settingsForm.className = 'config-field-form';
    NESTED_PID_CONFIG_FIELDS.forEach(field => addFormField(settingsForm, field));
    settings.body.appendChild(settingsForm);
    strategyForm.appendChild(settings.section);

    const angle = createConfigSubsection('Balance angle loop', 'The outer pitch loop produces the wheel speed target.');
    angle.body.appendChild(createPIDFormDiv('pid_angle'));
    strategyForm.appendChild(angle.section);

    const wheels = createConfigSubsection('Wheel speed loops', 'Each wheel has an independent speed PID and tuning candidate.');
    const wheelGrid = document.createElement('div');
    wheelGrid.className = 'config-pid-grid';
    const leftBlock = createPidBlock('Left wheel PID', createPIDFormDiv('pid_speed_left'));
    leftBlock.appendChild(createPidTuningPanel('left', 'Left Motor PID Tuning'));
    wheelGrid.appendChild(leftBlock);
    const rightBlock = createPidBlock('Right wheel PID', createPIDFormDiv('pid_speed_right'));
    rightBlock.appendChild(createPidTuningPanel('right', 'Right Motor PID Tuning'));
    wheelGrid.appendChild(rightBlock);
    wheels.body.appendChild(wheelGrid);
    strategyForm.appendChild(wheels.section);

    const yaw = createConfigSubsection('Yaw control loops', 'Yaw control remains available only when Nested PID is active.');
    const yawGrid = document.createElement('div');
    yawGrid.className = 'config-pid-grid';
    yawGrid.appendChild(createPidBlock('Yaw angle PID', createPIDFormDiv('pid_yaw_angle')));
    yawGrid.appendChild(createPidBlock('Yaw rate PID', createPIDFormDiv('pid_yaw_rate')));
    yaw.body.appendChild(yawGrid);
    strategyForm.appendChild(yaw.section);

    const saveAction = () => saveStrategyConfig(
        'nested_pid', strategyForm, NESTED_PID_CONFIG_FIELDS,
        ['pid_angle', 'pid_speed_left', 'pid_speed_right',
            'pid_yaw_angle', 'pid_yaw_rate']);
    strategyForm._saveAction = saveAction;
    const button = document.createElement('button');
    button.textContent = 'Save Nested PID';
    button.className = 'config-save-btn';
    button.addEventListener('click', saveAction);
    strategyForm.appendChild(button);
    container.appendChild(strategyForm);
}

export function createConfigForms() {
    if (!uiElements.nestedPidConfigFormContainer ||
        !uiElements.generalConfigFormContainer || !uiElements.longitudinalConfigFormContainer) {
        console.error('One or more config form containers are missing. Cannot create forms.');
        return;
    }

    uiElements.nestedPidConfigFormContainer.innerHTML = '';
    uiElements.generalConfigFormContainer.innerHTML = '<h3>General Settings</h3>';
    uiElements.longitudinalConfigFormContainer.innerHTML = '';

    createNestedPidConfigForm(uiElements.nestedPidConfigFormContainer);
    createGeneralConfigForm(uiElements.generalConfigFormContainer);
    createLongitudinalConfigForm(uiElements.longitudinalConfigFormContainer);
    console.log('Config forms created.');
}
