import { uiElements } from './ui.js';
import { GENERAL_CONFIG_FIELDS, LONGITUDINAL_CONFIG_FIELDS, PID_FIELDS } from './configSchema.js';
import { saveGeneralConfig, savePIDConfigSection } from './configPersistence.js';

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

function createPIDFormDiv(idPrefix, saveButtonSuffix = 'PID') {
    const formDiv = document.createElement('div');
    formDiv.dataset.pidSection = idPrefix;
    formDiv.className = 'config-pid-form';
    PID_FIELDS.forEach(field => {
        addFormField(formDiv, {
            label: field.label,
            id: `${idPrefix}_${field.suffix}`,
            step: field.step
        });
    });

    const button = document.createElement('button');
    button.textContent = `Save ${saveButtonSuffix}`;
    button.className = 'config-save-btn';
    button.addEventListener('click', () => savePIDConfigSection(idPrefix, formDiv));
    formDiv.appendChild(button);
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

    const header = document.createElement('div');
    header.className = 'strategy-form-header';
    const heading = document.createElement('h3');
    heading.textContent = 'Longitudinal Cascade';
    header.appendChild(heading);
    const note = document.createElement('p');
    note.className = 'config-help';
    note.textContent = 'Values use SI units. Editing this set does not activate the strategy; apply it from the selector while the robot is IDLE.';
    header.appendChild(note);
    container.appendChild(header);

    const formDiv = document.createElement('div');
    formDiv.dataset.strategySection = 'longitudinal_cascade';
    formDiv.className = 'config-field-form';
    LONGITUDINAL_CONFIG_FIELDS.forEach(field => addFormField(formDiv, field));

    const button = document.createElement('button');
    button.textContent = 'Save Longitudinal Settings';
    button.className = 'config-save-btn';
    button.addEventListener('click', () => saveGeneralConfig(formDiv, LONGITUDINAL_CONFIG_FIELDS));
    formDiv.appendChild(button);

    const parameters = createConfigSubsection('Cascade parameters', 'Motion profile, HOLD, synchronization and request limiter.');
    parameters.body.appendChild(formDiv);
    container.appendChild(parameters.section);

    const loops = createConfigSubsection('Loop PIDs', 'The pitch loop is shared with the velocity cascade; the velocity loop is used in velocity and position-hold modes.');
    const pidGrid = document.createElement('div');
    pidGrid.className = 'config-pid-grid';
    pidGrid.appendChild(createPidBlock('Pitch PID', createPIDFormDiv('longitudinal_pid_pitch', 'Save Pitch PID')));
    pidGrid.appendChild(createPidBlock('Velocity PI', createPIDFormDiv('longitudinal_pid_velocity', 'Save Velocity PI')));
    loops.body.appendChild(pidGrid);
    container.appendChild(loops.section);
}

function createNestedPidConfigForm(container) {
    container.classList.add('strategy-form-card');

    const header = document.createElement('div');
    header.className = 'strategy-form-header';
    const heading = document.createElement('h3');
    heading.textContent = 'Nested PID';
    header.appendChild(heading);
    const note = document.createElement('p');
    note.className = 'config-help';
    note.textContent = 'Existing pitch, wheel-speed and optional yaw loops. Each loop keeps its own draft and Save action.';
    header.appendChild(note);
    container.appendChild(header);

    const angle = createConfigSubsection('Balance angle loop', 'The outer pitch loop produces the wheel speed target.');
    angle.body.appendChild(createPIDFormDiv('pid_angle', 'Save Angle PID'));
    container.appendChild(angle.section);

    const wheels = createConfigSubsection('Wheel speed loops', 'Each wheel has an independent speed PID and tuning candidate.');
    const wheelGrid = document.createElement('div');
    wheelGrid.className = 'config-pid-grid';
    const leftBlock = createPidBlock('Left wheel PID', createPIDFormDiv('pid_speed_left', 'Save Left PID'));
    leftBlock.appendChild(createPidTuningPanel('left', 'Left Motor PID Tuning'));
    wheelGrid.appendChild(leftBlock);
    const rightBlock = createPidBlock('Right wheel PID', createPIDFormDiv('pid_speed_right', 'Save Right PID'));
    rightBlock.appendChild(createPidTuningPanel('right', 'Right Motor PID Tuning'));
    wheelGrid.appendChild(rightBlock);
    wheels.body.appendChild(wheelGrid);
    container.appendChild(wheels.section);

    const yaw = createConfigSubsection('Yaw control loops', 'Yaw control remains available only when Nested PID is active.');
    const yawGrid = document.createElement('div');
    yawGrid.className = 'config-pid-grid';
    yawGrid.appendChild(createPidBlock('Yaw angle PID', createPIDFormDiv('pid_yaw_angle', 'Save Yaw Angle PID')));
    yawGrid.appendChild(createPidBlock('Yaw rate PID', createPIDFormDiv('pid_yaw_rate', 'Save Yaw Rate PID')));
    yaw.body.appendChild(yawGrid);
    container.appendChild(yaw.section);
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
