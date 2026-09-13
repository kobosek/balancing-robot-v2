export const BALANCE_STRATEGIES = Object.freeze([
    { id: 'nested_pid', label: 'Nested PID', description: 'Existing pitch, wheel-speed and optional yaw loops.' },
    { id: 'longitudinal_cascade', label: 'Longitudinal Cascade', description: 'Pitch baseline with velocity, position hold and wheel synchronization.' }
]);

export function capabilityForState(state, strategyId) {
    const advertised = state?.strategy_capabilities?.[strategyId];
    if (advertised) return advertised;
    const stateKnown = state?.active_balance_strategy === 'nested_pid' ||
        state?.active_balance_strategy === 'longitudinal_cascade';
    return {
        active: state?.active_balance_strategy === strategyId,
        configured: stateKnown && strategyId === 'nested_pid',
        can_activate: false,
        turn_control_supported: strategyId === 'nested_pid',
        loop_mode: strategyId === 'nested_pid' ? 'nested_pid' : 'pitch_only',
        revision: 0,
        reason: stateKnown ? 'Strategy capability is not available yet.' : 'Strategy state is unknown.'
    };
}

export function canApplyStrategy(state, strategyId) {
    const capability = capabilityForState(state, strategyId);
    return state?.state_name === 'IDLE' && !state?.operation_active &&
        !state?.operation_recovery_pending && !!capability.can_activate &&
        state?.active_balance_strategy !== strategyId;
}

export function buildStrategySelectionConfig(config, strategyId) {
    const copy = config ? JSON.parse(JSON.stringify(config)) : null;
    if (!copy?.control?.strategies || !BALANCE_STRATEGIES.some(item => item.id === strategyId)) return null;
    copy.control.balance_strategy = strategyId;
    copy.control.strategies.active = strategyId;
    return copy;
}
