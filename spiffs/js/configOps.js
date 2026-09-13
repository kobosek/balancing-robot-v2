// Small client-side journal for configuration writes.  A lost HTTP response
// must not make the browser invent a second operation: the original payload
// and decimal operation ID remain available after a reload and can be
// reconciled with /api/config/operation.

const STORAGE_KEY = 'balancingRobot.configOperations.v1';
const CLIENT_KEY = 'balancingRobot.configClientInstance';
const MAX_RECORDS = 24;

function storage() {
    try {
        if (globalThis.localStorage) return globalThis.localStorage;
    } catch (_) { /* private mode or a blocked origin */ }
    try {
        if (globalThis.sessionStorage) return globalThis.sessionStorage;
    } catch (_) { /* storage is optional */ }
    return null;
}

function randomClientId() {
    try {
        if (globalThis.crypto?.getRandomValues) {
            const values = new Uint32Array(2);
            globalThis.crypto.getRandomValues(values);
            return `${values[0].toString(36)}${values[1].toString(36)}`;
        }
    } catch (_) { /* fall through to Math.random */ }
    return `${Math.floor(Math.random() * 0x100000000).toString(36)}${Date.now().toString(36)}`;
}

export function configClientInstance() {
    try {
        const existing = globalThis.sessionStorage?.getItem(CLIENT_KEY);
        if (existing) return existing;
        const created = randomClientId();
        globalThis.sessionStorage?.setItem(CLIENT_KEY, created);
        return created;
    } catch (_) {
        // A memory-only fallback still keeps one page internally consistent.
        if (!configClientInstance._fallback) configClientInstance._fallback = randomClientId();
        return configClientInstance._fallback;
    }
}

export function configOperationFingerprint(payload) {
    try { return JSON.stringify(payload); } catch (_) { return ''; }
}

function readRecords() {
    const target = storage();
    if (!target) return [];
    try {
        const parsed = JSON.parse(target.getItem(STORAGE_KEY) || '[]');
        return Array.isArray(parsed) ? parsed.filter(record => record && typeof record === 'object') : [];
    } catch (_) {
        return [];
    }
}

function writeRecords(records) {
    const target = storage();
    if (!target) return false;
    try {
        target.setItem(STORAGE_KEY, JSON.stringify(records.slice(-MAX_RECORDS)));
        return true;
    } catch (_) {
        // Quota/private-mode errors must not break the control panel.  The
        // firmware operation still has its own durable journal.
        return false;
    }
}

function ownerMatches(record, scope) {
    return record?.owner === configClientInstance() && record?.scope === String(scope);
}

export function beginConfigOperation({ scope = 'global', kind = 'configuration',
                                        operationId, payload, baseRevision = 0 }) {
    if (!operationId || String(operationId) === '0') return null;
    const normalizedId = String(operationId);
    const fingerprint = configOperationFingerprint(payload);
    const now = Date.now();
    const records = readRecords();
    const existing = records.find(record => record.owner === configClientInstance() &&
        record.scope === String(scope) && record.operationId === normalizedId);
    const next = {
        ...(existing || {}),
        owner: configClientInstance(),
        scope: String(scope),
        kind: String(kind || 'configuration'),
        operationId: normalizedId,
        payload,
        fingerprint,
        baseRevision: Number(baseRevision || 0),
        // A caller is about to send the request (possibly a replay after a
        // timeout), so the local state becomes pending again.  The frozen
        // payload and ID remain unchanged.
        status: 'pending',
        createdAt: Number(existing?.createdAt || now),
        updatedAt: now
    };
    const without = records.filter(record => !(record.owner === next.owner &&
        record.operationId === normalizedId));
    without.push(next);
    writeRecords(without);
    return next;
}

export function getConfigOperation(scope = 'global', fingerprint = null) {
    const candidates = readRecords().filter(record => ownerMatches(record, scope) &&
        (record.status === 'pending' || record.status === 'uncertain'));
    if (!candidates.length) return null;
    const matching = fingerprint === null ? candidates :
        candidates.filter(record => record.fingerprint === fingerprint);
    const source = matching.length ? matching : candidates;
    return source.sort((a, b) => Number(b.updatedAt || 0) - Number(a.updatedAt || 0))[0] || null;
}

export function updateConfigOperation(scope, operationId, status, extra = {}) {
    const normalizedId = String(operationId || '0');
    if (normalizedId === '0') return null;
    const records = readRecords();
    let changed = false;
    const updated = records.map(record => {
        if (!ownerMatches(record, scope) || record.operationId !== normalizedId) return record;
        changed = true;
        return { ...record, ...extra, status: String(status), updatedAt: Date.now() };
    });
    if (changed) writeRecords(updated);
    return updated.find(record => ownerMatches(record, scope) && record.operationId === normalizedId) || null;
}

export function clearConfigOperation(scope, operationId) {
    const normalizedId = String(operationId || '0');
    if (normalizedId === '0') return;
    const records = readRecords();
    writeRecords(records.filter(record => !(ownerMatches(record, scope) &&
        record.operationId === normalizedId)));
}

export function unresolvedConfigOperationCount() {
    const owner = configClientInstance();
    return readRecords().filter(record => record.owner === owner &&
        (record.status === 'pending' || record.status === 'uncertain')).length;
}

export function listConfigOperations() {
    const owner = configClientInstance();
    return readRecords().filter(record => record.owner === owner);
}
