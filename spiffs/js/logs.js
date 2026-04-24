import { appState } from './state.js';
import { fetchLogsApi, clearLogsApi } from './api.js';
import { uiElements } from './ui.js';

const ANSI_ESCAPE_RE = /\x1b\[[0-9;]*m/g;
const LOG_STORAGE_KEY = 'esp32WebLogLines';
const LOG_SEQUENCE_KEY = 'esp32WebLogNextSequence';

function escapeHtml(text) {
    return text
        .replaceAll('&', '&amp;')
        .replaceAll('<', '&lt;')
        .replaceAll('>', '&gt;')
        .replaceAll('"', '&quot;')
        .replaceAll("'", '&#39;');
}

function getLogLevelClass(line) {
    const match = line.match(/(?:^|\s)([EWIDV])\s+\(/);
    if (!match) {
        return 'log-line-default';
    }

    switch (match[1]) {
        case 'E': return 'log-line-error';
        case 'W': return 'log-line-warning';
        case 'I': return 'log-line-info';
        case 'D': return 'log-line-debug';
        case 'V': return 'log-line-verbose';
        default: return 'log-line-default';
    }
}

function normalizeLogLine(line) {
    return String(line || '').replace(ANSI_ESCAPE_RE, '');
}

function normalizeLogLines(lines) {
    return lines.map(normalizeLogLine);
}

function renderLogLines(lines) {
    return lines
        .map(line => {
            const lineClass = getLogLevelClass(line);
            return `<span class="${lineClass}">${escapeHtml(line)}</span>`;
        })
        .join('');
}

function downloadBlob(content, filename, mimeType) {
    const blob = new Blob([content], { type: mimeType });
    const url = URL.createObjectURL(blob);
    const link = document.createElement('a');
    link.href = url;
    link.download = filename;
    document.body.appendChild(link);
    link.click();
    link.remove();
    URL.revokeObjectURL(url);
}

function logExportTimestamp() {
    return new Date().toISOString().replace(/[:.]/g, '-');
}

function buildFormattedHtmlLog() {
    const renderedLogs = renderLogLines(appState.webLogLines);
    const generatedAt = escapeHtml(new Date().toISOString());
    return `<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>ESP32 Logs</title>
<style>
body {
    margin: 0;
    padding: 18px;
    background: #111827;
    color: #e5e7eb;
    font-family: "Courier New", Courier, monospace;
}
h1 {
    margin: 0 0 4px;
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
    font-size: 18px;
}
.meta {
    margin-bottom: 16px;
    color: #9ca3af;
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
    font-size: 13px;
}
pre {
    margin: 0;
    white-space: pre-wrap;
    overflow-wrap: anywhere;
    font-size: 13px;
    line-height: 1.35;
}
.log-line-info { color: #4ade80; }
.log-line-warning { color: #facc15; }
.log-line-error { color: #fb7185; }
.log-line-debug { color: #60a5fa; }
.log-line-verbose { color: #c084fc; }
.log-line-default { color: #e5e7eb; }
</style>
</head>
<body>
<h1>ESP32 Logs</h1>
<div class="meta">Exported ${generatedAt}</div>
<pre>${renderedLogs}</pre>
</body>
</html>`;
}

function findOverlapLength(existingLines, incomingLines) {
    const maxOverlap = Math.min(existingLines.length, incomingLines.length);
    for (let overlap = maxOverlap; overlap > 0; overlap--) {
        let matches = true;
        const existingStart = existingLines.length - overlap;
        for (let i = 0; i < overlap; i++) {
            if (existingLines[existingStart + i] !== incomingLines[i]) {
                matches = false;
                break;
            }
        }
        if (matches) {
            return overlap;
        }
    }
    return 0;
}

function mergeIncomingLogs(incomingLines) {
    if (incomingLines.length === 0) {
        return false;
    }

    if (appState.webLogLines.length === 0) {
        appState.webLogLines.push(...incomingLines);
        return true;
    }

    const overlap = findOverlapLength(appState.webLogLines, incomingLines);
    const newLines = incomingLines.slice(overlap);
    appState.webLogLines.push(...newLines);
    return newLines.length > 0;
}

function persistWebLogs() {
    try {
        localStorage.setItem(LOG_STORAGE_KEY, JSON.stringify(appState.webLogLines));
        localStorage.setItem(LOG_SEQUENCE_KEY, String(appState.webLogNextSequence || 0));
    } catch (error) {
        console.warn('Could not persist web logs:', error);
    }
}

export function loadPersistedLogs() {
    try {
        const stored = localStorage.getItem(LOG_STORAGE_KEY);
        if (!stored) {
            return;
        }

        const parsed = JSON.parse(stored);
        if (Array.isArray(parsed)) {
            appState.webLogLines = parsed.map(normalizeLogLine);
        }
        const storedSequence = Number(localStorage.getItem(LOG_SEQUENCE_KEY) || 0);
        if (Number.isFinite(storedSequence) && storedSequence > 0) {
            appState.webLogNextSequence = storedSequence;
        }
    } catch (error) {
        console.warn('Could not load persisted web logs:', error);
    }
}

export async function updateLogsPanel() {
    const requestedSequence = appState.webLogNextSequence;
    const data = await fetchLogsApi(requestedSequence);
    if (!data || !Array.isArray(data.logs)) {
        return;
    }

    const responseNextSequence = Number(data.next_sequence);
    if (requestedSequence > 0 &&
        Number.isFinite(responseNextSequence) &&
        responseNextSequence <= requestedSequence &&
        data.logs.length === 0) {
        appState.webLogNextSequence = 0;
        return;
    }

    const incomingLines = normalizeLogLines(data.logs);
    const changed = mergeIncomingLogs(incomingLines);
    if (Number.isFinite(responseNextSequence)) {
        appState.webLogNextSequence = responseNextSequence;
    }
    if (changed) {
        persistWebLogs();
    }

    const logsOutput = uiElements.logsOutput;
    if (!logsOutput || !appState.logsPanelExpanded || !changed) {
        return;
    }

    const distanceFromBottom = logsOutput.scrollHeight - logsOutput.scrollTop - logsOutput.clientHeight;
    const shouldAutoScroll = distanceFromBottom < 24;
    logsOutput.innerHTML = renderLogLines(appState.webLogLines);
    if (shouldAutoScroll) {
        logsOutput.scrollTop = logsOutput.scrollHeight;
    }
}

export function startLogsPolling(intervalMs = 1000) {
    stopLogsPolling();
    updateLogsPanel();
    appState.timers.logsFetch = setInterval(updateLogsPanel, intervalMs);
}

export function stopLogsPolling() {
    if (appState.timers.logsFetch !== null) {
        clearInterval(appState.timers.logsFetch);
        appState.timers.logsFetch = null;
    }
}

export async function handleClearLogs() {
    if (uiElements.clearLogsBtn) {
        uiElements.clearLogsBtn.disabled = true;
    }

    try {
        await clearLogsApi();
    } finally {
        if (uiElements.clearLogsBtn) {
            uiElements.clearLogsBtn.disabled = false;
        }
    }
}

export function handleSaveLogsText() {
    const text = appState.webLogLines.join('');
    downloadBlob(text, `esp32-logs-${logExportTimestamp()}.txt`, 'text/plain;charset=utf-8');
}

export function handleSaveLogsHtml() {
    downloadBlob(
        buildFormattedHtmlLog(),
        `esp32-logs-${logExportTimestamp()}.html`,
        'text/html;charset=utf-8'
    );
}

export function renderPersistedLogs() {
    if (uiElements.logsOutput) {
        uiElements.logsOutput.innerHTML = renderLogLines(appState.webLogLines);
    }
}

export function toggleLogsPanel() {
    appState.logsPanelExpanded = !appState.logsPanelExpanded;

    if (uiElements.logsPanel) {
        uiElements.logsPanel.classList.toggle('logs-panel-collapsed', !appState.logsPanelExpanded);
    }
    if (uiElements.toggleLogsBtn) {
        uiElements.toggleLogsBtn.textContent = appState.logsPanelExpanded ? 'Hide Logs' : 'Show Logs';
    }

    if (appState.logsPanelExpanded) {
        renderPersistedLogs();
    } else {
        // Keep background polling active so collapsed logs are still collected.
    }
}
