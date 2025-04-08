// js/graph.js
import { appState } from './state.js';
import { uiElements } from './ui.js';
import { GRAPH_COLORS, Y_SPEED_RANGE_DPS, Y_ANGLE_RANGE_DEG, Y_EFFORT_RANGE, MAX_DATA_POINTS } from './constants.js';

export function setupGraphing() {
    appState.graph.dpr = window.devicePixelRatio || 1;
    resizeGraphCanvas(); // Initial sizing
    // Add resize listener
    window.addEventListener('resize', resizeGraphCanvas);
}

export function resizeGraphCanvas() {
    const { canvas, ctx, container, dpr } = appState.graph;
    if (!container || !canvas || !ctx) return;

    try {
        const rect = container.getBoundingClientRect();
        canvas.width = rect.width * dpr;
        canvas.height = rect.height * dpr;
        ctx.setTransform(dpr, 0, 0, dpr, 0, 0); // Use setTransform for clarity
        canvas.style.width = `${rect.width}px`;
        canvas.style.height = `${rect.height}px`;
        drawGraph();
    } catch (e) {
        console.error("Resize Error:", e);
    }
}

export function drawGraph() {
    const { canvas, ctx } = appState.graph;
    const { telemetryData, telemetryJsonCache } = appState; // Get data from state
    if (!ctx || !canvas || canvas.width === 0 || canvas.height === 0) {
        return;
    }

    try {
        const dpr = appState.graph.dpr;
        const logicalWidth = canvas.width / dpr;
        const logicalHeight = canvas.height / dpr;
        const padding = { top: 20, right: 100, bottom: 30, left: 50 };
        const graphWidth = logicalWidth - padding.left - padding.right;
        const graphHeight = logicalHeight - padding.top - padding.bottom;

        ctx.clearRect(0, 0, logicalWidth, logicalHeight);

        // Draw Axes and Grid (code is identical to original, keep it here)
        ctx.lineWidth = 0.5;
        ctx.font = '10px Arial';
        const ySpeedSteps = 8;
        ctx.fillStyle = '#666'; ctx.strokeStyle = '#ccc';
        for (let i = 0; i <= ySpeedSteps; i++) {
            const value = Y_SPEED_RANGE_DPS - (i * (2 * Y_SPEED_RANGE_DPS) / ySpeedSteps);
            const y = padding.top + graphHeight * (i / ySpeedSteps);
            ctx.beginPath(); ctx.moveTo(padding.left - 5, y); ctx.lineTo(padding.left + graphWidth, y); ctx.stroke();
            ctx.textAlign = 'right'; ctx.fillText(value.toFixed(0), padding.left - 8, y + 3);
        }
        ctx.save(); ctx.translate(padding.left - 35, padding.top + graphHeight / 2); ctx.rotate(-Math.PI / 2); ctx.textAlign = 'center'; ctx.fillText('Speed (dps)', 0, 0); ctx.restore();

        const yRightSteps = 4;
        ctx.fillStyle = GRAPH_COLORS[0];
        for (let i = 0; i <= yRightSteps; i++) {
            const value = Y_ANGLE_RANGE_DEG - (i * (2 * Y_ANGLE_RANGE_DEG) / yRightSteps);
            const y = padding.top + (graphHeight / 2) * (i / yRightSteps);
            ctx.beginPath(); ctx.moveTo(padding.left + graphWidth, y); ctx.lineTo(padding.left + graphWidth + 5, y); ctx.strokeStyle = '#ccc'; ctx.stroke();
            ctx.textAlign = 'left'; ctx.fillText(value.toFixed(0) + '°', padding.left + graphWidth + 8, y + 3);
        }
        ctx.save(); ctx.translate(padding.left + graphWidth + 45, padding.top + graphHeight / 4); ctx.rotate(Math.PI / 2); ctx.textAlign = 'center'; ctx.fillText('Angle (deg)', 0, 0); ctx.restore();

        ctx.fillStyle = GRAPH_COLORS[5];
        for (let i = 0; i <= yRightSteps; i++) {
            const value = Y_EFFORT_RANGE - (i * (2 * Y_EFFORT_RANGE) / yRightSteps);
            const y = padding.top + graphHeight / 2 + (graphHeight / 2) * (i / yRightSteps);
            ctx.beginPath(); ctx.moveTo(padding.left + graphWidth, y); ctx.lineTo(padding.left + graphWidth + 5, y); ctx.strokeStyle = '#ccc'; ctx.stroke();
            ctx.textAlign = 'left'; ctx.fillText(value.toFixed(1), padding.left + graphWidth + 8, y + 3);
        }
        ctx.save(); ctx.translate(padding.left + graphWidth + 45, padding.top + 3 * graphHeight / 4); ctx.rotate(Math.PI / 2); ctx.textAlign = 'center'; ctx.fillText('Effort (-1:1)', 0, 0); ctx.restore();

        const intervalMs = parseInt(telemetryJsonCache?.interval_ms || 5);
        const totalTimeSec = (MAX_DATA_POINTS * intervalMs) / 1000.0;
        const xTimeSteps = 4;
        ctx.fillStyle = '#666';
        for (let i = 0; i <= xTimeSteps; i++) {
            const x = padding.left + graphWidth * (i / xTimeSteps);
            ctx.beginPath(); ctx.moveTo(x, padding.top); ctx.lineTo(x, padding.top + graphHeight); ctx.strokeStyle = '#ccc'; ctx.stroke();
            ctx.textAlign = 'center';
            const secondsAgo = (totalTimeSec - (i * totalTimeSec / xTimeSteps)).toFixed(1);
            ctx.fillText(`${secondsAgo}s ago`, x, padding.top + graphHeight + 15);
        }


        // --- Plot Data Lines ---
        const drawLine = (data, color, yMin, yMax, yBase = padding.top, yHeight = graphHeight, lineWidth = 1.5) => {
            if (!Array.isArray(data) || data.length === 0) return;
            ctx.strokeStyle = color; ctx.lineWidth = lineWidth; ctx.beginPath();
            let firstValidPoint = true; const range = yMax - yMin; if (Math.abs(range) < 1e-6) return;
            let pointsDrawn = 0;
            for (let i = 0; i < data.length; i++) {
                const val = data[i];
                if (val === null || isNaN(val)) { firstValidPoint = true; continue; }
                const x = padding.left + graphWidth * (i / (MAX_DATA_POINTS - 1));
                const normalizedValue = (val - yMin) / range;
                const y = yBase + yHeight * (1 - normalizedValue);
                const clampedY = Math.max(yBase, Math.min(yBase + yHeight, y));
                if (firstValidPoint) { ctx.moveTo(x, clampedY); firstValidPoint = false; } else { ctx.lineTo(x, clampedY); }
                pointsDrawn++;
            }
            if (pointsDrawn > 1) { ctx.stroke(); }
        };

        // Draw each telemetry line (pass data from appState)
        drawLine(telemetryData.speedLDPS, GRAPH_COLORS[2], -Y_SPEED_RANGE_DPS, Y_SPEED_RANGE_DPS);
        drawLine(telemetryData.speedRDPS, GRAPH_COLORS[3], -Y_SPEED_RANGE_DPS, Y_SPEED_RANGE_DPS);
        drawLine(telemetryData.balanceSpeedDPS, GRAPH_COLORS[1], -Y_SPEED_RANGE_DPS, Y_SPEED_RANGE_DPS);
        drawLine(telemetryData.targetAngVelDPS, GRAPH_COLORS[6], -Y_SPEED_RANGE_DPS, Y_SPEED_RANGE_DPS, padding.top, graphHeight, 1.0);
        drawLine(telemetryData.pitchDeg, GRAPH_COLORS[0], -Y_ANGLE_RANGE_DEG, Y_ANGLE_RANGE_DEG, padding.top, graphHeight / 2, 2.0);
        drawLine(telemetryData.effortL, GRAPH_COLORS[4], -Y_EFFORT_RANGE, Y_EFFORT_RANGE, padding.top + graphHeight / 2, graphHeight / 2);
        drawLine(telemetryData.effortR, GRAPH_COLORS[5], -Y_EFFORT_RANGE, Y_EFFORT_RANGE, padding.top + graphHeight / 2, graphHeight / 2);

    } catch (e) {
        console.error("Draw Graph Error:", e);
    }
}