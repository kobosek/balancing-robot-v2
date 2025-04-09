import { appState } from './state.js';
// <<< ADDED Y_YAWRATE_RANGE_DPS >>>
import { GRAPH_COLORS, Y_SPEED_RANGE_DPS, Y_ANGLE_RANGE_DEG, Y_EFFORT_RANGE, Y_YAWRATE_RANGE_DPS, MAX_DATA_POINTS } from './constants.js';

// --- Graph Configurations ---
const graphConfigs = [
    // Graph 1: Pitch (Actual/Desired) & Control Input
    {
        yLeftAxis: { min: -Y_ANGLE_RANGE_DEG, max: Y_ANGLE_RANGE_DEG, steps: 6, label: 'Angle (deg)', color: GRAPH_COLORS[0] },
        yRightAxis: { min: -Y_EFFORT_RANGE, max: Y_EFFORT_RANGE, steps: 4, label: 'Joy (-1:1)', color: GRAPH_COLORS[1] },
        series: [
            { dataKey: 'pitchDeg', color: GRAPH_COLORS[0], yAxis: 'left', lineWidth: 2.0 },        // Actual Pitch (Purple)
            { dataKey: 'desiredAngleDeg', color: GRAPH_COLORS[6], yAxis: 'left', lineWidth: 1.0 }, // Desired Angle (Teal)
            { dataKey: 'joystickX', color: GRAPH_COLORS[1], yAxis: 'right', lineWidth: 1.0 },       // Joy X (Blue)
            { dataKey: 'joystickY', color: GRAPH_COLORS[3], yAxis: 'right', lineWidth: 1.0 },       // Joy Y (Green)
        ]
    },
    // Graph 2: Speeds (Actual and Setpoints)
    {
        yLeftAxis: { min: -Y_SPEED_RANGE_DPS, max: Y_SPEED_RANGE_DPS, steps: 8, label: 'Speed (dps)', color: '#666' },
        yRightAxis: null,
        series: [
            { dataKey: 'speedSetpointLDPS', color: GRAPH_COLORS[4], yAxis: 'left', lineWidth: 1.5 }, // Setpoint L (Yellow)
            { dataKey: 'speedSetpointRDPS', color: GRAPH_COLORS[5], yAxis: 'left', lineWidth: 1.5 }, // Setpoint R (Orange)
            { dataKey: 'speedLDPS', color: GRAPH_COLORS[2], yAxis: 'left', lineWidth: 1.5 },         // Actual L (Red)
            { dataKey: 'speedRDPS', color: GRAPH_COLORS[3], yAxis: 'left', lineWidth: 1.5 },         // Actual R (Green)
        ]
    },
    // Graph 3: Yaw Rate <<< ADDED >>>
    {
        yLeftAxis: { min: -Y_YAWRATE_RANGE_DPS, max: Y_YAWRATE_RANGE_DPS, steps: 6, label: 'Yaw Rate (dps)', color: GRAPH_COLORS[7] }, // Use new constant & color
        yRightAxis: null,
        series: [
            { dataKey: 'yawRateDPS', color: GRAPH_COLORS[7], yAxis: 'left', lineWidth: 2.0 }, // Yaw Rate (Light Blue)
            // Optional: Add targetAngVel_dps here later if needed
        ]
    },
];

export function setupGraphs() {
    // <<< MODIFIED: Loop through all graph configs >>>
    appState.graphs.forEach((graphState, index) => {
        // Check if config exists for this index
        if (index >= graphConfigs.length) {
             console.warn(`No graph config found for graph index ${index}. Skipping setup.`);
             return;
        }
        graphState.dpr = window.devicePixelRatio || 1;
        graphState.config = graphConfigs[index]; // Assign config

        // Ensure canvas exists (logic moved from assignElements)
        graphState.canvas = document.getElementById(`telemetryGraph${index + 1}`);
        if (!graphState.canvas) {
            console.error(`Canvas telemetryGraph${index + 1} not found!`);
            return; // Skip if canvas missing
        }
        graphState.container = document.getElementById(`graphContainer${index + 1}`); // Also get container

        try {
            graphState.ctx = graphState.canvas.getContext('2d');
            if (!graphState.ctx) throw new Error('Failed context');
             // Get legend elements (logic moved from assignElements)
             graphState.legendValueElements = [];
             const legendElement = document.getElementById(`legend${index + 1}`);
             if (legendElement) {
                 legendElement.querySelectorAll('.legend-value').forEach(span => {
                     graphState.legendValueElements.push(span);
                 });
             } else {
                 console.warn(`Legend element legend${index + 1} not found.`);
             }
        } catch(error) {
             console.error(`Error setting up graph ${index + 1}:`, error);
             graphState.ctx = null; // Ensure ctx is null on error
        }
    });
    // <<< END MODIFICATION >>>
    resizeAllGraphs();
    window.addEventListener('resize', resizeAllGraphs);
}


// --- resizeAllGraphs, drawAllGraphs, drawSingleGraph ---
// (No changes needed in the functions themselves, they iterate over appState.graphs)
export function resizeAllGraphs() {
    appState.graphs.forEach((graphState, index) => {
        const { canvas, ctx, container, dpr } = graphState;
        if (!container || !canvas || !ctx) { return; }
        try {
            const rect = container.getBoundingClientRect();
            const newWidth = Math.max(rect.width, 50);
            const newHeight = Math.max(rect.height, 50);
            canvas.width = newWidth * dpr; canvas.height = newHeight * dpr;
            ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
            canvas.style.width = `${newWidth}px`; canvas.style.height = `${newHeight}px`;
        } catch (e) { console.error(`Resize Error graph ${index + 1}:`, e); }
    });
    requestAnimationFrame(drawAllGraphs);
}
export function drawAllGraphs() {
    appState.graphs.forEach((graphState) => {
        if (graphState.ctx && graphState.canvas && graphState.config) { // Check config too
             drawSingleGraph(graphState);
        }
    });
}
function drawSingleGraph(graphState) {
    const { canvas, ctx, config } = graphState; // Relies on config being set
    const { telemetryData } = appState;
    if (!ctx || !canvas || !config || canvas.width < 1 || canvas.height < 1) { return; }
    try {
        const dpr = graphState.dpr; const logicalWidth = canvas.width / dpr; const logicalHeight = canvas.height / dpr;
        const padding = { top: 10, right: 55, bottom: 30, left: 55 };
        const graphWidth = logicalWidth - padding.left - padding.right; const graphHeight = logicalHeight - padding.top - padding.bottom;
        if (graphWidth <= 0 || graphHeight <= 0) return;
        ctx.clearRect(0, 0, logicalWidth, logicalHeight); ctx.font = '10px Arial';
        const drawYAxis = (axisConfig, isLeftAxis) => { /* ... Axis drawing logic ... */
            if (!axisConfig) return;
            const { min, max, steps, label, color } = axisConfig; const range = max - min; if (Math.abs(range) < 1e-6) return;
            ctx.lineWidth = 0.5; ctx.fillStyle = color || '#666'; ctx.strokeStyle = '#ccc';
            for (let i = 0; i <= steps; i++) {
                const value = max - (i * range / steps); const y = padding.top + graphHeight * (i / steps);
                ctx.beginPath(); ctx.moveTo(padding.left, y); ctx.lineTo(padding.left + graphWidth, y); ctx.stroke();
                const labelX = isLeftAxis ? padding.left - 8 : padding.left + graphWidth + 8;
                ctx.textAlign = isLeftAxis ? 'right' : 'left';
                let precision = 0; if (Math.abs(range) < 10) precision = 1; if (Math.abs(range) < 1) precision = 2; if (Math.abs(range) > 1000) precision = 0; // Adjust for large ranges
                ctx.fillText(value.toFixed(precision), labelX, y + 3);
            }
            ctx.save(); const labelPosX = isLeftAxis ? padding.left - 40 : padding.left + graphWidth + 40; const labelPosY = padding.top + graphHeight / 2;
            ctx.translate(labelPosX, labelPosY); ctx.rotate(isLeftAxis ? -Math.PI / 2 : Math.PI / 2); ctx.textAlign = 'center'; ctx.fillText(label, 0, 0); ctx.restore();
        };
        drawYAxis(config.yLeftAxis, true); drawYAxis(config.yRightAxis, false);
        const intervalMs = parseInt(appState.telemetryJsonCache?.interval_ms || 5); const totalTimeSec = (MAX_DATA_POINTS * intervalMs) / 1000.0;
        const xTimeSteps = Math.min(4, Math.floor(graphWidth / 80)); ctx.lineWidth = 0.5; ctx.fillStyle = '#666'; ctx.strokeStyle = '#ccc';
        for (let i = 0; i <= xTimeSteps; i++) {
            const x = padding.left + graphWidth * (i / xTimeSteps); ctx.beginPath(); ctx.moveTo(x, padding.top); ctx.lineTo(x, padding.top + graphHeight + 5); ctx.stroke();
            ctx.textAlign = 'center'; const secondsAgo = (totalTimeSec - (i * totalTimeSec / xTimeSteps)).toFixed(1); ctx.fillText(`${secondsAgo}s`, x, padding.top + graphHeight + 15);
        }
        ctx.textAlign = 'center'; ctx.fillText('Time ago', padding.left + graphWidth/2, padding.top + graphHeight + 28);
        const drawLine = (data, color, yMin, yMax, yBase, yHeight, lineWidth = 1.5) => { /* ... Line drawing logic ... */
             if (!Array.isArray(data) || data.length === 0) return;
             ctx.strokeStyle = color; ctx.lineWidth = lineWidth; ctx.beginPath(); let firstValidPoint = true; const range = yMax - yMin; if (Math.abs(range) < 1e-6) return; let pointsDrawn = 0;
             for (let i = 0; i < data.length; i++) {
                 const val = data[i]; if (val === null || isNaN(val)) { firstValidPoint = true; continue; }
                 const x = padding.left + graphWidth * (i / (MAX_DATA_POINTS - 1)); const normalizedValue = (val - yMin) / range; const y = yBase + yHeight * (1 - normalizedValue);
                 const clampedY = Math.max(yBase, Math.min(yBase + yHeight, y)); if (firstValidPoint) { ctx.moveTo(x, clampedY); firstValidPoint = false; } else { ctx.lineTo(x, clampedY); } pointsDrawn++;
             } if (pointsDrawn > 1) { ctx.stroke(); }
         };
        config.series.forEach(seriesConfig => { // Draw based on assigned config
            const dataArray = telemetryData[seriesConfig.dataKey]; const yAxisConfig = seriesConfig.yAxis === 'left' ? config.yLeftAxis : config.yRightAxis;
            if (dataArray && yAxisConfig) { drawLine(dataArray, seriesConfig.color, yAxisConfig.min, yAxisConfig.max, padding.top, graphHeight, seriesConfig.lineWidth); }
        });
    } catch (e) {
        console.error("Draw Graph Error:", e); ctx.fillStyle = 'red'; ctx.font = '12px Arial'; ctx.textAlign = 'center'; ctx.fillText('Graph Error', (canvas.width / dpr) / 2, (canvas.height / dpr) / 2);
    }
}