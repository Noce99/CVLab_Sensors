// Live view of the cameras of a tool, chessboard capture and coverage of the saved boards.
// TOOL (set by capture.html): cameras, labels, preview_delay_ms, aspect, api, timed_capture...

const CAMERAS = TOOL.cameras;
const LABELS = TOOL.labels;
const API = TOOL.api;
const GRID = 3;   // coverage grid: GRID x GRID cells per image
const CELL_NAMES = [['top-left', 'top', 'top-right'], ['left', 'center', 'right'], ['bottom-left', 'bottom', 'bottom-right']];
// Pause between two previews of a camera (an event preview costs ~100 ms of the camera process)
const PREVIEW_DELAY_MS = TOOL.preview_delay_ms;
// Image age shown amber / red above these
const SLOW_MS = 300;
const STALE_MS = 1000;
// Board size, as a fraction of the rgb_left image area
const NEAR_AREA = 0.25;
const FAR_AREA = 0.08;

let patternSize = PATTERN_SIZE;
let boards = [];          // [{name, corners: {cam: [[x, y] normalized]}}]
let lastCapture = null;   // corners of the last capture, highlighted on the overlay

document.addEventListener('DOMContentLoaded', () => {
    buildCameraTiles();
    CAMERAS.forEach(previewLoop);
    updateStatus();
    setInterval(updateStatus, 2000);
    loadSamples();
    loadSessions();

    document.getElementById('capture').addEventListener('click', manualCapture);
    document.getElementById('auto-capture').addEventListener('click', toggleAutoCapture);
    document.getElementById('show-coverage').addEventListener('change', drawAllOverlays);
    document.getElementById('session').addEventListener('change', changeSession);
    document.getElementById('open-pattern')?.addEventListener('click', openPattern);
    document.getElementById('save-failed')?.addEventListener('change', e => setSession({save_failed: e.target.checked}));
    document.addEventListener('keydown', e => {
        if (e.code === 'Space' && !e.repeat && e.target.tagName !== 'INPUT') {
            e.preventDefault();
            manualCapture();
        }
    });
    window.addEventListener('resize', drawAllOverlays);
});

// Opens (or reloads with the new frequency) the blinking chessboard in its own window
function openPattern() {
    const hz = parseFloat(document.getElementById('blink-hz').value) || 10;
    window.open(`/pattern?hz=${hz}`, 'blinking-pattern', 'popup,width=1280,height=800');
}

// ============================================================================
// SESSION
// ============================================================================

async function loadSessions() {
    const list = await (await fetch(`${API}/sessions`)).json();
    const select = document.getElementById('session');
    select.length = 1;   // keep "New session"
    for (const s of list) select.add(new Option(`${s.name} — ${s.boards} boards`, s.name));
}

async function changeSession(e) {
    await setSession({name: e.target.value || null});
}

async function setSession(body) {
    const response = await fetch(`${API}/session`, {
        method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(body),
    });
    if (!response.ok) setMessage((await response.json()).error, 'bad');
    lastCapture = null;
    document.getElementById('capture-strip').innerHTML = '';
    await Promise.all([updateStatus(), loadSamples()]);
}

function buildCameraTiles() {
    const container = document.getElementById('cameras');
    document.getElementById('capture-strip').style.gridTemplateColumns = `repeat(${CAMERAS.length}, minmax(0, 1fr))`;
    for (const cam of CAMERAS) {
        container.insertAdjacentHTML('beforeend', `
            <div class="camera">
                <div class="camera-header">
                    <span class="dot" id="dot-${cam}"></span>
                    <h2>${LABELS[cam]}</h2>
                    <span class="age" id="age-${cam}" title="Age of the image shown">-</span>
                    <span class="muted" id="rate-${cam}">-</span>
                </div>
                <div class="camera-error" id="error-${cam}"></div>
                <div class="stage" style="aspect-ratio: ${TOOL.aspect[cam][0]} / ${TOOL.aspect[cam][1]}">
                    <div class="placeholder" id="placeholder-${cam}">Waiting for frames…</div>
                    <img id="live-${cam}" alt="">
                    <canvas id="overlay-${cam}"></canvas>
                </div>
                <div class="camera-footer">
                    <span id="count-${cam}">0 boards</span>
                    <span class="missing" id="missing-${cam}"></span>
                </div>
            </div>`);
    }
}

// ============================================================================
// LIVE VIEW / STATUS
// ============================================================================

async function previewLoop(cam) {
    const img = document.getElementById(`live-${cam}`);
    const placeholder = document.getElementById(`placeholder-${cam}`);
    while (true) {
        try {
            const response = await fetch(`${API}/preview/${cam}`, {cache: 'no-store'});
            if (!response.ok) throw new Error((await response.json()).error);
            showAge(cam, Number(response.headers.get('X-Frame-Age-Ms')));
            const url = URL.createObjectURL(await response.blob());
            await new Promise(resolve => { img.onload = img.onerror = resolve; img.src = url; });
            if (img.dataset.url) URL.revokeObjectURL(img.dataset.url);
            img.dataset.url = url;
            placeholder.style.display = 'none';
            await sleep(PREVIEW_DELAY_MS[cam]);
        } catch (error) {
            placeholder.style.display = '';
            showAge(cam, null);
            placeholder.textContent = `Waiting for frames… (${error.message})`;
            await sleep(1000);
        }
    }
}

function showAge(cam, ageMs) {
    const element = document.getElementById(`age-${cam}`);
    element.textContent = ageMs === null ? '-' : formatAge(ageMs);
    element.className = 'age' + (ageMs === null ? '' : ageMs > STALE_MS ? ' stale' : ageMs > SLOW_MS ? ' slow' : '');
}

function formatAge(ageMs) {
    return ageMs < 1000 ? `${ageMs} ms old` : `${(ageMs / 1000).toFixed(1)} s old`;
}

async function updateStatus() {
    try {
        const status = await (await fetch(`${API}/status`)).json();
        document.getElementById('folder').textContent = status.folder;
        patternSize = status.pattern_size;
        const select = document.getElementById('session');
        select.value = [...select.options].some(o => o.value === status.session) ? status.session : '';
        const saveFailed = document.getElementById('save-failed');
        if (saveFailed) saveFailed.checked = status.save_failed;
        for (const cam of CAMERAS) {
            const s = status.sensors[cam];
            document.getElementById(`rate-${cam}`).textContent = `${s.rate} ${s.unit}`;
            const error = document.getElementById(`error-${cam}`);
            error.textContent = s.error || s.warning || '';
            error.classList.toggle('warn', !s.error && !!s.warning);
            const dot = document.getElementById(`dot-${cam}`);
            dot.className = 'dot ' + (s.error ? 'bad' : s.warning || s.rate === 0 ? 'warn' : 'ok');
        }
    } catch (error) {
        CAMERAS.forEach(cam => {
            document.getElementById(`dot-${cam}`).className = 'dot bad';
            document.getElementById(`error-${cam}`).textContent = 'Server not reachable';
        });
    }
}

// ============================================================================
// CAPTURE
// ============================================================================

let capturing = false;     // a capture request is in flight
let autoCapture = false;   // automatic capture: a new capture every few seconds
let autoTimer = null;

async function capture() {
    if (capturing) return;
    capturing = true;
    updateCaptureButtons();
    const seconds = TOOL.timed_capture ? parseFloat(document.getElementById('capture-seconds').value) || TOOL.capture_s : 0;
    const countdown = seconds ? startCountdown(seconds) : null;
    if (!seconds && !autoCapture) setMessage('Capturing…', '');
    try {
        const response = await fetch(`${API}/capture`, {
            method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify({seconds}),
        });
        const result = await response.json();
        if (!response.ok) throw new Error(result.error);
        showCapture(result);
        if (result.saved) {
            await loadSamples(result.saved);
            await loadSessions();   // the first board creates the session folder
            updateStatus();
        }
    } catch (error) {
        setMessage(`Capture failed: ${error.message}`, 'bad');
    } finally {
        clearInterval(countdown);
        capturing = false;
        updateCaptureButtons();
        scheduleAutoCapture();
    }
}

// Timed capture: the board must stay still while the points are collected
function startCountdown(seconds) {
    const end = Date.now() + seconds * 1000;
    const tick = () => {
        const left = Math.ceil((end - Date.now()) / 1000);
        setMessage(left > 0 ? `Hold the board still… ${left} s` : 'Searching the chessboard…', '');
    };
    tick();
    return setInterval(tick, 250);
}

function manualCapture() {
    if (!autoCapture) capture();
}

// The interval runs from the end of one capture to the start of the next, so captures never overlap
function scheduleAutoCapture() {
    clearTimeout(autoTimer);
    if (!autoCapture || capturing) return;
    const seconds = Math.max(0.2, parseFloat(document.getElementById('auto-interval').value) || 1);
    autoTimer = setTimeout(capture, seconds * 1000);
}

function toggleAutoCapture() {
    autoCapture = !autoCapture;
    clearTimeout(autoTimer);
    updateCaptureButtons();
    if (autoCapture) capture();
}

function updateCaptureButtons() {
    document.getElementById('capture').disabled = capturing || autoCapture;
    const auto = document.getElementById('auto-capture');
    auto.textContent = autoCapture ? 'Stop automatic capture' : 'Automatic capture';
    auto.classList.toggle('active', autoCapture);
}

function showCapture(result) {
    const strip = document.getElementById('capture-strip');
    strip.innerHTML = '';
    for (const cam of CAMERAS) {
        const c = result.cameras[cam];
        let detail = '';
        if (c.activation !== undefined && c.activation < 0.1) detail = ' — few events, move the board';
        strip.insertAdjacentHTML('beforeend', `
            <div class="shot ${c.found ? 'found' : 'missed'}">
                <img src="${c.image}" alt="${LABELS[cam]}">
                <div class="shot-label">
                    <span>${LABELS[cam]} · ${formatAge(c.age_ms)}${detail}</span>
                    <span class="verdict">${c.found ? '✓ detected' + variantLabel(c.variant) : '✗ not found'}</span>
                </div>
            </div>`);
    }

    const missed = CAMERAS.filter(cam => !result.cameras[cam].found).map(cam => LABELS[cam]);
    const all = CAMERAS.length === 2 ? 'Both' : `All ${CAMERAS.length}`;
    const drift = result.rgb_drift_px != null ? ` (RGB moved ${result.rgb_drift_px} px)` : '';
    if (result.saved) {
        setMessage(`Saved as board ${result.saved}` + (result.warning ? ` — ${result.warning}` : ''),
                   result.warning ? 'warn' : 'ok');
    } else if (result.moving) {
        setMessage(`${all} saw the board, but it moved during the capture${drift}: not saved, hold it still`, 'warn');
    } else if (result.duplicate_of) {
        setMessage(`Detected in ${all.toLowerCase()}, but too close to board ${result.duplicate_of}: not saved, move the board`, 'warn');
    } else {
        setMessage(`Not saved: no chessboard in ${missed.join(', ')}`, 'bad');
    }

    lastCapture = Object.fromEntries(CAMERAS.map(cam => [cam, result.cameras[cam].corners]));
    drawAllOverlays();
}

// Image correction that found the board: [gamma, blur kernel, CLAHE clip(, event rendering)]
function variantLabel(variant) {
    if (!variant) return '';
    const [gamma, blur, clahe, rendering] = variant;
    const parts = rendering && rendering !== 'count' ? [rendering] : [];
    if (clahe) parts.push(`CLAHE ${clahe}`);
    if (gamma !== 1) parts.push(`γ ${gamma}`);
    if (blur) parts.push(`blur ${blur}`);
    return parts.length ? ` (${parts.join(', ')})` : '';
}

function setMessage(text, kind) {
    const message = document.getElementById('capture-message');
    message.textContent = text;
    message.className = `message ${kind}`;
}

// ============================================================================
// SAVED BOARDS
// ============================================================================

async function loadSamples(latest) {
    boards = await (await fetch(`${API}/samples`)).json();
    const container = document.getElementById('samples');
    container.innerHTML = '';
    for (const board of boards) {
        const chip = document.createElement('div');
        chip.className = 'sample' + (board.name === latest ? ' latest' : '');
        chip.innerHTML = `<button class="view">${board.name}</button><button class="delete" title="Delete">×</button>`;
        chip.querySelector('.view').addEventListener('click', () => viewSample(board));
        chip.querySelector('.delete').addEventListener('click', () => deleteSample(board.name));
        container.appendChild(chip);
    }
    updateCoverageStats();
    drawAllOverlays();
}

function viewSample(board) {
    const strip = document.getElementById('capture-strip');
    strip.innerHTML = CAMERAS.map(cam => `
        <div class="shot found">
            <img src="${API}/samples/${board.name}/${cam}.jpg" alt="${LABELS[cam]}">
            <div class="shot-label"><span>${LABELS[cam]}</span><span class="verdict">board ${board.name}</span></div>
        </div>`).join('');
    setMessage(`Board ${board.name}`, '');
    lastCapture = board.corners;
    drawAllOverlays();
}

async function deleteSample(name) {
    if (!confirm(`Delete board ${name}?`)) return;
    const response = await fetch(`${API}/samples/${name}`, {method: 'DELETE'});
    if (!response.ok) {
        setMessage(`Delete failed: ${(await response.json()).error}`, 'bad');
        return;
    }
    await loadSamples();
}

// ============================================================================
// COVERAGE
// ============================================================================

// Outer corners of the board, in drawing order
function outline(corners) {
    const [cols, rows] = patternSize;
    return [corners[0], corners[cols - 1], corners[cols * rows - 1], corners[cols * (rows - 1)]];
}

function area(polygon) {
    let sum = 0;
    for (let i = 0; i < polygon.length; i++) {
        const [x1, y1] = polygon[i];
        const [x2, y2] = polygon[(i + 1) % polygon.length];
        sum += x1 * y2 - x2 * y1;
    }
    return Math.abs(sum) / 2;
}

// Boards touching each grid cell (a board counts once per cell it has corners in)
function cellCounts(cam) {
    const counts = Array.from({length: GRID}, () => new Array(GRID).fill(0));
    for (const board of boards) {
        const cells = new Set();
        for (const [x, y] of board.corners[cam]) {
            const col = Math.min(GRID - 1, Math.max(0, Math.floor(x * GRID)));
            const row = Math.min(GRID - 1, Math.max(0, Math.floor(y * GRID)));
            cells.add(row * GRID + col);
        }
        cells.forEach(cell => counts[Math.floor(cell / GRID)][cell % GRID]++);
    }
    return counts;
}

function updateCoverageStats() {
    document.getElementById('n-boards').textContent = boards.length;
    const areas = boards.map(b => area(outline(b.corners.rgb_left)));
    document.getElementById('n-near').textContent = areas.filter(a => a > NEAR_AREA).length;
    document.getElementById('n-far').textContent = areas.filter(a => a < FAR_AREA).length;
    document.getElementById('n-mid').textContent = areas.filter(a => a >= FAR_AREA && a <= NEAR_AREA).length;

    for (const cam of CAMERAS) {
        document.getElementById(`count-${cam}`).textContent = `${boards.length} boards`;
        const counts = cellCounts(cam);
        const missing = [];
        counts.forEach((row, r) => row.forEach((n, c) => { if (n === 0) missing.push(CELL_NAMES[r][c]); }));
        const element = document.getElementById(`missing-${cam}`);
        element.textContent = missing.length ? `Not covered: ${missing.join(', ')}` : 'Whole image covered';
        element.classList.toggle('done', missing.length === 0);
    }
}

function drawAllOverlays() {
    CAMERAS.forEach(drawOverlay);
}

function drawOverlay(cam) {
    const canvas = document.getElementById(`overlay-${cam}`);
    const w = canvas.width = canvas.clientWidth * devicePixelRatio;
    const h = canvas.height = canvas.clientHeight * devicePixelRatio;
    const ctx = canvas.getContext('2d');
    ctx.clearRect(0, 0, w, h);

    if (document.getElementById('show-coverage').checked) {
        // Cells: red = no board yet, amber = 1-2 boards, green = 3 or more
        const counts = cellCounts(cam);
        ctx.font = `${14 * devicePixelRatio}px system-ui, sans-serif`;
        ctx.textAlign = 'left';
        ctx.textBaseline = 'top';
        for (let r = 0; r < GRID; r++) {
            for (let c = 0; c < GRID; c++) {
                const n = counts[r][c];
                ctx.fillStyle = n === 0 ? 'rgba(248, 81, 73, 0.22)' : n < 3 ? 'rgba(210, 153, 34, 0.15)' : 'rgba(63, 185, 80, 0.10)';
                const x = c * w / GRID, y = r * h / GRID;
                ctx.fillRect(x, y, w / GRID, h / GRID);
                ctx.strokeStyle = 'rgba(255, 255, 255, 0.25)';
                ctx.lineWidth = devicePixelRatio;
                ctx.strokeRect(x, y, w / GRID, h / GRID);
                ctx.fillStyle = 'rgba(255, 255, 255, 0.85)';
                ctx.fillText(n, x + 6 * devicePixelRatio, y + 4 * devicePixelRatio);
            }
        }
        ctx.strokeStyle = 'rgba(76, 141, 255, 0.7)';
        ctx.lineWidth = 1.5 * devicePixelRatio;
        for (const board of boards) strokeOutline(ctx, outline(board.corners[cam]), w, h);
    }

    if (lastCapture && lastCapture[cam]) {
        ctx.strokeStyle = '#ffffff';
        ctx.lineWidth = 3 * devicePixelRatio;
        strokeOutline(ctx, outline(lastCapture[cam]), w, h);
    }
}

function strokeOutline(ctx, points, w, h) {
    ctx.beginPath();
    points.forEach(([x, y], i) => i ? ctx.lineTo(x * w, y * h) : ctx.moveTo(x * w, y * h));
    ctx.closePath();
    ctx.stroke();
}

function sleep(ms) {
    return new Promise(resolve => setTimeout(resolve, ms));
}
