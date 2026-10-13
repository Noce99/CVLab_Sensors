// Replay of recorded sessions: frames rendered by the server for a time, a timeline to play them.
// LABELS, SENSORS (set by replay.html).

const API = '/replay/api';
const KIND_COLOURS = {rgb: '#4c8dff', event: '#d29922', livox: '#b07cf0', hesai: '#3fc1c9'};

let sessions = [];         // [{name, path, place}] of the recorded sessions
let session = null;        // summary of the open session
let t = 0;                 // host time shown, us
let playing = false;
let inFlight = false;      // a frame request is running
let pending = false;       // another render was asked meanwhile
let lastFrameUrl = null;

document.addEventListener('DOMContentLoaded', async () => {
    for (const id of ['view', 'res', 'lidar-ms', 'depth', 'warp']) document.getElementById(id).addEventListener('change', settingsChanged);
    document.querySelectorAll('.layer-on, .layer-alpha').forEach(el => el.addEventListener('input', () => requestFrame()));
    document.getElementById('session').addEventListener('change', e => openSession(e.target.value));
    document.getElementById('prepare').addEventListener('click', prepare);
    document.getElementById('save-session').addEventListener('click', () => keepOrDiscard('save'));
    document.getElementById('discard-session').addEventListener('click', () => {
        if (confirm(`Delete the whole session ${selectedName()}?`)) keepOrDiscard('discard');
    });
    document.getElementById('play').addEventListener('click', togglePlay);
    document.getElementById('step-back').addEventListener('click', () => step(-1));
    document.getElementById('step-forward').addEventListener('click', () => step(1));
    document.getElementById('scrub').addEventListener('input', e => {
        if (!session) return;
        const [t0, t1] = session.range;
        seek(t0 + (t1 - t0) * e.target.value / 1000);
    });
    document.getElementById('goto').addEventListener('change', e => {
        if (session) seek(session.range[0] + parseFloat(e.target.value || 0) * 1e6);
    });
    document.addEventListener('keydown', e => {
        if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT') return;
        if (e.code === 'Space') { e.preventDefault(); togglePlay(); }
        if (e.code === 'ArrowLeft') { e.preventDefault(); step(-1); }
        if (e.code === 'ArrowRight') { e.preventDefault(); step(1); }
    });
    window.addEventListener('resize', drawAvailability);
    settingsChanged();

    // ?session=<name>&t=<s from the start>&view=<view>: open that moment directly
    const url = new URLSearchParams(location.search);
    if (url.get('view')) {
        document.getElementById('view').value = url.get('view');
        settingsChanged();
    }
    await loadSessions(url.get('session'), url.has('t') ? parseFloat(url.get('t')) : 0);
});

// ============================================================================
// SESSION
// ============================================================================

async function loadSessions(name, offsetS = 0) {
    sessions = await (await fetch(`${API}/sessions`)).json();
    const select = document.getElementById('session');
    select.replaceChildren();
    for (const s of sessions) select.add(new Option(`${s.name}${s.place === 'staging' ? ' (not saved)' : ''}`, s.name));
    const wanted = sessions.find(s => s.name === name) || sessions[0];
    if (wanted) {
        select.value = wanted.name;
        await openSession(wanted.name, offsetS);
    } else {
        session = null;
        document.getElementById('staging-panel').hidden = true;
        document.getElementById('prepare-panel').hidden = true;
        document.getElementById('frame').removeAttribute('src');
        document.getElementById('placeholder').textContent = 'No recorded session: record one first';
    }
}

function selectedName() {
    return document.getElementById('session').value;
}

async function keepOrDiscard(action) {
    const message = document.getElementById('staging-message');
    for (const id of ['save-session', 'discard-session']) document.getElementById(id).disabled = true;
    const response = await fetch(`${API}/session/${selectedName()}/${action}`, {method: 'POST'});
    const result = await response.json();
    for (const id of ['save-session', 'discard-session']) document.getElementById(id).disabled = false;
    if (!response.ok) {
        message.textContent = result.error;
        message.className = 'message bad';
        return;
    }
    message.textContent = '';
    // Saved: stay on it (now in data/sessions/); discarded: open the newest session left
    await loadSessions(result.saved || null, session && result.saved ? (t - session.range[0]) / 1e6 : 0);
}

async function openSession(name, offsetS = 0) {
    pause();
    const entry = sessions.find(s => s.name === name);
    document.getElementById('staging-panel').hidden = !entry || entry.place !== 'staging';
    const response = await fetch(`${API}/session/${name}`);
    session = await response.json();
    if (!response.ok || !session.range) {
        document.getElementById('placeholder').textContent = session.error || 'This session has no data';
        session = null;
        return;
    }
    // Sensors that cannot be reprojected (no calibration) are shown, disabled, with the reason
    document.querySelectorAll('.layer').forEach(row => {
        const why = session.rig_missing[row.dataset.sensor];
        row.classList.toggle('unavailable', !!why);
        row.querySelector('.layer-status').textContent = why || '';
    });
    showPrepare();
    drawAvailability();
    seek(session.range[0] + offsetS * 1e6);
}

function showPrepare() {
    const needed = Object.values(session.has_raw).some(Boolean) && !session.events_prepared;
    document.getElementById('prepare-panel').hidden = !needed;
}

async function prepare() {
    const response = await fetch(`${API}/session/${session.name}/prepare`, {method: 'POST'});
    if (!response.ok) {
        document.getElementById('prepare-message').textContent = (await response.json()).error;
        return;
    }
    document.getElementById('prepare').disabled = true;
    while (true) {
        const status = await (await fetch(`${API}/prepare`)).json();
        const values = Object.values(status.progress);
        const fraction = values.length ? values.reduce((a, b) => a + b) / values.length : 0;
        document.getElementById('prepare-fill').style.width = `${Math.round(fraction * 100)}%`;
        document.getElementById('prepare-pct').textContent = `${Math.round(fraction * 100)}%`;
        if (status.state !== 'running') {
            const message = document.getElementById('prepare-message');
            message.textContent = status.error || '';
            message.className = 'message ' + (status.error ? 'bad' : 'ok');
            break;
        }
        await sleep(1000);
    }
    document.getElementById('prepare').disabled = false;
    await openSession(session.name);
}

// ============================================================================
// FRAMES
// ============================================================================

function settingsChanged() {
    const overlay = document.getElementById('view').value === 'overlay';
    document.getElementById('layers').hidden = !overlay;
    document.getElementById('depth-label').hidden = !overlay;
    document.getElementById('warp-label').hidden = !overlay;
    requestFrame();
}

function layerAlphas() {
    const alphas = {};
    document.querySelectorAll('.layer').forEach(row => {
        const on = row.querySelector('.layer-on').checked && !row.classList.contains('unavailable');
        alphas[row.dataset.sensor] = on ? row.querySelector('.layer-alpha').value / 100 : 0;
    });
    return alphas;
}

function frameUrl() {
    const params = new URLSearchParams({
        t: Math.round(t),
        view: document.getElementById('view').value,
        res: document.getElementById('res').value,
        lidar_ms: document.getElementById('lidar-ms').value,
        depth: document.getElementById('depth').value,
        warp: document.getElementById('warp').value,
        layers: JSON.stringify(layerAlphas()),
    });
    return `${API}/session/${session.name}/frame?${params}`;
}

// One request at a time; a request asked meanwhile is sent when the current one is done
async function requestFrame() {
    if (!session) return;
    if (inFlight) { pending = true; return; }
    inFlight = true;
    try {
        do {
            pending = false;
            const response = await fetch(frameUrl());
            const info = JSON.parse(response.headers.get('X-Frame-Info') || '{}');
            const url = URL.createObjectURL(await response.blob());
            const img = document.getElementById('frame');
            await new Promise(resolve => { img.onload = img.onerror = resolve; img.src = url; });
            if (lastFrameUrl) URL.revokeObjectURL(lastFrameUrl);
            lastFrameUrl = url;
            document.getElementById('placeholder').style.display = 'none';
            showFrameInfo(info);
        } while (pending && !playing);
    } catch (error) {
        document.getElementById('frame-info').textContent = `Frame failed: ${error.message}`;
    } finally {
        inFlight = false;
    }
}

// Per sensor: how far its data is from the time shown, or why it is not shown
function showFrameInfo(info) {
    const parts = [];
    for (const s of SENSORS) {
        if (info.times && s in info.times) {
            const dt = (info.times[s] - t) / 1000;
            parts.push(`${LABELS[s]} ${dt >= 0 ? '+' : ''}${dt.toFixed(0)} ms`);
        }
    }
    document.getElementById('frame-info').textContent = parts.join(' · ') +
        (info.size ? `  ·  ${info.size[0]}×${info.size[1]}` : '') + (info.warp ? `  ·  warp: ${info.warp}` : '');
    document.getElementById('render-ms').textContent = info.render_ms !== undefined ? `rendered in ${info.render_ms} ms` : '';
    document.querySelectorAll('.layer').forEach(row => {
        const s = row.dataset.sensor;
        if (row.classList.contains('unavailable')) return;
        const status = row.querySelector('.layer-status');
        const note = info.notes && info.notes[s];
        status.textContent = note || (info.times && s in info.times ? `${((info.times[s] - t) / 1000).toFixed(0)} ms` : '');
        status.classList.toggle('warn', !!note);
    });
    if (document.getElementById('view').value !== 'overlay' && info.notes) {
        const note = Object.values(info.notes)[0];
        if (note) document.getElementById('frame-info').textContent = note;
    }
}

// ============================================================================
// TIMELINE
// ============================================================================

function seek(time) {
    if (!session) return;
    const [t0, t1] = session.range;
    t = Math.min(Math.max(time, t0), t1);
    updateTimeline();
    requestFrame();
}

function updateTimeline() {
    const [t0, t1] = session.range;
    document.getElementById('scrub').value = Math.round((t - t0) / (t1 - t0) * 1000);
    const date = new Date(t / 1000);
    document.getElementById('time').textContent =
        date.toLocaleTimeString('en-GB', {hour12: false}) + '.' + String(date.getMilliseconds()).padStart(3, '0');
    document.getElementById('offset').textContent = `+${((t - t0) / 1e6).toFixed(3)} / ${((t1 - t0) / 1e6).toFixed(3)} s`;
    if (document.activeElement !== document.getElementById('goto')) {
        document.getElementById('goto').value = ((t - t0) / 1e6).toFixed(3);
    }
    drawPlayhead();
}

async function step(direction) {
    if (!session) return;
    pause();
    const around = await (await fetch(`${API}/session/${session.name}/neighbours?t=${Math.round(t)}`)).json();
    const target = direction < 0 ? around.previous : around.next;
    if (target !== null) seek(target);
}

function togglePlay() {
    if (playing) pause(); else play();
}

function pause() {
    playing = false;
    document.getElementById('play').textContent = '▶';
}

// Real-time playback: each frame shows the time elapsed since the previous one (x speed), as
// fast as the server renders; the time shown is the requested one, the data the nearest before it
async function play() {
    if (!session) return;
    if (t >= session.range[1]) t = session.range[0];
    playing = true;
    document.getElementById('play').textContent = '⏸';
    let wall = performance.now();
    while (playing) {
        await requestFrame();
        while (inFlight && playing) await sleep(5);
        const now = performance.now();
        const speed = parseFloat(document.getElementById('speed').value);
        t += (now - wall) * 1000 * speed;
        wall = now;
        if (t >= session.range[1]) {
            t = session.range[1];
            updateTimeline();
            pause();
            break;
        }
        updateTimeline();
    }
}

function drawAvailability() {
    const canvas = document.getElementById('availability');
    const w = canvas.width = canvas.clientWidth * devicePixelRatio;
    const h = canvas.height = canvas.clientHeight * devicePixelRatio;
    const ctx = canvas.getContext('2d');
    ctx.clearRect(0, 0, w, h);
    if (!session) return;
    const [t0, t1] = session.range;
    const row = h / SENSORS.length;
    ctx.font = `${Math.min(row - 1, 10 * devicePixelRatio)}px system-ui, sans-serif`;
    ctx.textBaseline = 'middle';
    SENSORS.forEach((s, i) => {
        ctx.fillStyle = 'rgba(255, 255, 255, 0.05)';
        ctx.fillRect(0, i * row + 1, w, row - 2);
        ctx.fillStyle = KIND_COLOURS[s.split('_')[0]];
        for (const [a, b] of session.availability[s] || []) {
            ctx.fillRect((a - t0) / (t1 - t0) * w, i * row + 1, Math.max((b - a) / (t1 - t0) * w, 2), row - 2);
        }
        // Name on the left, readable over the bar
        ctx.fillStyle = 'rgba(0, 0, 0, 0.55)';
        ctx.fillRect(0, i * row + 1, ctx.measureText(LABELS[s]).width + 8 * devicePixelRatio, row - 2);
        ctx.fillStyle = '#e6e9ee';
        ctx.fillText(LABELS[s], 4 * devicePixelRatio, i * row + row / 2);
    });
    drawPlayhead();
}

function drawPlayhead() {
    const head = document.getElementById('playhead') || (() => {
        const el = document.createElement('div');
        el.id = 'playhead';
        document.querySelector('.timeline').appendChild(el);
        return el;
    })();
    if (!session) return;
    const canvas = document.getElementById('availability');
    const [t0, t1] = session.range;
    head.style.left = `${canvas.offsetLeft + (t - t0) / (t1 - t0) * canvas.clientWidth}px`;
    head.style.top = `${canvas.offsetTop}px`;
    head.style.height = `${canvas.clientHeight}px`;
}

function sleep(ms) {
    return new Promise(resolve => setTimeout(resolve, ms));
}
