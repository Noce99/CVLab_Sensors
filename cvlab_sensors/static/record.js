// Start / stop the recorders of each sensor group, live previews, save or discard the session.
// OUTPUTS, GROUPS (set by record.html): [[key, label]] in display order.

const API = '/record/api';
const PREVIEW_MS = 600;
// Aspect ratio of each preview: RGB full frame (ROI, from the settings), event sensor, Livox virtual camera, Hesai panorama
const RGB_ASPECT = `${RGB_SIZE[0]} / ${RGB_SIZE[1]}`;
const ASPECT = {rgb_left: RGB_ASPECT, rgb_right: RGB_ASPECT, event_left: '16 / 9', event_right: '16 / 9',
                lidar_livox: '15 / 4', lidar_hesai: '9 / 1'};

document.addEventListener('DOMContentLoaded', () => {
    const groups = document.getElementById('groups');
    for (const [group, label] of GROUPS) {
        groups.insertAdjacentHTML('beforeend', `
            <div class="record-group">
                <span class="dot" id="gdot-${group}"></span>
                <span class="record-group-name">${label}</span>
                <button class="btn-secondary" id="toggle-${group}">Start</button>
            </div>`);
        document.getElementById(`toggle-${group}`).addEventListener('click', () => toggleGroup(group));
    }
    const previews = document.getElementById('previews');
    for (const [output, label] of OUTPUTS) {
        previews.insertAdjacentHTML('beforeend', `
            <div class="camera">
                <div class="camera-header">
                    <span class="dot" id="dot-${output}"></span>
                    <h2>${label}</h2>
                    <span class="muted" id="rate-${output}">-</span>
                    <span class="muted" id="last-${output}"></span>
                </div>
                <div class="camera-error" id="error-${output}"></div>
                <div class="stage" style="aspect-ratio: ${ASPECT[output]}">
                    <div class="placeholder" id="placeholder-${output}">Not recording</div>
                    <img id="live-${output}" alt="">
                </div>
            </div>`);
        previewLoop(output);
    }
    document.getElementById('start-all').addEventListener('click', () => post('start/all'));
    document.getElementById('stop-all').addEventListener('click', () => post('stop/all', 'Stopping (the recorders flush their data)…'));
    document.getElementById('save').addEventListener('click', () => post('save'));
    document.getElementById('discard').addEventListener('click', () => {
        if (confirm('Delete the whole session?')) post('discard');
    });
    updateStatus();
    setInterval(updateStatus, 1000);
});

let status = null;

async function updateStatus() {
    try {
        status = await (await fetch(`${API}/status`)).json();
    } catch (error) {
        setMessage('Server not reachable', 'bad');
        return;
    }
    document.getElementById('session').textContent = status.session || 'none';
    document.getElementById('size').textContent = status.size_gb !== null ? `${status.size_gb} GB` : '-';
    document.getElementById('free').textContent = `${status.free_gb} GB`;
    document.getElementById('event-cap').textContent = status.event_max_rate ? `${status.event_max_rate / 1e6} Mev/s` : 'no cap';
    const running = Object.values(status.groups).some(g => g.running);
    document.getElementById('save').disabled = running || !status.session;
    document.getElementById('discard').disabled = running || !status.session;
    for (const [group, g] of Object.entries(status.groups)) {
        const button = document.getElementById(`toggle-${group}`);
        button.textContent = g.running ? 'Stop' : 'Start';
        button.classList.toggle('active', g.running);
        const errors = g.outputs.some(o => status.outputs[o].error);
        document.getElementById(`gdot-${group}`).className = 'dot ' + (!g.running ? '' : errors ? 'bad' : 'ok');
    }
    for (const [output, s] of Object.entries(status.outputs)) {
        document.getElementById(`rate-${output}`).textContent = s.running ? `${s.fps} /s` : '';
        document.getElementById(`last-${output}`).textContent = s.last_update ? `last ${s.last_update}` : '';
        document.getElementById(`error-${output}`).textContent = s.running ? (s.error || '') : '';
        document.getElementById(`dot-${output}`).className = 'dot ' + (!s.running ? '' : s.error ? 'bad' : s.active ? 'ok' : 'warn');
    }
    renderSettings(status.settings, status.settings_warnings, running);
    if (status.message) setMessage(status.message, 'warn');
}

// Settings the recorders use: changed from the defaults, older than the saved ones, not applied
function renderSettings(settings, warnings, running) {
    const lines = warnings.map(w => `<div class="banner bad">⚠ ${escapeHtml(w)}</div>`);
    for (const s of ['rgb', 'event', 'livox', 'hesai'].map(group => settings[group])) {
        let changed = s.changed.length ? s.changed.map(escapeHtml).join(', ') : 'defaults';
        if (s.kind === 'readonly') {
            changed = (s.reported.length ? s.reported.map(escapeHtml).join(', ') : 'not read yet')
                + ' <span class="muted">(its own settings, changed on its web page)</span>';
        }
        const at = s.saved_at ? ` <span class="muted">(saved ${s.saved_at.replace('T', ' ')})</span>` : '';
        lines.push(`<div><strong>${s.label}:</strong> ${changed}${at}</div>`);
        if (s.stale.length) {
            lines.push(`<div class="banner pending">⚠ <strong>${s.stale.join(', ')}</strong> ${s.stale.length === 1 ? 'records' : 'record'}
                with older settings: the changes saved since the start apply to the next recording (stop and start ${escapeHtml(s.label)}).</div>`);
        }
        for (const note of s.notes) lines.push(`<div class="banner pending">⚠ Not applied as saved: ${escapeHtml(note)}</div>`);
    }
    const prefix = running ? '' : '<div class="muted">The next recording uses:</div>';
    const html = prefix + lines.join('');
    const el = document.getElementById('settings-summary');
    if (el.dataset.html !== html) {
        el.innerHTML = html;
        el.dataset.html = html;
    }
}

function escapeHtml(text) {
    return String(text).replace(/[&<>"]/g, c => ({'&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;'})[c]);
}

async function previewLoop(output) {
    const img = document.getElementById(`live-${output}`);
    const placeholder = document.getElementById(`placeholder-${output}`);
    while (true) {
        const running = status && status.outputs[output] && status.outputs[output].running;
        if (!running) {
            placeholder.style.display = '';
            placeholder.textContent = 'Not recording';
            img.removeAttribute('src');
            await sleep(1000);
            continue;
        }
        try {
            const response = await fetch(`${API}/preview/${output}`, {cache: 'no-store'});
            if (!response.ok) throw new Error((await response.json()).error);
            const url = URL.createObjectURL(await response.blob());
            await new Promise(resolve => { img.onload = img.onerror = resolve; img.src = url; });
            if (img.dataset.url) URL.revokeObjectURL(img.dataset.url);
            img.dataset.url = url;
            placeholder.style.display = 'none';
        } catch (error) {
            placeholder.style.display = '';
            placeholder.textContent = `Waiting… (${error.message})`;
        }
        await sleep(PREVIEW_MS);
    }
}

async function toggleGroup(group) {
    const running = status && status.groups[group].running;
    await post(`${running ? 'stop' : 'start'}/${group}`, running ? 'Stopping…' : 'Starting…');
}

async function post(path, pendingText) {
    if (pendingText) setMessage(pendingText, '');
    const response = await fetch(`${API}/${path}`, {method: 'POST'});
    const result = await response.json();
    if (!response.ok) setMessage(result.error, 'bad');
    else if (result.saved) setMessage(`Saved as data/sessions/${result.saved}`, 'ok');
    else setMessage('', '');
    updateStatus();
}

function setMessage(text, kind) {
    const el = document.getElementById('message');
    el.textContent = text;
    el.className = 'message ' + kind;
}

function sleep(ms) {
    return new Promise(resolve => setTimeout(resolve, ms));
}
