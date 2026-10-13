// Live view of the Livox and the Hesai, recording of static stops for the target-free calibration.
// TOOL (set by scene_capture.html): cameras, labels, preview_delay_ms, aspect, api.

const CAMERAS = TOOL.cameras;
const LABELS = TOOL.labels;
const API = TOOL.api;
const SLOW_MS = 600;     // preview age shown amber / red above these
const STALE_MS = 2000;

document.addEventListener('DOMContentLoaded', () => {
    buildTiles();
    CAMERAS.forEach(previewLoop);
    updateStatus();
    setInterval(updateStatus, 2000);
    loadSessions();
    loadStops();
    document.getElementById('capture').addEventListener('click', capture);
    document.getElementById('session').addEventListener('change', async e => {
        const response = await fetch(`${API}/session`, {
            method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify({name: e.target.value || null}),
        });
        if (!response.ok) setMessage((await response.json()).error, 'bad');
        document.getElementById('capture-strip').innerHTML = '';
        await Promise.all([updateStatus(), loadStops()]);
    });
    document.addEventListener('keydown', e => {
        if (e.code === 'Space' && !e.repeat && e.target.tagName !== 'INPUT') {
            e.preventDefault();
            capture();
        }
    });
});

function buildTiles() {
    const container = document.getElementById('cameras');
    for (const cam of CAMERAS) {
        container.insertAdjacentHTML('beforeend', `
            <div class="camera">
                <div class="camera-header">
                    <span class="dot" id="dot-${cam}"></span>
                    <h2>${LABELS[cam]}</h2>
                    <span class="age" id="age-${cam}" title="Age of the newest points shown">-</span>
                    <span class="muted" id="rate-${cam}">-</span>
                </div>
                <div class="camera-error" id="error-${cam}"></div>
                <div class="stage" style="aspect-ratio: ${TOOL.aspect[cam][0]} / ${TOOL.aspect[cam][1]}">
                    <div class="placeholder" id="placeholder-${cam}">Waiting for points…</div>
                    <img id="live-${cam}" alt="">
                </div>
            </div>`);
    }
}

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
            await sleep(TOOL.preview_delay_ms[cam]);
        } catch (error) {
            placeholder.style.display = '';
            showAge(cam, null);
            placeholder.textContent = `Waiting for points… (${error.message})`;
            await sleep(1000);
        }
    }
}

function showAge(cam, ageMs) {
    const element = document.getElementById(`age-${cam}`);
    element.textContent = ageMs === null ? '-' : ageMs < 1000 ? `${ageMs} ms old` : `${(ageMs / 1000).toFixed(1)} s old`;
    element.className = 'age' + (ageMs === null ? '' : ageMs > STALE_MS ? ' stale' : ageMs > SLOW_MS ? ' slow' : '');
}

async function updateStatus() {
    try {
        const status = await (await fetch(`${API}/status`)).json();
        document.getElementById('folder').textContent = status.folder;
        const select = document.getElementById('session');
        select.value = [...select.options].some(o => o.value === status.session) ? status.session : '';
        for (const cam of CAMERAS) {
            const s = status.sensors[cam];
            document.getElementById(`rate-${cam}`).textContent = `${s.rate} ${s.unit}`;
            const error = document.getElementById(`error-${cam}`);
            error.textContent = s.error || s.warning || '';
            error.classList.toggle('warn', !s.error && !!s.warning);
            document.getElementById(`dot-${cam}`).className = 'dot ' + (s.error ? 'bad' : s.warning || s.rate === 0 ? 'warn' : 'ok');
        }
    } catch (error) {
        CAMERAS.forEach(cam => {
            document.getElementById(`dot-${cam}`).className = 'dot bad';
            document.getElementById(`error-${cam}`).textContent = 'Server not reachable';
        });
    }
}

async function loadSessions() {
    const list = await (await fetch(`${API}/sessions`)).json();
    const select = document.getElementById('session');
    select.length = 1;   // keep "New session"
    for (const s of list) select.add(new Option(`${s.name} — ${s.boards} stops`, s.name));
}

// ============================================================================
// RECORD
// ============================================================================

let capturing = false;

async function capture() {
    if (capturing) return;
    capturing = true;
    const button = document.getElementById('capture');
    button.disabled = true;
    const seconds = parseFloat(document.getElementById('capture-seconds').value) || TOOL.capture_s;
    const end = Date.now() + seconds * 1000;
    const tick = () => {
        const left = Math.ceil((end - Date.now()) / 1000);
        setMessage(left > 0 ? `Keep the rig and the scene still… ${left} s` : 'Checking the stop…', '');
    };
    tick();
    const countdown = setInterval(tick, 250);
    try {
        const response = await fetch(`${API}/capture`, {
            method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify({seconds}),
        });
        const result = await response.json();
        if (!response.ok) throw new Error(result.error);
        showStop(result);
        if (result.saved) {
            await loadStops(result.saved);
            await loadSessions();   // the first stop creates the session folder
            updateStatus();
        }
    } catch (error) {
        setMessage(`Recording failed: ${error.message}`, 'bad');
    } finally {
        clearInterval(countdown);
        capturing = false;
        button.disabled = false;
    }
}

function showStop(result) {
    const c = result.check;
    document.getElementById('capture-strip').style.gridTemplateColumns = `repeat(${CAMERAS.length}, minmax(0, 1fr))`;
    document.getElementById('capture-strip').innerHTML = CAMERAS.map(cam => `
        <div class="shot ${result.moved ? 'missed' : 'found'}">
            <img src="${result.cameras[cam].image}" alt="${LABELS[cam]}">
            <div class="shot-label"><span>${LABELS[cam]}</span><span>${result.cameras[cam].points.toLocaleString()} points</span></div>
        </div>`).join('');
    if (result.moved) {
        setMessage(`The rig moved during the recording (${c.moved_mm} mm, ${c.moved_deg}°): not saved, keep it still`, 'bad');
    } else if (c.weakest_strength < WEAK_STRENGTH) {
        setMessage(`Saved as stop ${result.saved}: overlap ${Math.round(c.overlap * 100)}%; this place hardly constrains ` +
                   `${c.weakest} — the next stop should have surfaces facing that way`, 'warn');
    } else {
        setMessage(`Saved as stop ${result.saved}: overlap ${Math.round(c.overlap * 100)}%, every direction constrained`, 'ok');
    }
}

async function loadStops(latest) {
    const stops = await (await fetch(`${API}/stops`)).json();
    document.getElementById('n-stops').textContent = stops.length;
    document.getElementById('stops').innerHTML = stops.map(s => `
        <tr class="${s.name === latest ? 'latest-row' : ''}">
            <td>${s.name}</td>
            <td>${s.seconds}</td>
            <td>${s.points.livox.toLocaleString()} / ${s.points.hesai.toLocaleString()}</td>
            <td>${Math.round(s.overlap * 100)}%</td>
            <td>${s.rmse_mm ?? '-'}</td>
            <td><code>${s.t_mm.map(v => v.toFixed(1)).join(', ')}</code></td>
            <td class="${s.weakest_strength < WEAK_STRENGTH ? 'warn' : 'ok'}">${s.weakest} (${s.weakest_strength.toFixed(3)})</td>
            <td><button class="row-delete" data-name="${s.name}" title="Delete">×</button></td>
        </tr>`).join('') || '<tr><td colspan="8" class="muted">No stop yet</td></tr>';
    document.querySelectorAll('.row-delete').forEach(b => b.addEventListener('click', () => deleteStop(b.dataset.name)));
}

async function deleteStop(name) {
    if (!confirm(`Delete stop ${name}?`)) return;
    const response = await fetch(`${API}/stops/${name}`, {method: 'DELETE'});
    if (!response.ok) setMessage(`Delete failed: ${(await response.json()).error}`, 'bad');
    await loadStops();
}

function setMessage(text, kind) {
    const message = document.getElementById('capture-message');
    message.textContent = text;
    message.className = `message ${kind}`;
}

function sleep(ms) {
    return new Promise(resolve => setTimeout(resolve, ms));
}
