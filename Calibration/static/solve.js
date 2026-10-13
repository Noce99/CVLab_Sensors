// Runs a solver (rgb_event/solve.py, rgb_livox/solve.py) on a processed_data folder and shows its
// progress and results. TOOL (set by solve.html): api, min_boards. Sections without data are hidden.
// Needs rig.js (LABELS, fmt, showRig).

const API = TOOL.api;
const MIN_BOARDS = TOOL.min_boards;   // the solver refuses fewer (after removing duplicates)
const POLL_MS = 300;
// RMS reprojection error shown amber / red above these (px)
const RMS_WARN = 0.5;
const RMS_BAD = 1.0;

let pollTimer = null;
let shownResult = null;   // results folder currently rendered, to avoid reloading the images

document.addEventListener('DOMContentLoaded', async () => {
    document.getElementById('run').addEventListener('click', run);
    document.getElementById('blend-slider').addEventListener('input', updateBlend);
    await loadFolders();
    poll();   // a calibration may already be running (page reloaded)
});

async function loadFolders() {
    const data = await (await fetch(`${API}/folders`)).json();
    const select = document.getElementById('folder');
    select.innerHTML = '';
    for (const f of data.folders) {
        const option = new Option(`${f.name} — ${f.boards} ${TOOL.count_unit || 'boards'} (${f.modified})`, f.name);
        option.disabled = f.boards < MIN_BOARDS;
        select.add(option);
    }
    const firstUsable = data.folders.find(f => f.boards >= MIN_BOARDS);
    if (firstUsable) select.value = firstUsable.name;
    document.getElementById('install-label').title = `Copy the result to ${data.install_path}`;
}

async function run() {
    const folder = document.getElementById('folder').value;
    if (!folder) return;
    const response = await fetch(`${API}/calibrate`, {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({folder, install: document.getElementById('install').checked,
                              params: Object.fromEntries([...document.querySelectorAll('.param')].map(i => [i.dataset.name, i.value]))}),
    });
    if (!response.ok) {
        setMessage((await response.json()).error, 'bad');
        return;
    }
    document.getElementById('results').hidden = true;
    shownResult = null;
    poll();
}

async function poll() {
    clearTimeout(pollTimer);
    let job;
    try {
        job = await (await fetch(`${API}/job`)).json();
    } catch (error) {
        setMessage('Server not reachable', 'bad');
        pollTimer = setTimeout(poll, 2000);
        return;
    }
    showJob(job);
    if (job.state === 'running') pollTimer = setTimeout(poll, POLL_MS);
}

function showJob(job) {
    const running = job.state === 'running';
    document.getElementById('run').disabled = running;
    document.getElementById('folder').disabled = running;

    const fill = document.getElementById('progress-fill');
    const pct = Math.round(job.progress * 100);
    fill.style.width = `${job.state === 'idle' ? 0 : pct}%`;
    fill.className = 'progress-fill ' + (job.state === 'failed' ? 'bad' : job.state === 'done' ? 'ok' : '');
    document.getElementById('progress-pct').textContent = job.state === 'idle' ? '' : `${pct}%`;
    if (job.state !== 'idle') document.getElementById('progress-label').textContent = job.label;

    const log = document.getElementById('log');
    log.textContent = job.log.join('\n');
    log.scrollTop = log.scrollHeight;

    if (running) setMessage(`Calibrating ${job.folder}…`, '');
    else if (job.state === 'failed') setMessage(job.error, 'bad');
    else if (job.state === 'done') {
        setMessage(job.result.installed ? 'Done, installed as current calibration' : 'Done (not installed)', 'ok');
        showResult(job.result);
    }
}

function setMessage(text, kind) {
    const el = document.getElementById('job-message');
    el.textContent = text;
    el.className = 'message ' + kind;
}

function rmsClass(rms) {
    return rms > RMS_BAD ? 'bad' : rms > RMS_WARN ? 'warn' : 'ok';
}

// Board plane offset (mm) shown amber / red above these
const PLANE_WARN_MM = 10;
const PLANE_BAD_MM = 25;

function planeClass(mm) {
    return Math.abs(mm) > PLANE_BAD_MM ? 'bad' : Math.abs(mm) > PLANE_WARN_MM ? 'warn' : 'ok';
}

// Slider at 0 = only blend.images[0] (on top), 100 = only blend.images[1] (below, it sets the size)
function updateBlend() {
    const right = document.getElementById('blend-slider').value / 100;
    document.getElementById('blend-top').style.opacity = 1 - right;
}

function showResult(r) {
    if (shownResult === r.results) return;
    shownResult = r.results;
    const run = r.run;
    const file = name => `${API}/results/${run}/${name}`;

    document.getElementById('results').hidden = false;
    document.getElementById('results-folder').textContent = `calibration_results/${run}`;
    document.getElementById('n-found').textContent = r.n_found;
    document.getElementById('n-unique').textContent = r.n_unique ?? '-';
    document.getElementById('stat-unique').hidden = r.n_unique === undefined;
    document.getElementById('n-used').textContent = r.n_used;
    document.getElementById('stat-square').hidden = r.square_size_mm === undefined;
    document.getElementById('square-size').textContent = r.square_size_mm ?? '-';
    const [foundLabel, usedLabel] = r.count_labels || ['boards found', 'used'];
    document.getElementById('label-found').textContent = foundLabel;
    document.getElementById('label-used').textContent = usedLabel;
    const warnings = document.getElementById('warnings');
    warnings.hidden = !(r.warnings && r.warnings.length);
    warnings.textContent = (r.warnings || []).map(w => `⚠ ${w}`).join('  ·  ');
    const unit = r.rms_unit || 'px';
    document.getElementById('rms-header').textContent = `RMS ${unit}`;
    const extrinsicsNote = document.getElementById('extrinsics-note');
    extrinsicsNote.hidden = !r.extrinsics_note;
    extrinsicsNote.textContent = r.extrinsics_note || '';
    document.getElementById('rig-origin').textContent = r.rig_origin || 'RGB left';
    document.getElementById('tables').innerHTML = (r.tables || []).map(t => `
        <h3 class="table-title">${t.title} <span class="muted">(${t.note})</span></h3>
        <div class="table-wrap">
            <table>
                <thead><tr>${t.columns.map(c => `<th>${c}</th>`).join('')}</tr></thead>
                <tbody>${t.rows.map((row, i) => `<tr>${row.map((cell, j) =>
                    `<td class="${(t.flags && t.flags[i][j]) || ''}">${cell}</td>`).join('')}</tr>`).join('')}</tbody>
            </table>
        </div>`).join('');

    document.getElementById('intrinsics-section').hidden = !r.intrinsics;
    const note = document.getElementById('intrinsics-note');
    note.hidden = !r.intrinsics_note;
    note.textContent = r.intrinsics_note ? `Intrinsics (fixed): ${r.intrinsics_note}` : '';
    document.getElementById('intrinsics').innerHTML = Object.entries(r.intrinsics || {}).map(([cam, c]) => `
        <tr>
            <td>${LABELS[cam] || cam}</td>
            <td class="${rmsClass(c.rms)}">${fmt(c.rms, 3)}</td>
            <td>${fmt(c.median_view_error, 3)}</td>
            <td>${fmt(c.max_view_error, 3)}</td>
            <td>${fmt(c.K[0][0], 1)}</td>
            <td>${fmt(c.K[1][1], 1)}</td>
            <td>${fmt(c.K[0][2], 1)}</td>
            <td>${fmt(c.K[1][2], 1)}</td>
            <td><code>${c.D.map(d => fmt(d, 4)).join(' ')}</code></td>
        </tr>`).join('');

    document.getElementById('extrinsics').innerHTML = r.extrinsics.map(e => `
        <tr>
            <td>${LABELS[e.from]} → ${LABELS[e.to]}</td>
            <td class="${unit === 'px' ? rmsClass(e.rms) : ''}">${fmt(e.rms, unit === 'px' ? 3 : 1)}</td>
            <td><code>${e.T.map(t => fmt(t, 1)).join(', ')}</code></td>
            <td>${fmt(e.distance_mm, 1)}</td>
            <td>${fmt(e.rotation_deg, 2)}</td>
        </tr>`).join('');

    document.getElementById('plane-section').hidden = !r.plane;
    document.getElementById('plane').innerHTML = (r.plane || []).map(p => `
        <tr>
            <td>${p.board}</td>
            <td class="${rmsClass(p.rms_px)}">${fmt(p.rms_px, 2)}</td>
            <td>${p.points}</td>
            <td class="${p.offset_mm === null ? '' : planeClass(p.offset_mm)}">${p.offset_mm === null ? '-' : fmt(p.offset_mm, 1)}</td>
            <td>${p.spread_mm === null ? '-' : fmt(p.spread_mm, 1)}</td>
        </tr>`).join('');

    document.getElementById('blend-section').hidden = !r.blend;
    if (r.blend) {
        document.getElementById('blend-title').textContent = r.blend.title;
        document.getElementById('blend-note').textContent = r.blend.note;
        document.getElementById('blend-top').src = file(r.blend.images[0]);
        document.getElementById('blend-bottom').src = file(r.blend.images[1]);
        document.getElementById('blend-label-0').textContent = r.blend.labels[0];
        document.getElementById('blend-label-1').textContent = r.blend.labels[1];
        updateBlend();
    }

    if (r.images_title) {
        document.getElementById('images-title').textContent = r.images_title;
        document.getElementById('images-note').textContent = `(${r.images_note})`;
    }

    showRig(r.poses, r.rig_pairs);

    document.getElementById('images').innerHTML = r.images.map(name => {
        const src = file(name);
        return `<figure>
            <a href="${src}" target="_blank"><img src="${src}" alt="${name}"></a>
            <figcaption class="muted">${name.replace('.png', '').replaceAll('_', ' ')}</figcaption>
        </figure>`;
    }).join('');
}
