// Runs calibrate.py on a processed_data folder and shows its progress and results.

const LABELS = {rgb_left: 'RGB left', rgb_right: 'RGB right', event_left: 'Event left', event_right: 'Event right'};
const MIN_BOARDS = 10;    // calibrate.py refuses fewer (after removing duplicates)
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
    const data = await (await fetch('/api/folders')).json();
    const select = document.getElementById('folder');
    select.innerHTML = '';
    for (const f of data.folders) {
        const option = new Option(`${f.name} — ${f.boards} boards (${f.modified})`, f.name);
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
    const response = await fetch('/api/calibrate', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({folder, install: document.getElementById('install').checked}),
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
        job = await (await fetch('/api/job')).json();
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

function fmt(x, digits = 2) {
    return Number(x).toFixed(digits);
}

// Slider at 0 = only the event image, 100 = only the RGB one
function updateBlend() {
    const rgb = document.getElementById('blend-slider').value / 100;
    document.getElementById('blend-event').style.opacity = 1 - rgb;
}

function showResult(r) {
    if (shownResult === r.results) return;
    shownResult = r.results;
    const run = r.results.split('/').pop();

    document.getElementById('results').hidden = false;
    document.getElementById('results-folder').textContent = `calibration_results/${run}`;
    document.getElementById('n-found').textContent = r.n_found;
    document.getElementById('n-unique').textContent = r.n_unique;
    document.getElementById('n-used').textContent = r.n_used;

    document.getElementById('intrinsics').innerHTML = Object.entries(r.intrinsics).map(([cam, c]) => `
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
            <td class="${rmsClass(e.rms)}">${fmt(e.rms, 3)}</td>
            <td><code>${e.T.map(t => fmt(t, 1)).join(', ')}</code></td>
            <td>${fmt(e.distance_mm, 1)}</td>
            <td>${fmt(e.rotation_deg, 2)}</td>
        </tr>`).join('');

    document.getElementById('blend-rgb').src = `/api/results/${run}/${r.blend.rgb}`;
    document.getElementById('blend-event').src = `/api/results/${run}/${r.blend.event}`;
    updateBlend();

    showRig(r.poses);

    document.getElementById('images').innerHTML = r.images.map(name => {
        const src = `/api/results/${run}/${name}`;
        return `<figure>
            <a href="${src}" target="_blank"><img src="${src}" alt="${name}"></a>
            <figcaption class="muted">${name.replace('.png', '').replaceAll('_', ' ')}</figcaption>
        </figure>`;
    }).join('');
}

// ============================================================================
// CAMERA POSITIONS
// ============================================================================

// Rig axes from the OpenCV ones (x right, y down, z forward)
const rigAxes = p => ({right: p[0], up: -p[1], forward: p[2]});
const RIG_VIEWS = [
    {title: 'Front', note: 'from behind, looking forward', h: 'right', v: 'up'},
    {title: 'Side', note: 'from the right', h: 'forward', v: 'up'},
    {title: 'Top', note: 'from above', h: 'right', v: 'forward'},
];
const RIG_PAIRS = [['rgb_left', 'rgb_right'], ['event_left', 'event_right'],
                   ['rgb_left', 'event_left'], ['rgb_right', 'event_right']];
const SHORT = {rgb_left: 'RGB L', rgb_right: 'RGB R', event_left: 'Event L', event_right: 'Event R'};
const VIEW_W = 360, VIEW_H = 240, MARGIN = 44, MIN_SPAN_MM = 60;
const MARGIN_X = 72;   // room for the names written beside the cameras
const ARROW_PX = 14;   // length of the viewing direction arrow

function showRig(poses) {
    const cams = Object.keys(poses);
    const pos = Object.fromEntries(cams.map(c => [c, rigAxes(poses[c].center)]));
    const dir = Object.fromEntries(cams.map(c => [c, rigAxes(poses[c].axis)]));
    document.getElementById('rig-views').innerHTML = RIG_VIEWS.map(view => rigView(view, cams, pos, dir)).join('');

    document.getElementById('rig-distances').innerHTML = RIG_PAIRS.map(([a, b]) => {
        const d = ['right', 'up', 'forward'].map(k => pos[b][k] - pos[a][k]);
        return `<tr><td>${LABELS[a]}</td><td>${LABELS[b]}</td><td>${fmt(Math.hypot(...d), 1)}</td>
                ${d.map(x => `<td>${fmt(x, 1)}</td>`).join('')}</tr>`;
    }).join('');
}

function rigView(view, cams, pos, dir) {
    const hs = cams.map(c => pos[c][view.h]), vs = cams.map(c => pos[c][view.v]);
    const span = (xs) => Math.max(Math.max(...xs) - Math.min(...xs), MIN_SPAN_MM);
    const scale = Math.min((VIEW_W - 2 * MARGIN_X) / span(hs), (VIEW_H - 2 * MARGIN) / span(vs));
    const hMid = (Math.max(...hs) + Math.min(...hs)) / 2, vMid = (Math.max(...vs) + Math.min(...vs)) / 2;
    const X = h => VIEW_W / 2 + (h - hMid) * scale;
    const Y = v => VIEW_H / 2 - (v - vMid) * scale;   // up on the page
    const at = c => [X(pos[c][view.h]), Y(pos[c][view.v])];
    let svg = '';

    // Distances: dashed line, labelled when long enough on this view to be readable. Lines on top
    // of each other in this view (e.g. both RGB -> event from the side) share one label.
    const labels = [];
    for (const [a, b] of RIG_PAIRS) {
        const [x1, y1] = at(a), [x2, y2] = at(b);
        svg += `<line x1="${x1}" y1="${y1}" x2="${x2}" y2="${y2}" class="rig-link"/>`;
        if (Math.hypot(x2 - x1, y2 - y1) < 24) continue;
        const d = fmt(Math.hypot(...['right', 'up', 'forward'].map(k => pos[b][k] - pos[a][k])), 1);
        const mx = (x1 + x2) / 2, my = (y1 + y2) / 2;
        const same = labels.find(l => Math.hypot(l.mx - mx, l.my - my) < 16);
        if (same) same.text.push(d);
        else labels.push({mx, my, text: [d], vertical: Math.abs(x2 - x1) < Math.abs(y2 - y1)});
    }
    for (const l of labels) {
        // Beside a vertical line (outwards), above a horizontal one
        const right = l.mx > VIEW_W / 2;
        const [x, y, anchor] = l.vertical ? [l.mx + (right ? 8 : -8), l.my + 4, right ? 'start' : 'end']
                                          : [l.mx, l.my - 5, 'middle'];
        svg += `<text x="${x}" y="${y}" class="rig-distance" style="text-anchor: ${anchor}">${l.text.join(' / ')}</text>`;
    }

    // Cameras: dot, viewing direction (⊗ when it points into the page), name. Cameras on top
    // of each other in this view (e.g. left and right from the side) share one name.
    const groups = [];
    for (const c of cams) {
        const [x, y] = at(c);
        const g = groups.find(g => Math.hypot(g.x - x, g.y - y) < 12);
        if (g) { g.names.push(SHORT[c]); g.points.push([x, y]); }
        else groups.push({x, y, names: [SHORT[c]], cam: c, points: [[x, y]]});
    }
    // One arrow per group: the cameras of a group look the same way within a few degrees
    for (const g of groups) {
        const kind = g.cam.startsWith('rgb') ? 'rgb' : 'event';
        const dh = dir[g.cam][view.h], dv = dir[g.cam][view.v];
        const n = Math.hypot(dh, dv);
        if (n > 0.3) {
            const [ux, uy] = [dh / n, -dv / n];
            // Start past the camera of the group furthest along the arrow
            const start = Math.max(...g.points.map(([x, y]) => (x - g.x) * ux + (y - g.y) * uy)) + 7;
            // ... and stop before the nearest other camera
            const nearest = Math.min(...groups.filter(o => o !== g).map(o => Math.hypot(o.x - g.x, o.y - g.y)));
            const length = Math.max(6, Math.min(ARROW_PX, nearest - start - 8));
            g.vertical = Math.abs(uy) > 0.7;
            svg += `<line x1="${g.x + ux * start}" y1="${g.y + uy * start}" x2="${g.x + ux * (start + length)}" y2="${g.y + uy * (start + length)}"
                          class="rig-axis ${kind}" marker-end="url(#arrow-${kind})"/>`;
        }
    }
    for (const c of cams) {
        const [x, y] = at(c);
        const kind = c.startsWith('rgb') ? 'rgb' : 'event';
        if (Math.hypot(dir[c][view.h], dir[c][view.v]) > 0.3) {
            svg += `<circle cx="${x}" cy="${y}" r="5" class="rig-cam ${kind}"/>`;
        } else {
            svg += `<circle cx="${x}" cy="${y}" r="7" class="rig-cam hollow ${kind}"/>
                    <path d="M${x - 4} ${y - 4}L${x + 4} ${y + 4}M${x - 4} ${y + 4}L${x + 4} ${y - 4}" class="rig-axis ${kind}"/>`;
        }
    }
    for (const g of groups) {
        // Above / below, or beside (outwards) when a vertical arrow takes that place
        const right = g.x > VIEW_W / 2;
        const [x, y, anchor] = g.vertical ? [g.x + (right ? 10 : -10), g.y + 4, right ? 'start' : 'end']
                                          : [g.x, g.y + (g.y < VIEW_H / 2 ? -14 : 22), 'middle'];
        svg += `<text x="${x}" y="${y}" class="rig-name" style="text-anchor: ${anchor}">${g.names.join(' / ')}</text>`;
    }

    // Scale bar and axis directions
    const bar = [10, 20, 50, 100, 200].find(mm => mm * scale >= 40) || 200;
    svg += `<line x1="12" y1="${VIEW_H - 12}" x2="${12 + bar * scale}" y2="${VIEW_H - 12}" class="rig-scale"/>
            <text x="${12 + bar * scale / 2}" y="${VIEW_H - 17}" class="rig-distance">${bar} mm</text>
            <text x="${VIEW_W - 10}" y="${VIEW_H - 12}" class="rig-axes">${view.h} → · ${view.v} ↑</text>`;

    return `<figure class="rig-view">
        <svg viewBox="0 0 ${VIEW_W} ${VIEW_H}" role="img" aria-label="${view.title} view of the cameras">
            <defs>${['rgb', 'event'].map(k => `<marker id="arrow-${k}" viewBox="0 0 10 10" refX="8" refY="5"
                markerWidth="4" markerHeight="4" orient="auto-start-reverse"><path d="M0 0L10 5L0 10z" class="rig-arrow ${k}"/></marker>`).join('')}</defs>
            ${svg}
        </svg>
        <figcaption><b>${view.title}</b> <span class="muted">${view.note}</span></figcaption>
    </figure>`;
}
