// Settings page: edit, validate and save the sensor settings; show what every camera runs with.
// GROUPS ([[group, label]]), FIELDS ({group: [field]}), OUTPUTS ({group: [output]}), KINDS ({group: "apply" or
// "expected"}) are set by settings.html. Three states per field: edited here (unsaved), saved but not used by
// a running sensor yet (applies at next start), in use. "expected" groups (the Hesai) are never sent to the
// sensor: their values are compared with what it reports (differs).

const API = '/settings/api';
const PREVIEW_MS = 500;

let status = null;
const saved = {};      // group -> saved values (as last loaded)
const form = {};       // group -> values in the form
let errors = {};       // group -> {key: message}
let validateTimer = null;

document.addEventListener('DOMContentLoaded', async () => {
    for (const [group] of GROUPS) {
        document.getElementById(`save-${group}`).addEventListener('click', () => save(group));
        document.getElementById(`discard-${group}`).addEventListener('click', () => {
            form[group] = {...saved[group]};
            errors[group] = {};
            renderFields(group);
            validate(group);
        });
        document.getElementById(`reset-${group}`).addEventListener('click', () => reset(group));
    }
    const hesaiRefresh = document.getElementById('hesai-refresh');
    if (hesaiRefresh) hesaiRefresh.addEventListener('click', async () => {
        hesaiRefresh.disabled = true;
        await postJson('hesai/refresh', {});
        await updateStatus();
        hesaiRefresh.disabled = false;
    });
    for (const tab of document.querySelectorAll('.tab')) {
        tab.addEventListener('click', () => showGroup(tab.dataset.group));
    }
    let initial = GROUPS[0][0];
    try { initial = localStorage.getItem('settings-tab') || initial; } catch (e) {}
    showGroup(FIELDS[initial] ? initial : GROUPS[0][0]);

    await updateStatus();
    for (const [group] of GROUPS) {
        form[group] = {...saved[group]};
        renderFields(group);
        validate(group);
    }
    setInterval(updateStatus, 1000);
    for (const outputs of Object.values(OUTPUTS)) outputs.forEach(previewLoop);
    window.addEventListener('beforeunload', event => {
        if (GROUPS.some(([g]) => editedKeys(g).length)) event.preventDefault();
    });
});

// Show a tab and open its sensors (the server closes the others)
function showGroup(group) {
    for (const tab of document.querySelectorAll('.tab')) tab.classList.toggle('active', tab.dataset.group === group);
    for (const [g] of GROUPS) document.getElementById(`group-${g}`).hidden = g !== group;
    try { localStorage.setItem('settings-tab', group); } catch (e) {}
    postJson('use', {group});
}

// ============================================================================
// FORM
// ============================================================================

function renderFields(group) {
    const body = document.getElementById(`fields-${group}`);
    body.innerHTML = '';
    let section = null;
    for (const f of FIELDS[group]) {
        if (f.section !== section) {
            section = f.section;
            body.insertAdjacentHTML('beforeend',
                `<tr class="section-row"><th colspan="${3 + OUTPUTS[group].length}">${section}</th></tr>`);
        }
        const row = document.createElement('tr');
        row.id = `row-${group}-${f.key}`;
        row.innerHTML = `
            <td class="setting-name">${f.label}${f.geometry ? ' <span class="badge geometry" title="Changes the image geometry: the installed calibrations depend on it">calibration</span>' : ''}
                ${f.help ? `<div class="muted setting-help">${f.help}</div>` : ''}</td>
            <td class="setting-value"></td>
            <td class="muted">${formatValue(f, f.default)}</td>
            ${OUTPUTS[group].map(o => `<td class="setting-running" id="run-${o}-${f.key}"></td>`).join('')}`;
        row.querySelector('.setting-value').append(input(group, f));
        body.append(row);
    }
    refresh(group);
}

function input(group, f) {
    const wrap = document.createElement('div');
    wrap.className = 'setting-input';
    let el;
    if (f.kind === 'bool') {
        el = document.createElement('input');
        el.type = 'checkbox';
        el.checked = !!form[group][f.key];
        el.addEventListener('change', () => edit(group, f.key, el.checked));
    } else if (f.kind === 'choice') {
        el = document.createElement('select');
        el.className = 'field';
        for (const [value, label] of f.choices) el.add(new Option(label, value, false, value === form[group][f.key]));
        el.addEventListener('change', () => edit(group, f.key, el.value));
    } else {
        el = document.createElement('input');
        el.type = 'number';
        el.className = 'field';
        for (const attr of ['min', 'max', 'step']) if (f[attr] !== null) el[attr] = f[attr];
        el.value = form[group][f.key] ?? '';
        if (f.optional) el.placeholder = f.key.includes('rate') ? 'no cap' : 'default';
        el.addEventListener('input', () => edit(group, f.key, el.value === '' ? null : Number(el.value)));
    }
    el.id = `input-${group}-${f.key}`;
    wrap.append(el);
    if (f.unit) wrap.insertAdjacentHTML('beforeend', `<span class="muted">${f.unit}</span>`);
    wrap.insertAdjacentHTML('beforeend', `<span class="badge edited" hidden>unsaved</span>
        <div class="field-error" id="error-${group}-${f.key}"></div>`);
    return wrap;
}

function edit(group, key, value) {
    form[group][key] = value;
    refresh(group);
    clearTimeout(validateTimer);
    validateTimer = setTimeout(() => validate(group), 300);
}

function editedKeys(group) {
    if (!form[group] || !saved[group]) return [];
    return FIELDS[group].filter(f => !same(form[group][f.key], saved[group][f.key])).map(f => f.key);
}

function applies(f, values) {
    return !f.when || values[f.when[0]] === f.when[1];
}

// Rows, badges, errors and the running values of one group (the inputs are left as typed)
function refresh(group) {
    if (!form[group]) return;
    const edited = editedKeys(group);
    const locked = status && status.locked;
    for (const f of FIELDS[group]) {
        const row = document.getElementById(`row-${group}-${f.key}`);
        if (!row) continue;
        row.hidden = !applies(f, form[group]);
        row.classList.toggle('edited', edited.includes(f.key));
        row.querySelector('.badge.edited').hidden = !edited.includes(f.key);
        document.getElementById(`input-${group}-${f.key}`).disabled = !!locked;
        const error = (errors[group] || {})[f.key];
        document.getElementById(`error-${group}-${f.key}`).textContent = error || '';
        row.classList.toggle('invalid', !!error);
        for (const output of OUTPUTS[group]) {
            document.getElementById(`run-${output}-${f.key}`).innerHTML = runningCell(group, output, f);
        }
    }
    const hasErrors = Object.keys(errors[group] || {}).length > 0;
    const saveButton = document.getElementById(`save-${group}`);
    saveButton.disabled = !!locked || !edited.length || hasErrors;
    saveButton.title = locked ? 'Read-only while recording' : hasErrors ? 'Fix the fields in red first' : '';
    document.getElementById(`discard-${group}`).disabled = !edited.length;
    document.getElementById(`reset-${group}`).disabled = !!locked;
    if (edited.length && !hasErrors) {
        setMessage(group, `${edited.length} unsaved edit${edited.length === 1 ? '' : 's'}`, 'edited');
    } else if (document.getElementById(`message-${group}`).classList.contains('edited')) {
        setMessage(group, '', '');
    }
}

// What a camera runs with: the value read back from the camera when it reports one
function runningCell(group, output, f) {
    const entry = status && status.outputs[output];
    if (!entry || entry.state === 'closed' || !entry.report) return '<span class="muted">not open</span>';
    const values = entry.report.values || {};
    if (!applies(f, values) && !applies(f, saved[group] || {})) return '<span class="muted">–</span>';
    const readback = (entry.report.readback || {})[f.key];
    if (KINDS[group] === 'expected') {
        if (readback === undefined) return '<span class="muted">not reported</span>';
        const differs = saved[group] && !same(readback, saved[group][f.key]);
        return `${formatValue(f, readback)}${differs ? ' <span class="badge differs">differs</span>' : ''}`;
    }
    let shown = formatValue(f, values[f.key]);
    let note = '';
    if (readback !== undefined && readback !== null && !same(readback, values[f.key])) {
        // The camera reports something else: its default for an empty field, or a clamped value
        note = values[f.key] === null || values[f.key] === undefined || values[f.key] === 'unchanged'
            ? ` <span class="muted">(${formatValue(f, readback)})</span>`
            : ` <span class="readback" title="Value the camera reports">camera: ${formatValue(f, readback)}</span>`;
    }
    const pending = saved[group] && !same(values[f.key], saved[group][f.key]);
    return `${shown}${note}${pending ? ' <span class="badge pending">applies at next start</span>' : ''}`;
}

async function validate(group) {
    const result = await postJson('validate', {group, values: form[group]});
    if (!result.ok) return;
    errors[group] = result.data.errors;
    showWarnings(group, result.data.warnings);
    refresh(group);
}

function showWarnings(group, warnings) {
    const list = document.getElementById(`warnings-${group}`);
    list.innerHTML = warnings.map(w => `<li>⚠ ${escapeHtml(w)}</li>`).join('');
}

async function save(group) {
    const result = await postJson('save', {group, values: form[group]});
    if (!result.ok) {
        errors[group] = result.data.errors || {};
        setMessage(group, result.data.error, 'bad');
        refresh(group);
        return;
    }
    saved[group] = {...result.data.values};
    form[group] = {...result.data.values};
    errors[group] = {};
    renderFields(group);
    showWarnings(group, result.data.warnings);
    await updateStatus();
    const open = OUTPUTS[group].some(o => status.outputs[o].state !== 'closed');
    if (KINDS[group] === 'expected') setMessage(group, 'Expected values saved. Nothing was sent to the lidar.', 'ok');
    else setMessage(group, open ? 'Saved. Not active yet: it applies at the next start of the sensors (see the banner above).'
                                : 'Saved. It applies the next time the sensors open.', 'warn');
}

async function reset(group) {
    if (!confirm(KINDS[group] === 'expected' ? 'Reset the expected values to their defaults?'
                                             : 'Reset every setting of this group to its default? (applies at next start)')) return;
    const result = await postJson('reset', {group});
    if (!result.ok) { setMessage(group, result.data.error, 'bad'); return; }
    saved[group] = {...result.data.values};
    form[group] = {...result.data.values};
    errors[group] = {};
    renderFields(group);
    validate(group);
    await updateStatus();
    setMessage(group, KINDS[group] === 'expected' ? 'Default expected values saved. Nothing was sent to the lidar.'
                                                  : 'Defaults saved. They apply at the next start of the sensors.', 'warn');
}

// ============================================================================
// STATUS
// ============================================================================

async function updateStatus() {
    try {
        status = await (await fetch(`${API}/status`, {cache: 'no-store'})).json();
    } catch (error) {
        document.getElementById('banners').innerHTML = '<div class="banner bad">Server not reachable</div>';
        return;
    }
    for (const [group] of GROUPS) {
        if (!saved[group] || !editedKeys(group).length) {
            // Nothing edited here: follow the file (saved from another browser...)
            const changed = saved[group] && !same(saved[group], status.saved[group].values);
            saved[group] = {...status.saved[group].values};
            if (changed) {
                form[group] = {...saved[group]};
                renderFields(group);
            }
        } else {
            saved[group] = {...status.saved[group].values};
        }
        const at = status.saved[group].saved_at;
        document.getElementById(`saved-at-${group}`).textContent = at ? `saved ${at.replace('T', ' ')}` : 'all defaults';
        refresh(group);
    }
    renderBanners();
    for (const [output, entry] of Object.entries(status.outputs)) renderCamera(output, entry);
}

function renderBanners() {
    const banners = [];
    // Settings that break the data whatever the tab: shown first, in red
    const problems = [...status.hesai.warnings, ...status.calibration];
    if (problems.length) {
        banners.push(`<div class="banner bad"><strong>⚠ Check before recording:</strong>
            <ul>${problems.map(p => `<li>${escapeHtml(p)}</li>`).join('')}</ul></div>`);
    }
    if (status.locked) {
        banners.push(`<div class="banner bad"><strong>Read-only:</strong> ${escapeHtml(status.locked)}.
            The recording uses the settings saved when it started; stop it on the <a href="/record">Record page</a> to edit.</div>`);
    }
    // Saved changes a running camera does not use yet
    const stale = Object.values(status.outputs).filter(e => e.state !== 'closed' && e.pending.length);
    if (stale.length) {
        const list = stale.map(e => `<li><strong>${e.label}</strong> (${e.where}) runs with older values of:
            ${e.pending.map(escapeHtml).join(', ')}</li>`).join('');
        const canRestart = !status.locked && status.preview;
        banners.push(`<div class="banner pending">
            <strong>⚠ Saved changes are not active yet.</strong>
            They take effect the next time these cameras open: next recording, next calibration capture${canRestart ? ', or restart the preview now' : ''}.
            <ul>${list}</ul>
            ${canRestart ? '<button class="btn-secondary" id="restart">Restart preview now</button>' : ''}
        </div>`);
    }
    const opening = Object.values(status.outputs).filter(e => e.state === 'opening');
    if (opening.length) {
        banners.push(`<div class="banner muted-banner">Opening ${opening.map(e => e.label).join(', ')}…
            (event cameras take a few seconds each)</div>`);
    }
    // (the Hesai problems are in the red banner)
    const notes = Object.values(status.outputs).filter(e => e.group !== 'hesai')
        .flatMap(e => (e.report && e.report.notes || []).map(n => `${e.label}: ${n}`));
    if (notes.length) {
        banners.push(`<div class="banner pending"><strong>The sensors could not apply everything as saved:</strong>
            <ul>${notes.map(n => `<li>${escapeHtml(n)}</li>`).join('')}</ul></div>`);
    }
    const html = banners.join('');
    const container = document.getElementById('banners');
    if (container.dataset.html !== html) {
        container.innerHTML = html;
        container.dataset.html = html;
        const restart = document.getElementById('restart');
        if (restart) restart.addEventListener('click', restartPreview);
    }
}

async function restartPreview() {
    const button = document.getElementById('restart');
    button.disabled = true;
    button.textContent = 'Restarting…';
    await postJson('restart', {});
    await updateStatus();
}

function renderCamera(output, entry) {
    if (entry.group === 'hesai') {
        const rate = status.rates[output];
        const read = status.hesai.read_at ? `settings read ${status.hesai.read_at.replace('T', ' ').slice(11)}` : '';
        document.getElementById(`rate-${output}`).textContent = [rate ? `${rate.rate} ${rate.unit}` : '', read]
            .filter(Boolean).join(' · ');
        const state = document.getElementById(`state-${output}`);
        const differs = status.hesai.warnings.length;
        state.textContent = status.hesai.error || (differs ? 'Differs from the expected values: see the banner above'
                                                           : 'Matches the expected values');
        state.className = 'camera-error' + (status.hesai.error || differs ? '' : ' ok');
        document.getElementById(`dot-${output}`).className = 'dot ' + (status.hesai.error ? 'warn' : differs ? 'bad' : 'ok');
        return;
    }
    const rate = status.rates[output];
    const size = entry.report && entry.report.readback && entry.report.readback.size;
    document.getElementById(`rate-${output}`).textContent = rate && entry.state === 'open'
        ? `${size ? size + ' · ' : ''}${rate.rate} ${rate.unit}` : '';
    const problem = (rate && rate.error && rate.error !== 'starting' ? rate.error : null) || entry.error;
    const state = document.getElementById(`state-${output}`);
    let text = '';
    if (entry.state === 'closed') text = status.locked ? '' : 'Not open: saved settings are used the next time it opens.';
    else if (entry.where === 'Record page') text = 'Recording: preview on the Record page.';
    else if (entry.state === 'opening') text = 'Opening…';
    state.textContent = problem || (rate && rate.warning) || text;
    state.className = 'camera-error' + (problem ? '' : ' warn');
    document.getElementById(`dot-${output}`).className = 'dot ' +
        (problem ? 'bad' : entry.state === 'open' ? (entry.pending.length ? 'warn' : 'ok') : '');
}

async function previewLoop(output) {
    const img = document.getElementById(`live-${output}`);
    const placeholder = document.getElementById(`placeholder-${output}`);
    while (true) {
        const visible = !document.getElementById(`live-${output}`).closest('.settings-layout').hidden;
        // Streamed by the page (the rates come from the live sensors; a recording has its own previews)
        const streamed = status && status.rates[output] && status.outputs[output].where !== 'Record page';
        if (!status || !status.preview || !visible || !streamed) {
            placeholder.style.display = '';
            placeholder.textContent = status && status.locked ? 'Recording: see the Record page' : 'Not open';
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

// ============================================================================
// HELPERS
// ============================================================================

function formatValue(f, value) {
    if (value === null || value === undefined) return f.key.includes('rate') ? 'no cap' : 'camera default';
    if (f.kind === 'bool' || typeof value === 'boolean') return value ? 'on' : 'off';
    if (f.kind === 'choice') return (f.choices.find(c => c[0] === value) || [value, value])[1];
    const number = typeof value === 'number' ? +value.toPrecision(6) : value;
    return f.unit ? `${number} ${f.unit}` : `${number}`;
}

function same(a, b) {
    if (a && b && typeof a === 'object') return JSON.stringify(a) === JSON.stringify(b);
    if (typeof a === 'number' && typeof b === 'number') return Math.abs(a - b) <= 1e-9 * Math.max(1, Math.abs(a));
    return a === b;
}

async function postJson(path, data) {
    try {
        const response = await fetch(`${API}/${path}`, {method: 'POST', headers: {'Content-Type': 'application/json'},
                                                         body: JSON.stringify(data)});
        return {ok: response.ok, data: await response.json()};
    } catch (error) {
        return {ok: false, data: {error: 'Server not reachable'}};
    }
}

function setMessage(group, text, kind) {
    const el = document.getElementById(`message-${group}`);
    el.textContent = text;
    el.className = 'message ' + kind;
}

function escapeHtml(text) {
    return String(text).replace(/[&<>"]/g, c => ({'&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;'})[c]);
}

function sleep(ms) {
    return new Promise(resolve => setTimeout(resolve, ms));
}
