// Sensor positions of a rig: front, side and top SVG views and a distance table, drawn into
// #rig-views and #rig-distances. Used by the solve pages and the home page.

const LABELS = {rgb_left: 'RGB left', rgb_right: 'RGB right', event_left: 'Event left', event_right: 'Event right',
                livox: 'Livox', hesai: 'Hesai'};

function fmt(x, digits = 2) {
    return Number(x).toFixed(digits);
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
const SHORT = {rgb_left: 'RGB L', rgb_right: 'RGB R', event_left: 'Event L', event_right: 'Event R', livox: 'Livox', hesai: 'Hesai'};
const KINDS = ['rgb', 'event', 'livox', 'hesai'];
const kindOf = cam => cam.split('_')[0];
let rigPairs = [];   // [[cam, cam]] drawn and listed: the pairs of the result whose cameras are both known
const VIEW_W = 360, VIEW_H = 240, MARGIN = 44, MIN_SPAN_MM = 60;
const MARGIN_X = 72;   // room for the names written beside the cameras
const ARROW_PX = 14;   // length of the viewing direction arrow
// Approximate text widths (px per character) of the names (bold) and of the distances
const NAME_CHAR_PX = 6.6;
const LABEL_CHAR_PX = 6;

// [x0, y0, x1, y1] of a text drawn at (x, y) (baseline) with the given text-anchor
function textBox(x, y, anchor, text, charPx) {
    const w = text.length * charPx;
    const x0 = anchor === 'start' ? x : anchor === 'end' ? x - w : x - w / 2;
    return [x0, y - 10, x0 + w, y + 2];
}

function overlaps(a, b) {
    return a[0] < b[2] && b[0] < a[2] && a[1] < b[3] && b[1] < a[3];
}

function showRig(poses, pairs) {
    const cams = Object.keys(poses);
    rigPairs = pairs.filter(([a, b]) => a in poses && b in poses);
    const pos = Object.fromEntries(cams.map(c => [c, rigAxes(poses[c].center)]));
    const dir = Object.fromEntries(cams.map(c => [c, rigAxes(poses[c].axis)]));
    document.getElementById('rig-views').innerHTML = RIG_VIEWS.map(view => rigView(view, cams, pos, dir)).join('');

    document.getElementById('rig-distances').innerHTML = rigPairs.map(([a, b]) => {
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
    // Cameras on top of each other in this view (e.g. left and right from the side) are one group:
    // one name, one arrow (they look the same way within a few degrees)
    const groups = [];
    for (const c of cams) {
        const [x, y] = at(c);
        const g = groups.find(g => Math.hypot(g.x - x, g.y - y) < 12);
        if (g) { g.names.push(SHORT[c]); g.points.push([x, y]); }
        else groups.push({x, y, names: [SHORT[c]], cam: c, points: [[x, y]]});
    }

    let arrows = '';
    for (const g of groups) {
        const kind = kindOf(g.cam);
        const dh = dir[g.cam][view.h], dv = dir[g.cam][view.v];
        const n = Math.hypot(dh, dv);
        if (n > 0.3) {
            const [ux, uy] = [dh / n, -dv / n];
            // Start past the camera of the group furthest along the arrow ...
            const start = Math.max(...g.points.map(([x, y]) => (x - g.x) * ux + (y - g.y) * uy)) + 7;
            // ... and stop before the nearest other camera
            const nearest = Math.min(...groups.filter(o => o !== g).map(o => Math.hypot(o.x - g.x, o.y - g.y)));
            const length = Math.max(6, Math.min(ARROW_PX, nearest - start - 8));
            g.vertical = Math.abs(uy) > 0.7;
            arrows += `<line x1="${g.x + ux * start}" y1="${g.y + uy * start}" x2="${g.x + ux * (start + length)}" y2="${g.y + uy * (start + length)}"
                          class="rig-axis ${kind}" marker-end="url(#arrow-${kind})"/>`;
        }
    }

    let dots = '';
    for (const c of cams) {
        const [x, y] = at(c);
        const kind = kindOf(c);
        if (Math.hypot(dir[c][view.h], dir[c][view.v]) > 0.3) {
            dots += `<circle cx="${x}" cy="${y}" r="5" class="rig-cam ${kind}"/>`;
        } else {   // looking into the page
            dots += `<circle cx="${x}" cy="${y}" r="7" class="rig-cam hollow ${kind}"/>
                    <path d="M${x - 4} ${y - 4}L${x + 4} ${y + 4}M${x - 4} ${y + 4}L${x + 4} ${y - 4}" class="rig-axis ${kind}"/>`;
        }
    }

    // Names: above / below, or beside (outwards) when a vertical arrow takes that place.
    // Their boxes, and the cameras, are obstacles for the distance labels.
    const obstacles = groups.map(g => [g.x - 9, g.y - 9, g.x + 9, g.y + 9]);
    let names = '';
    for (const g of groups) {
        const right = g.x > VIEW_W / 2;
        const [x, y, anchor] = g.vertical ? [g.x + (right ? 10 : -10), g.y + 4, right ? 'start' : 'end']
                                          : [g.x, g.y + (g.y < VIEW_H / 2 ? -14 : 22), 'middle'];
        const text = g.names.join(' / ');
        obstacles.push(textBox(x, y, anchor, text, NAME_CHAR_PX));
        names += `<text x="${x}" y="${y}" class="rig-name" style="text-anchor: ${anchor}">${text}</text>`;
    }

    // Distances: dashed line, labelled when long enough on this view to be readable. Lines on top
    // of each other in this view (e.g. both RGB -> event from the side) share one label.
    let lines = '';
    const labels = [];
    for (const [a, b] of rigPairs) {
        const [x1, y1] = at(a), [x2, y2] = at(b);
        lines += `<line x1="${x1}" y1="${y1}" x2="${x2}" y2="${y2}" class="rig-link"/>`;
        if (Math.hypot(x2 - x1, y2 - y1) < 24) continue;
        const d = fmt(Math.hypot(...['right', 'up', 'forward'].map(k => pos[b][k] - pos[a][k])), 1);
        const mx = (x1 + x2) / 2, my = (y1 + y2) / 2;
        const same = labels.find(l => Math.hypot(l.mx - mx, l.my - my) < 16);
        if (same) same.text.push(d);
        else labels.push({x1, y1, x2, y2, mx, my, text: [d], vertical: Math.abs(x2 - x1) < Math.abs(y2 - y1)});
    }
    let distances = '';
    for (const l of labels) {
        // Candidates along the line (middle first), beside a vertical line / above then below a
        // horizontal one: the first that hits no camera, name or other label
        const text = l.text.join(' / ');
        const candidates = [];
        for (const t of [0.5, 0.35, 0.65, 0.25, 0.75]) {
            const px = l.x1 + (l.x2 - l.x1) * t, py = l.y1 + (l.y2 - l.y1) * t;
            const out = px > VIEW_W / 2;
            candidates.push(l.vertical ? [px + (out ? 8 : -8), py + 4, out ? 'start' : 'end']
                                       : [px, py - 5, 'middle'],
                            l.vertical ? [px + (out ? -8 : 8), py + 4, out ? 'end' : 'start']
                                       : [px, py + 14, 'middle']);
        }
        const [x, y, anchor] = candidates.find(([x, y, anchor]) =>
            !obstacles.some(o => overlaps(o, textBox(x, y, anchor, text, LABEL_CHAR_PX)))) || candidates[0];
        obstacles.push(textBox(x, y, anchor, text, LABEL_CHAR_PX));
        distances += `<text x="${x}" y="${y}" class="rig-distance" style="text-anchor: ${anchor}">${text}</text>`;
    }

    let svg = lines + arrows + dots + distances + names;

    // Scale bar and axis directions
    const bar = [10, 20, 50, 100, 200].find(mm => mm * scale >= 40) || 200;
    svg += `<line x1="12" y1="${VIEW_H - 12}" x2="${12 + bar * scale}" y2="${VIEW_H - 12}" class="rig-scale"/>
            <text x="${12 + bar * scale / 2}" y="${VIEW_H - 17}" class="rig-distance">${bar} mm</text>
            <text x="${VIEW_W - 10}" y="${VIEW_H - 12}" class="rig-axes">${view.h} → · ${view.v} ↑</text>`;

    return `<figure class="rig-view">
        <svg viewBox="0 0 ${VIEW_W} ${VIEW_H}" role="img" aria-label="${view.title} view of the cameras">
            <defs>${KINDS.map(k => `<marker id="arrow-${k}" viewBox="0 0 10 10" refX="8" refY="5"
                markerWidth="4" markerHeight="4" orient="auto-start-reverse"><path d="M0 0L10 5L0 10z" class="rig-arrow ${k}"/></marker>`).join('')}</defs>
            ${svg}
        </svg>
        <figcaption><b>${view.title}</b> <span class="muted">${view.note}</span></figcaption>
    </figure>`;
}
