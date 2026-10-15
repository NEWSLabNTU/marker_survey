// The survey, check, localization test and map screens. Shown only when the page is served by
// marker_survey_web_node on the robot: the state arrives over the WebSocket
// /api/live twice a second, camera frames as JPEG while a camera is shown,
// and actions go to the JSON API (/api/survey, /api/check, /api/map).
'use strict';

(() => {
  const $ = (id) => document.getElementById(id);
  const LIGHT = {'good': 'good', 'pass again': 'again', 'bad': 'bad'};
  let S = null;               // the last state
  let ws = null;
  let tab = '';
  let camera = null;          // the camera this page asks for
  let camUrl = null;
  let starting = false;       // "start a new survey" pressed: show the start screen
  let solving = false;
  let reportShown = null;     // null: not decided yet
  let report = null;
  let frameKey = '';
  let checkWhich = null;
  let localizeWhich = null;
  let pointsKey = '';

  function el(tag, cls, text) {
    const e = document.createElement(tag);
    if (cls) e.className = cls;
    if (text !== undefined) e.textContent = text;
    return e;
  }

  function dot(light) {
    const d = el('span', `dot ${LIGHT[light] || 'none'}`);
    d.title = light || 'not judged yet';
    d.setAttribute('aria-label', light || 'not judged yet');
    return d;
  }

  function cm(m) {
    return m === null || m === undefined ? '-' : `${(m * 100).toFixed(1)} cm`;
  }

  async function api(method, path, body) {
    try {
      const r = await fetch(path, {method, headers: {'Content-Type': 'application/json'},
                                   body: body ? JSON.stringify(body) : undefined});
      return await r.json();
    } catch (e) {
      return {ok: false, error: 'the robot does not answer'};
    }
  }

  function show(id, on) { $(id).hidden = !on; }

  function message(id, text, bad) {
    const p = $(id);
    p.hidden = !text;
    p.textContent = text || '';
    p.classList.toggle('bad', !!bad);
  }

  // ------------------------------------------------------------------ connection
  function connect() {
    const proto = location.protocol === 'https:' ? 'wss' : 'ws';
    ws = new WebSocket(`${proto}://${location.host}/api/live`);
    ws.binaryType = 'blob';
    ws.onopen = () => { $('link').textContent = 'connected'; $('link').className = 'link-state ok'; camera = undefined; wantCamera(); };
    ws.onmessage = (ev) => {
      if (typeof ev.data === 'string') onState(JSON.parse(ev.data));
      else onCamera(ev.data);
    };
    ws.onclose = () => {
      $('link').textContent = 'reconnecting...';
      $('link').className = 'link-state bad';
      setTimeout(connect, 2000);
    };
  }

  function wantCamera() {
    let want = null;
    if (tab === 'survey' && !$('sv_run').hidden && $('sv_cam_on').checked) want = 'survey';
    if (tab === 'check' && $('ck_cam_on').checked) want = 'check';
    if (tab === 'localize' && $('lz_cam_on').checked) want = 'check';     // the localizer's camera
    if (want === camera || !ws || ws.readyState !== 1) return;
    camera = want;
    ws.send(JSON.stringify({camera: want}));
    if (!want) for (const id of ['sv_cam', 'ck_cam', 'lz_cam']) $(id).removeAttribute('src');
  }

  function onCamera(blob) {
    const img = camera === 'survey' ? $('sv_cam')
      : camera === 'check' ? (tab === 'localize' ? $('lz_cam') : $('ck_cam')) : null;
    if (!img) return;
    const url = URL.createObjectURL(blob);
    img.src = url;
    if (camUrl) URL.revokeObjectURL(camUrl);
    camUrl = url;
  }

  function onState(st) {
    S = st;
    if (!st.robot) return;
    renderSurvey();
    renderCheck();
    renderLocalize();
    wantCamera();
  }

  // ------------------------------------------------------------------ the map from above
  function css(name) {
    return getComputedStyle(document.documentElement).getPropertyValue(name).trim();
  }

  // layers: {markers: [{x, y, size, light, id}], trail: [[x, y]], robot: {x, y, yaw_deg},
  //          fixes: [{fix: [x, y], prior: [x, y], agrees}],
  //          track: [[x, y, flag]] (flag 1: a kidnap ends here, dashed; 2: a gap, dotted),
  //          points: [{n, x_measured, y_measured, x, y}] (test points)}
  function drawMap(canvas, layers) {
    const dpr = window.devicePixelRatio || 1;
    const w = canvas.clientWidth || 600;
    const h = Math.round(w * 0.7);
    canvas.width = Math.round(w * dpr);
    canvas.height = Math.round(h * dpr);
    canvas.style.height = `${h}px`;
    const g = canvas.getContext('2d');
    g.setTransform(dpr, 0, 0, dpr, 0, 0);
    g.clearRect(0, 0, w, h);
    const pts = [];
    for (const m of layers.markers || []) pts.push([m.x, m.y]);
    for (const p of layers.trail || []) pts.push(p);
    for (const f of layers.fixes || []) { pts.push(f.fix); if (f.prior) pts.push(f.prior); }
    for (const p of layers.track || []) pts.push([p[0], p[1]]);
    for (const p of layers.points || []) { pts.push([p.x_measured, p.y_measured]); pts.push([p.x, p.y]); }
    if (layers.robot) pts.push([layers.robot.x, layers.robot.y]);
    if (!pts.length) {
      g.fillStyle = css('--muted');
      g.font = '14px system-ui, sans-serif';
      g.fillText('Nothing to show yet.', 16, 28);
      return;
    }
    let [x0, y0, x1, y1] = [Infinity, Infinity, -Infinity, -Infinity];
    for (const [x, y] of pts) { x0 = Math.min(x0, x); x1 = Math.max(x1, x); y0 = Math.min(y0, y); y1 = Math.max(y1, y); }
    const pad = 1.0;
    x0 -= pad; y0 -= pad; x1 += pad; y1 += pad;
    const s = Math.min(w / (x1 - x0), h / (y1 - y0));
    const ox = (w - s * (x1 - x0)) / 2, oy = (h - s * (y1 - y0)) / 2;
    const X = (x) => ox + (x - x0) * s;
    const Y = (y) => h - oy - (y - y0) * s;          // y up, as on a map
    // a 1 m grid
    g.strokeStyle = css('--line');
    g.lineWidth = 1;
    g.beginPath();
    for (let x = Math.ceil(x0); x <= x1; x++) { g.moveTo(X(x), 0); g.lineTo(X(x), h); }
    for (let y = Math.ceil(y0); y <= y1; y++) { g.moveTo(0, Y(y)); g.lineTo(w, Y(y)); }
    g.stroke();
    g.fillStyle = css('--muted');
    g.font = '11px system-ui, sans-serif';
    g.fillText('1 m grid', 6, h - 6);
    // the robot's path
    const trail = layers.trail || [];
    if (trail.length > 1) {
      g.strokeStyle = css('--accent');
      g.globalAlpha = 0.6;
      g.lineWidth = 2;
      g.beginPath();
      trail.forEach(([x, y], i) => (i ? g.lineTo(X(x), Y(y)) : g.moveTo(X(x), Y(y))));
      g.stroke();
      g.globalAlpha = 1;
    }
    // a localizer's track: solid where fixes follow each other, dashed red across a kidnap
    const track = layers.track || [];
    for (let i = 1; i < track.length; i++) {
      const [a, b] = [track[i - 1], track[i]];
      const kind = b[2] || 0;
      g.strokeStyle = kind === 1 ? css('--bad') : kind === 2 ? css('--muted') : css('--accent');
      g.lineWidth = kind === 1 ? 2.5 : 2;
      g.setLineDash(kind === 1 ? [8, 6] : kind === 2 ? [2, 4] : []);
      g.beginPath();
      g.moveTo(X(a[0]), Y(a[1]));
      g.lineTo(X(b[0]), Y(b[1]));
      g.stroke();
      if (kind === 1) {
        g.fillStyle = css('--bad');
        g.beginPath();
        g.arc(X(b[0]), Y(b[1]), 4, 0, 2 * Math.PI);
        g.fill();
      }
    }
    g.setLineDash([]);
    // test points: the measured spot (a cross), a line to the localizer's pose there
    g.font = '11px system-ui, sans-serif';
    for (const p of layers.points || []) {
      const [mx, my] = [X(p.x_measured), Y(p.y_measured)];
      g.strokeStyle = css('--warn');
      g.lineWidth = 2;
      g.beginPath();
      g.moveTo(mx - 6, my); g.lineTo(mx + 6, my); g.moveTo(mx, my - 6); g.lineTo(mx, my + 6);
      g.stroke();
      g.lineWidth = 1;
      g.beginPath();
      g.moveTo(mx, my); g.lineTo(X(p.x), Y(p.y));
      g.stroke();
      g.fillStyle = css('--warn');
      g.fillText(String(p.n), mx + 7, my + 12);
    }
    // fixes and the robot's position at each
    for (const f of layers.fixes || []) {
      if (f.prior) {
        g.fillStyle = css('--accent');
        g.fillRect(X(f.prior[0]) - 1.5, Y(f.prior[1]) - 1.5, 3, 3);
      }
      g.fillStyle = f.agrees === false ? css('--bad') : css('--ok');
      g.beginPath();
      g.arc(X(f.fix[0]), Y(f.fix[1]), 2.5, 0, 2 * Math.PI);
      g.fill();
    }
    // markers
    const colour = {good: css('--ok'), again: css('--warn'), bad: css('--bad')};
    g.font = 'bold 12px system-ui, sans-serif';
    for (const m of layers.markers || []) {
      const half = Math.max(5, (m.size || 0.3) * s / 2);
      g.fillStyle = colour[LIGHT[m.light]] || css('--muted');
      g.fillRect(X(m.x) - half, Y(m.y) - half, 2 * half, 2 * half);
      g.strokeStyle = css('--ink');
      g.lineWidth = 1;
      g.strokeRect(X(m.x) - half, Y(m.y) - half, 2 * half, 2 * half);
      g.fillStyle = css('--ink');
      g.fillText(String(m.id), X(m.x) + half + 3, Y(m.y) - half);
    }
    // the robot
    const r = layers.robot;
    if (r) {
      const a = r.yaw_deg * Math.PI / 180;
      const L = 12;
      g.fillStyle = css('--accent');
      g.beginPath();
      g.moveTo(X(r.x) + L * Math.cos(a), Y(r.y) - L * Math.sin(a));
      g.lineTo(X(r.x) + L * 0.6 * Math.cos(a + 2.4), Y(r.y) - L * 0.6 * Math.sin(a + 2.4));
      g.lineTo(X(r.x) + L * 0.6 * Math.cos(a - 2.4), Y(r.y) - L * 0.6 * Math.sin(a - 2.4));
      g.closePath();
      g.fill();
    }
  }

  // ------------------------------------------------------------------ survey
  function surveyView(sv) {
    if (solving || S.busy === 'solving') return 'solving';
    if (starting) return 'start';
    if (sv.state === 'recording') return 'run';
    if (!sv.keyframes) return 'start';
    if (reportShown === null) reportShown = !!(sv.last_save && sv.last_save.ok);
    return reportShown ? 'report' : 'run';
  }

  function renderSurvey() {
    const sv = S.survey;
    const online = S.connected && S.connected.survey;
    show('sv_offline', !online);
    if (!sv) {
      for (const id of ['sv_start', 'sv_run', 'sv_report', 'sv_solving']) show(id, false);
      return;
    }
    const view = surveyView(sv);
    show('sv_start', view === 'start');
    show('sv_run', view === 'run');
    show('sv_report', view === 'report');
    show('sv_solving', view === 'solving');
    if (view === 'start') renderStart(sv);
    if (view === 'run') renderRun(sv);
    if (view === 'report' && !report) loadReport();
  }

  function renderStart(sv) {
    const opts = sv.frame_options || [];
    const key = JSON.stringify(opts);
    if (key === frameKey) return;
    frameKey = key;
    const box = $('sv_frames');
    box.replaceChildren();
    let first = true;
    for (const o of opts) {
      const choice = el('label', 'choice frame');
      const r = el('input');
      r.type = 'radio';
      r.name = 'sv_frame';
      r.value = o.use;
      r.disabled = !o.available;
      if (o.available && first) { r.checked = true; first = false; }
      choice.append(r, el('strong', '', o.label));
      if (o.use === 'marker') {
        const n = el('input');
        n.type = 'number';
        n.min = '0';
        n.step = '1';
        n.id = 'sv_origin';
        n.placeholder = 'number';
        n.setAttribute('aria-label', 'marker number');
        n.addEventListener('focus', () => { r.checked = true; });
        choice.append(n);
      }
      const t = el('small', '', o.available ? o.text : o.why);
      const wrap = el('div', 'option');
      wrap.append(choice, t);
      box.append(wrap);
    }
    if (sv.frame && sv.frame.mode === 'origin_marker' && sv.frame.origin_marker !== null && $('sv_origin')) {
      $('sv_origin').value = sv.frame.origin_marker;
    }
    box.append(el('p', 'bad', ''));
    box.lastChild.id = 'sv_error';
  }

  function lostText(sv) {
    if (sv.tracking !== 'lost') return '';
    const since = sv.lost_since === null || sv.lost_since === undefined ? ''
      : ` (${Math.round(sv.lost_since)} s)`;
    return sv.last_known_marker === null || sv.last_known_marker === undefined
      ? `No mapped marker in view${since}: go back to where a mapped marker is in view.`
      : `No mapped marker in view${since}: go back to marker ${sv.last_known_marker}.`;
  }

  function renderRun(sv) {
    const rec = sv.state === 'recording';
    const handheld = (sv.pose_source || (sv.frame || {}).pose_source) === 'markers';
    const lost = lostText(sv);
    show('sv_lost', !!lost);
    $('sv_lost').textContent = lost;
    $('sv_path_label').textContent = handheld ? 'the path walked' : 'the robot\'s path';
    $('sv_state').textContent = !sv.camera ? (rec ? 'Recording: waiting for the camera' : 'Waiting for the camera')
      : rec ? (handheld ? 'Recording: walk slowly, keep a mapped marker in view'
        : 'Recording: drive slowly past every marker') : 'Paused';
    $('sv_state').className = `state ${rec ? 'rec' : ''}`;
    const L = sv.lights || {};
    $('sv_counts').textContent = `${sv.markers.length} markers (${L.good || 0} good, ` +
      `${L['pass again'] || 0} to pass again${L.bad ? `, ${L.bad} bad` : ''}), ${sv.keyframes} keyframes` +
      (sv.dropped ? `; ${sv.dropped} frames without ${handheld ? 'a mapped marker in view' : 'a robot position'}` : '');
    $('sv_pause').textContent = rec ? 'Pause' : 'Continue recording';
    const inview = (sv.detections || []).map((d) => `${d.id} (${d.state === 'unusable'
      ? (d.why === 'border' ? 'cut off' : 'too small') : d.state})`);
    $('sv_inview').textContent = inview.length ? `In view: ${inview.join(', ')}` : 'No marker in view.';
    const hints = $('sv_hints');
    hints.replaceChildren();
    const f = sv.frame || {};
    if (f.mode === 'origin_marker' && !sv.origin_seen) {
      hints.append(el('li', 'warn', `marker ${f.origin_marker}, where the map starts, has not been seen yet: drive past it`));
    }
    for (const h of (sv.hints || []).slice(0, 8)) hints.append(el('li', '', h));
    if ((sv.hints || []).length > 8) hints.append(el('li', 'note', `and ${sv.hints.length - 8} more`));
    if (!hints.children.length) {
      hints.append(el('li', 'note', sv.markers.length
        ? 'Every marker so far is seen well. Pass the rest, then save.' : 'No marker seen yet.'));
    }
    const body = $('sv_markers');
    body.replaceChildren();
    for (const m of sv.markers) {
      const tr = el('tr');
      const td = el('td');
      td.append(dot(m.light));
      tr.append(td, el('td', 'num', m.id), el('td', 'num', `${m.views}x`),
                el('td', 'note', m.hint || (m.source === 'report' ? 'good' : 'good so far')));
      body.append(tr);
    }
    drawMap($('sv_map'), {markers: sv.markers, trail: sv.trail, robot: sv.robot});
  }

  async function startSurvey() {
    const r = document.querySelector('input[name="sv_frame"]:checked');
    if (!r) return;
    const frame = {use: r.value};
    if (r.value === 'marker') frame.marker = $('sv_origin').value;
    $('sv_start_btn').disabled = true;
    const res = await api('POST', '/api/survey', {action: 'start', frame});
    $('sv_start_btn').disabled = false;
    if (!res.ok) { $('sv_error').textContent = res.error; return; }
    $('sv_error').textContent = '';
    starting = false;
    reportShown = false;
    report = null;
  }

  async function save() {
    solving = true;
    renderSurvey();
    const res = await api('POST', '/api/survey', {action: 'save'});
    solving = false;
    if (!res.ok) {
      reportShown = false;
      alertIn('sv_hints', `Not solved: ${res.error}`);
      renderSurvey();
      return;
    }
    report = res;
    reportShown = true;
    renderSurvey();
    renderReport();
  }

  function alertIn(id, text) {
    const li = el('li', 'bad', text);
    $(id).prepend(li);
  }

  async function loadReport() {
    const res = await api('GET', '/api/report');
    if (res.ok && res.report) {
      report = res;
      renderReport();
    }
  }

  function renderReport() {
    const rep = report && report.report;
    if (!rep) return;
    const c = rep.summary || {};
    const n = rep.markers.length;
    $('rp_summary').textContent = `${n} markers: ${c.good || 0} good, ${c['pass again'] || 0} to pass ` +
      `again, ${c.bad || 0} bad`;
    const f = rep.frame || {};
    const where = f.mode === 'origin_marker' ? `starting from marker ${f.origin_marker}`
      : f.mode === 'align_to_track' ? `in the robot's map (${f.frame_id}), fitted to the robot's path ` +
        `within ${cm(f.fit_rms_m)} RMS` : `in the frame ${f.frame_id}`;
    $('rp_detail').textContent = `Placed ${where}. ${rep.solve.keyframes} keyframes; the marker ` +
      `corners fit within ${(rep.solve.reproj_rms_px || 0).toFixed(2)} pixels. Solved ${rep.date}.`;
    $('rp_again').textContent = (c['pass again'] || c.bad)
      ? 'Drive again past the marked ones' : 'Drive again (add more views)';
    const body = $('rp_markers');
    body.replaceChildren();
    for (const m of rep.markers) {
      const tr = el('tr');
      const td = el('td');
      td.append(dot(m.verdict));
      const note = m.verdict === 'good' ? '' : [m.hint, ...(m.flags || [])].filter(Boolean).join('; ');
      tr.append(td, el('td', 'num', m.id), el('td', 'num', `${m.views}x`),
                el('td', 'num', m.sd_xy_cm === null ? '-' : `${m.sd_xy_cm.toFixed(1)} cm`),
                el('td', 'note', note));
      body.append(tr);
    }
    drawMap($('rp_map'), {markers: rep.markers.map((m) => ({x: m.x, y: m.y, id: m.id, light: m.verdict,
                                                           size: rep.marker_size}))});
    const hist = $('rp_history');
    hist.replaceChildren();
    for (const h of report.history || []) hist.append(el('li', '', `${h.name}: ${h.files.join(', ')}`));
    if (!hist.children.length) hist.append(el('li', 'note', 'None: this is the first solve.'));
  }

  // ------------------------------------------------------------------ check
  function renderCheck() {
    const loc = S.localizer;
    const online = S.connected && S.connected.localizer;
    show('ck_offline', !online);
    const reads = S.map && S.map.localizer_reads;
    if (reads && reads !== checkWhich) {
      checkWhich = reads;
      for (const r of document.querySelectorAll('input[name="ck_which"]')) r.checked = r.value === reads;
    }
    const surveyMap = S.map && S.map.survey_map;
    document.querySelector('input[name="ck_which"][value="survey"]').disabled = !surveyMap;
    if (loc) {
      $('ck_reads').textContent = `The localizer reads ${loc.map}` +
        (loc.markers && loc.markers.length ? ` (${loc.markers.length} markers).` : ' (no map yet).') +
        (loc.prior_topic ? '' : ' It has no robot position to compare with: its prior_topic is empty.');
    }
    const c = S.check;
    if (!c) return;
    const last = c.last;
    $('ck_dist').textContent = last && last.distance_m !== null ? cm(last.distance_m) : '-';
    $('ck_median').textContent = cm(c.distance_median_m);
    $('ck_agree').textContent = c.with_prior ? `${Math.round(100 * c.agreed / c.with_prior)} %` : '-';
    $('ck_fixes').textContent = String(c.fixes);
    const flags = $('ck_flags');
    flags.replaceChildren();
    for (const id of c.flagged) {
      flags.append(el('p', 'notice bad', `Marker ${id} disagrees with the robot's position: it has moved, ` +
        'or it was surveyed badly. Check that it is still in place, then survey it again.'));
    }
    const body = $('ck_markers');
    body.replaceChildren();
    for (const m of c.markers) {
      const tr = el('tr');
      if (m.flagged) tr.className = 'flagged';
      const td = el('td');
      td.append(dot(m.flagged ? 'bad' : m.agreement === null ? null : 'good'));
      tr.append(td, el('td', 'num', m.id), el('td', 'num', m.seen), el('td', 'num', m.used),
                el('td', 'num', m.agreement === null ? '-' : `${Math.round(m.agreement * 100)} % of ${m.compared}`),
                el('td', 'num', cm(m.mean_distance_m)),
                el('td', 'note', m.flagged ? 'disagrees: moved, or surveyed badly'
                  : m.outlier ? `left out ${m.outlier}x` : ''));
      body.append(tr);
    }
    const s = c.settings;
    $('ck_rule').textContent = `A fix agrees when it lies within ${cm(s.agree_distance)} and ` +
      `${s.agree_angle_deg} degrees of the robot's own position. A marker is flagged when it ` +
      `disagrees in more than ${Math.round(s.max_disagree * 100)} % of at least ${s.min_samples} fixes.`;
    if (tab === 'check') {
      drawMap($('ck_map'), {markers: (loc && loc.markers || []).map((m) => ({...m, light: null, size: 0.2})),
                            fixes: c.recent});
    }
  }

  async function chooseCheckMap(which) {
    const res = await api('POST', '/api/check', {action: 'map', which});
    if (!res.ok) {
      $('ck_reads').textContent = res.error;
      checkWhich = null;
    }
  }

  // ------------------------------------------------------------------ localization test
  function secs(v) {
    if (v === null || v === undefined) return '-';
    return v < 10 ? `${v.toFixed(1)} s` : `${Math.round(v)} s`;
  }

  function metres(v) { return v === null || v === undefined ? '-' : `${(+v).toFixed(2)} m`; }

  function renderLocalize() {
    const loc = S.localizer;
    const lz = S.localize;
    show('lz_offline', !(S.connected && S.connected.localizer));
    const reads = S.map && S.map.localizer_reads;
    if (reads && reads !== localizeWhich) {
      localizeWhich = reads;
      for (const r of document.querySelectorAll('input[name="lz_which"]')) r.checked = r.value === reads;
    }
    document.querySelector('input[name="lz_which"][value="survey"]').disabled = !(S.map && S.map.survey_map);
    if (loc) {
      $('lz_reads').textContent = `The localizer reads ${loc.map}` +
        (loc.markers && loc.markers.length ? ` (${loc.markers.length} markers), mode ${loc.mode}.`
          : ' (no map yet: survey first, or choose a map).');
    }
    if (!lz) return;
    const p = lz.pose;
    const threeD = (loc && loc.mode === '3d') || (p && p.z !== undefined);
    $('lz_x').textContent = p ? metres(p.x) : '-';
    $('lz_y').textContent = p ? metres(p.y) : '-';
    show('lz_z_box', threeD);
    $('lz_z').textContent = p && p.z !== undefined ? metres(p.z) : '-';
    $('lz_yaw').textContent = p ? `${Math.round(p.yaw_deg) || 0} deg` : '-';
    const age = lz.fix_age_s;
    $('lz_age').textContent = secs(age);
    const gap = (lz.settings && lz.settings.gap_s) || 1.0;
    const stale = age !== null && age !== undefined && age > gap;
    show('lz_nofix', stale);
    $('lz_nofix').textContent = stale
      ? `No marker in view for ${Math.round(age)} s: the pose shown is where the last marker was seen.` : '';
    $('lz_kidnaps').textContent = String(lz.kidnaps.length);
    const ev = $('lz_events');
    ev.replaceChildren();
    for (const k of lz.kidnaps.slice(-5)) {
      ev.append(el('li', 'bad', `kidnap: no fix for ${k.gap_s} s, then the pose jumped ${k.jump_m.toFixed(2)} m ` +
        `(to marker ${k.ids.join(', ')})`));
    }
    if (lz.kidnaps.length > 5) ev.append(el('li', 'note', `and ${lz.kidnaps.length - 5} earlier`));
    renderPoints(lz);
    if (tab === 'localize') {
      drawMap($('lz_map'), {markers: (loc && loc.markers || []).map((m) => ({...m, light: null, size: 0.2})),
                            track: lz.track, points: lz.points,
                            robot: p ? {x: p.x, y: p.y, yaw_deg: p.yaw_deg} : null});
    }
  }

  function renderPoints(lz) {
    const key = JSON.stringify(lz.points);
    if (key === pointsKey) return;
    pointsKey = key;
    const body = $('lz_points');
    body.replaceChildren();
    for (const p of lz.points) {
      const tr = el('tr');
      if (p.stale) tr.className = 'flagged';
      const del = el('button', 'small', 'delete');
      del.addEventListener('click', () => pointAction({action: 'delete', n: p.n}));
      const td = el('td');
      td.append(del);
      tr.append(el('td', 'num', p.n), el('td', 'num', `${p.x_measured.toFixed(2)}, ${p.y_measured.toFixed(2)}`),
                el('td', 'num', `${p.x.toFixed(2)}, ${p.y.toFixed(2)}`), el('td', 'num', cm(p.error_m)),
                el('td', 'num', secs(p.fix_age_s) + (p.stale ? ' (stale)' : '')), el('td', 'note', p.markers), td);
      body.append(tr);
    }
    const e = lz.errors;
    $('lz_errors').textContent = e.count
      ? `${e.count} test point${e.count > 1 ? 's' : ''}: error mean ${cm(e.mean_m)}, RMS ${cm(e.rms_m)}, ` +
        `largest ${cm(e.max_m)}.` : 'No test point yet.';
  }

  async function pointAction(body) {
    const res = await api('POST', '/api/localize', body);
    $('lz_point_msg').textContent = res.ok
      ? (res.point ? `Point ${res.point.n}: error ${cm(res.point.error_m)}` : '') : res.error;
    $('lz_point_msg').className = res.ok ? 'note' : 'bad';
    const st = await api('GET', '/api/state');
    if (st.robot) onState(st);
  }

  async function chooseLocalizeMap(which) {
    const res = await api('POST', '/api/localize', {action: 'map', which});
    if (!res.ok) {
      $('lz_reads').textContent = res.error;
      localizeWhich = null;
    }
  }

  // ------------------------------------------------------------------ use
  async function loadMap() {
    const res = await api('GET', '/api/map');
    const cur = $('us_current');
    cur.replaceChildren();
    if (!res.ok) { cur.append(el('p', 'bad', res.error)); return; }
    const u = res.in_use;
    if (!u || !u.exists) {
      cur.append(el('p', '', 'No map in use yet.'));
      if (u) cur.append(el('p', 'note', `The localizer will read ${u.path}.`));
    } else {
      cur.append(el('p', 'big', `${u.markers} markers, frame ${u.frame_id}`));
      cur.append(el('p', 'note', `${u.path}, written ${u.modified}`));
      if (u.error) cur.append(el('p', 'bad', u.error));
    }
    const nw = $('us_new');
    nw.replaceChildren();
    const d = res.diff;
    $('us_write').disabled = !d;
    if (!d) {
      nw.append(el('p', 'note', res.diff_error || 'No survey saved yet.'));
    } else {
      nw.append(el('p', 'big', `${d.markers} markers, frame ${d.frame_id}`));
      const parts = [];
      if (d.first) parts.push('There is no map in use yet: this will be the first.');
      else {
        if (d.added.length) parts.push(`new: ${d.added.join(', ')}`);
        if (d.removed.length) parts.push(`not in the survey: ${d.removed.join(', ')}`);
        if (d.moved.length) parts.push(`moved more than 2 cm: ${d.moved.map((m) => `${m.id} (${cm(m.distance_m)})`).join(', ')}`);
        parts.push(`${d.unchanged} unchanged`);
        if (d.old_frame_id !== d.frame_id) parts.push(`the frame changes from ${d.old_frame_id} to ${d.frame_id}`);
      }
      for (const p of parts) nw.append(el('p', '', p));
      nw.append(el('p', 'note', d.source));
      $('us_confirm_text').textContent = `Write the survey's ${d.markers} markers to ${d.target}? ` +
        (d.first ? '' : 'The map in use now is kept as an earlier version. ') +
        'The localizer reads the new map at once.';
    }
    const vs = $('us_versions');
    vs.replaceChildren();
    for (const v of (u && u.versions) || []) {
      vs.append(el('li', '', `${v.date}: ${v.markers !== undefined ? `${v.markers} markers` : v.error}`));
    }
    if (!vs.children.length) vs.append(el('li', 'note', 'None yet.'));
    $('us_undo').disabled = !(u && u.can_undo);
    if (u && u.can_undo) {
      $('us_undo_text').textContent = `Put back the version of ${u.versions[0].date}? The map in use ` +
        'now is kept too, and the localizer reads the restored map at once.';
    }
  }

  async function mapAction(action) {
    show('us_confirm', false);
    show('us_undo_confirm', false);
    const res = await api('POST', '/api/map', {action, confirm: true});
    if (!res.ok) { message('us_result', res.error, true); return; }
    const text = action === 'use'
      ? `Written to ${res.written}.` + (res.kept ? ` The previous map is kept as ${res.kept}.` : '')
      : `Put back ${res.restored}.` + (res.kept ? ` The replaced map is kept as ${res.kept}.` : '');
    message('us_result', `${text} ${res.reload}`, false);
    loadMap();
  }

  // ------------------------------------------------------------------ wiring
  async function init() {
    const st = await api('GET', '/api/state');
    if (!st.robot) return;              // a laptop: printing and identifying only
    document.querySelectorAll('.tab.robot').forEach((t) => { t.hidden = false; });
    $('link').hidden = false;
    document.addEventListener('tabchange', (e) => {
      tab = e.detail;
      if (tab === 'use') loadMap();
      if (S) onState(S);
      wantCamera();
    });
    showTab();                           // the address may name a robot tab
    $('sv_start_btn').addEventListener('click', startSurvey);
    $('sv_pause').addEventListener('click', async () => {
      const rec = S && S.survey && S.survey.state === 'recording';
      await api('POST', '/api/survey', {action: rec ? 'pause' : 'resume'});
    });
    $('sv_save').addEventListener('click', save);
    $('sv_cam_on').addEventListener('change', wantCamera);
    $('ck_cam_on').addEventListener('change', wantCamera);
    $('rp_again').addEventListener('click', async () => {
      const res = await api('POST', '/api/survey', {action: 'resume'});
      if (res.ok) { reportShown = false; report = null; }
    });
    $('rp_new').addEventListener('click', () => { starting = true; frameKey = ''; renderSurvey(); });
    for (const r of document.querySelectorAll('input[name="ck_which"]')) {
      r.addEventListener('change', () => chooseCheckMap(r.value));
    }
    $('ck_reset').addEventListener('click', () => api('POST', '/api/check', {action: 'reset'}));
    for (const r of document.querySelectorAll('input[name="lz_which"]')) {
      r.addEventListener('change', () => chooseLocalizeMap(r.value));
    }
    $('lz_cam_on').addEventListener('change', wantCamera);
    $('lz_reset').addEventListener('click', () => pointAction({action: 'reset', points: false}));
    $('lz_clear').addEventListener('click', () => pointAction({action: 'clear_points'}));
    $('lz_record').addEventListener('click', () => pointAction({action: 'point', x: $('lz_px').value,
                                                                 y: $('lz_py').value}));
    $('us_write').addEventListener('click', () => show('us_confirm', true));
    $('us_no').addEventListener('click', () => show('us_confirm', false));
    $('us_yes').addEventListener('click', () => mapAction('use'));
    $('us_undo').addEventListener('click', () => show('us_undo_confirm', true));
    $('us_undo_no').addEventListener('click', () => show('us_undo_confirm', false));
    $('us_undo_yes').addEventListener('click', () => mapAction('undo'));
    window.addEventListener('resize', () => { if (S) onState(S); if (report) renderReport(); });
    onState(st);
    connect();
  }

  init();
})();
