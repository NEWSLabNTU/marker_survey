// The plan screen: marker size, zone and spacing from the site and the camera. Every number
// comes from POST /api/plan (marker_survey's core/marker_plan.py); this file only draws them,
// with a side view generated from the same numbers.
'use strict';

(() => {
  const g = (id) => document.getElementById(id);
  const SVGNS = 'http://www.w3.org/2000/svg';
  let presets = {robot: {camera_height: 0.26, speed: 0.5}, handheld: {camera_height: 1.0, speed: 0.5}};
  let timer = null;
  let last = null;
  let seq = 0;

  function preset() {
    return document.querySelector('input[name="pl_preset"]:checked').value;
  }

  function inputs() {
    return {
      preset: preset(), ceiling: g('pl_ceiling').value, camera_height: g('pl_camera').value,
      hfov: g('pl_hfov').value, width: g('pl_w').value, height: g('pl_h').value,
      speed: g('pl_speed').value,
      exposure: g('pl_exposure').value === '' ? '' : +g('pl_exposure').value / 1000,
      route_length: g('pl_length').value, size_cm: g('pl_size').value,
    };
  }

  function row(tbody, label, value, cls) {
    const tr = document.createElement('tr');
    if (cls) tr.className = cls;
    const a = document.createElement('th');
    a.textContent = label;
    const b = document.createElement('td');
    b.textContent = value;
    tr.append(a, b);
    tbody.append(tr);
    return tr;
  }

  function m(v, digits = 2) { return v === null || v === undefined ? '-' : `${(+v).toFixed(digits)} m`; }

  function show(r) {
    last = r;
    const err = g('pl_error');
    err.hidden = r.ok;
    err.textContent = r.ok ? '' : r.error;
    const box = g('pl_numbers');
    box.replaceChildren();
    const t = document.createElement('table');
    t.className = 'numbers';
    const tb = document.createElement('tbody');
    t.append(tb);
    box.append(t);
    if (r.camera_to_ceiling_m === undefined) { g('pl_figure').replaceChildren(); g('pl_print').hidden = true; return; }
    row(tb, 'Camera to ceiling', m(r.camera_to_ceiling_m));
    row(tb, 'Seen on the ceiling', `${m(r.view_w_m)} x ${m(r.view_h_m)}`);
    row(tb, 'Readable marker sizes', `${r.readable_from_cm} to ${r.fits_up_to_cm} cm`);
    if (r.size_cm !== undefined) {
      row(tb, r.chosen ? 'Marker size (chosen)' : 'Recommended marker size',
          `${r.size_cm} cm black square (${r.px_per_cell} px per cell)`, 'key');
      row(tb, 'Printed', `${r.printed_cm} cm square with its white margin: ${r.sheet}`);
      row(tb, 'Zone R', `${m(r.zone_radius_m)}: the whole marker is in view within R of it`);
    }
    if (r.route_spacing_no_odom_m !== undefined) {
      row(tb, 'Spacing D, no odometry', `${m(r.route_spacing_no_odom_m)} along a route (2R): a marker always in view`, 'key');
      row(tb, 'Spacing on a grid', `${m(r.grid_spacing_m)} (a marker in view anywhere in the area)`);
      row(tb, 'Spacing with odometry', `${m(r.route_spacing_m)} along a route (a robot)`);
    }
    if (r.marker_count !== undefined) {
      row(tb, 'Markers for the route', `${r.marker_count} for ${r.route_length_m} m`, 'key');
    }
    drawSide(r);
    printLink(r);
  }

  function printLink(r) {
    const a = g('pl_print');
    a.hidden = !r.ok || r.size_cm === undefined;
    if (a.hidden) return;
    const first = Math.max(0, parseInt(g('pl_first_id').value || '0', 10) || 0);
    const p = new URLSearchParams({size: String(r.size_cm), unit: 'cm'});
    if (r.marker_count) {
      p.set('count', String(r.marker_count));
      p.set('ids', `${first}-${first + r.marker_count - 1}`);
      a.textContent = `Print ${r.marker_count} markers of ${r.size_cm} cm (IDs ${first} to ${first + r.marker_count - 1})`;
    } else {
      a.textContent = `Print markers of ${r.size_cm} cm`;
    }
    a.href = `#print?${p.toString()}`;
  }

  // ------------------------------------------------------------------ the side view
  function svg(tag, attrs, text) {
    const e = document.createElementNS(SVGNS, tag);
    for (const [k, v] of Object.entries(attrs || {})) e.setAttribute(k, v);
    if (text !== undefined) e.textContent = text;
    return e;
  }

  function drawSide(r) {
    const box = g('pl_figure');
    box.replaceChildren();
    const H = r.inputs.ceiling;
    const hc = r.inputs.camera_height;
    const S = (r.size_cm || 0) / 100;
    const D = r.route_spacing_no_odom_m || 0;
    const halfView = r.view_w_m / 2;
    const span = Math.max(halfView, D ? 1.6 * D : 0, 1.0) + 0.4;    // metres each side of the camera
    const W = 640, Ht = 380, padL = 46, padR = 16, padT = 64, padB = 24;
    const s = Math.min((W - padL - padR) / (2 * span), (Ht - padT - padB) / (H + 0.15));
    const X = (x) => padL + (x + span) * s;
    const Y = (z) => Ht - padB - z * s;
    const root = svg('svg', {viewBox: `0 0 ${W} ${Ht}`, role: 'img', class: 'side',
                             'aria-label': `side view: camera ${hc} m above the floor, ceiling ${H} m, ` +
                               `markers of ${r.size_cm} cm every ${D} m`});
    root.append(svg('title', {}, 'Side view of the camera, its field of view and the markers'));
    // floor and ceiling
    root.append(svg('line', {x1: X(-span), y1: Y(0), x2: X(span), y2: Y(0), class: 'floor'}));
    root.append(svg('line', {x1: X(-span), y1: Y(H), x2: X(span), y2: Y(H), class: 'ceiling'}));
    root.append(svg('text', {x: X(span) - 4, y: Y(0) - 6, class: 'lbl end'}, 'floor'));
    root.append(svg('text', {x: X(span) - 4, y: Y(H) + 16, class: 'lbl end'}, `ceiling ${H} m`));
    // the field of view: the cone and the width it covers on the ceiling
    const cx = X(0), cy = Y(hc);
    const cl = Math.max(-span, -halfView), cr = Math.min(span, halfView);
    root.append(svg('polygon', {points: `${cx},${cy} ${X(cl)},${Y(H)} ${X(cr)},${Y(H)}`, class: 'cone'}));
    root.append(svg('text', {x: cx, y: Y(H) + 26, class: 'lbl mid'},
                    `seen: ${r.view_w_m.toFixed(2)} x ${r.view_h_m.toFixed(2)} m`));
    // above the ceiling: the zone R around the marker over the camera, and the spacing D
    const dim = (x0, x1, y, label, cls) => {
      root.append(svg('line', {x1: X(x0), y1: y, x2: X(x1), y2: y, class: cls || 'dim'}));
      for (const x of [x0, x1]) root.append(svg('line', {x1: X(x), y1: y - 5, x2: X(x), y2: y + 5, class: 'dim'}));
      root.append(svg('text', {x: X((x0 + x1) / 2), y: y - 7, class: 'lbl mid'}, label));
    };
    if (r.zone_radius_m > 0) {
      const R = r.zone_radius_m;
      root.append(svg('rect', {x: X(-R), y: Y(H) - 3, width: 2 * R * s, height: 3, class: 'zone'}));
      dim(-R, R, Y(H) - 16, `zone: R ${R.toFixed(2)} m each side`, 'dim zone-dim');
    }
    if (D && D <= span) dim(0, D, Y(H) - 44, `D ${D.toFixed(2)} m`);
    // markers of size S at spacing D
    if (S > 0) {
      const xs = D ? [] : [0];
      if (D) for (let k = -Math.floor(span / D); k <= Math.floor(span / D); k++) xs.push(k * D);
      for (const x of xs) {
        if (x - S / 2 < -span || x + S / 2 > span) continue;
        root.append(svg('rect', {x: X(x - S / 2), y: Y(H), width: Math.max(2, S * s), height: 6, class: 'marker'}));
      }
    }
    // the camera and its height
    root.append(svg('rect', {x: cx - 9, y: cy - 4, width: 18, height: 10, rx: 2, class: 'cam'}));
    root.append(svg('line', {x1: cx, y1: Y(0), x2: cx, y2: cy + 6, class: preset() === 'handheld' ? 'person' : 'mast'}));
    root.append(svg('text', {x: cx + 14, y: (Y(0) + cy) / 2, class: 'lbl'}, `camera ${hc} m`));
    root.append(svg('line', {x1: X(-span) + 20, y1: cy, x2: X(-span) + 20, y2: Y(H), class: 'dim'}));
    root.append(svg('text', {x: X(-span) + 26, y: (cy + Y(H)) / 2, class: 'lbl'},
                    `${r.camera_to_ceiling_m.toFixed(2)} m`));
    box.append(root);
  }

  // ------------------------------------------------------------------ wiring
  async function update() {
    const n = ++seq;
    let r;
    try {
      const res = await fetch('/api/plan', {method: 'POST', headers: {'Content-Type': 'application/json'},
                                            body: JSON.stringify(inputs())});
      r = await res.json();
    } catch (e) {
      r = {ok: false, error: 'the server does not answer'};
    }
    if (n === seq) show(r);
  }

  function later() { clearTimeout(timer); timer = setTimeout(update, 200); }

  function applyPreset() {
    const p = presets[preset()];
    if (!p) return;
    g('pl_camera').value = p.camera_height;
    g('pl_speed').value = p.speed;
    later();
  }

  async function fromCamera() {
    try {
      const st = await (await fetch('/api/state')).json();
      const ci = st.camera_info;
      if (!ci) { g('pl_camera_note').textContent = 'No camera_info received yet.'; return; }
      g('pl_hfov').value = ci.hfov_deg;
      g('pl_w').value = ci.width;
      g('pl_h').value = ci.height;
      g('pl_camera_note').textContent = `From ${ci.topic || 'the camera'}: ${ci.width} x ${ci.height}, ` +
        `fx ${ci.fx} px, so ${ci.hfov_deg} deg (2 atan(w / 2 fx)).`;
      later();
    } catch (e) {
      g('pl_camera_note').textContent = 'The camera does not answer.';
    }
  }

  async function init() {
    for (const r of document.querySelectorAll('input[name="pl_preset"]')) r.addEventListener('change', applyPreset);
    g('pl_form').addEventListener('input', later);
    g('pl_form').addEventListener('change', later);
    g('pl_first_id').addEventListener('input', () => { if (last) printLink(last); });
    g('pl_from_camera').addEventListener('click', fromCamera);
    window.addEventListener('resize', () => { if (last && last.ok) drawSide(last); });
    update();
    try {
      const info = await (await fetch('/api/info')).json();
      if (info.presets) presets = info.presets;
      const st = await (await fetch('/api/state')).json();
      if (st.robot) {
        g('pl_from_camera').hidden = false;
        g('pl_camera_note').textContent = st.camera_info ? '' : 'The web node reads no camera_info (camera_info_topic).';
      }
    } catch (e) { /* the numbers still work */ }
  }

  init();
})();
