// physics_viewer/src/suspension.js
// A 2D front-view suspension tuner. Edit hardpoints (or upload a CSV), run the
// kinematic sweep on simulation_server, and see the linkage animate plus the
// camber / scrub / roll-center / motion-ratio curves. No three.js: pure SVG.

const SERVER = "http://localhost:8000";
const SVGNS = "http://www.w3.org/2000/svg";

// name, label, default y, default z, required?
const HARDPOINTS = [
  ["lca_inboard", "LCA inboard (chassis)", 0.240260,0.118850, true],
  ["uca_inboard", "UCA inboard (chassis)", 0.322560,0.303700, true],
  ["lower_ball_joint", "Lower ball joint", 0.51626,0.139330, true],
  ["upper_ball_joint", "Upper ball joint", 0.495745,0.355738, true],
  ["contact_patch", "Contact patch", 0.585,0.00, true],
  ["pushrod_outboard", "Pushrod @ lower arm", 0.478410,0.154470, false],
  ["rocker_pivot", "Rocker pivot (chassis)", 0.304450,0.340620, false],
  ["rocker_pushrod", "Rocker @ pushrod", 0.378880,0.371610, false],
  ["rocker_damper", "Rocker @ damper", 0.373710,0.402970, false],
  ["damper_inboard", "Damper inboard (chassis)", 0.253899,0.538541, false],
];

const el = (id) => document.getElementById(id);
const status = (msg, err = false) => {
  el("status").textContent = msg;
  el("status").classList.toggle("error", err);
};

// --- build the input grid -------------------------------------------------- //
function buildInputs() {
  const root = el("inputs");
  const head = document.createElement("div");
  head.className = "hp-row head";
  head.innerHTML = "<label>point</label><label>y</label><label>z</label>";
  root.appendChild(head);

  for (const [name, label, y, z, req] of HARDPOINTS) {
    const row = document.createElement("div");
    row.className = "hp-row" + (req ? "" : " optional");
    row.innerHTML = `<label>${label}</label>
      <input type="number" step="0.001" id="hp-${name}-y" value="${y}">
      <input type="number" step="0.001" id="hp-${name}-z" value="${z}">`;
    root.appendChild(row);
  }
}

function readHardpoints() {
  const hp = {};
  for (const [name] of HARDPOINTS) {
    const y = parseFloat(el(`hp-${name}-y`).value);
    const z = parseFloat(el(`hp-${name}-z`).value);
    if (Number.isFinite(y) && Number.isFinite(z)) hp[name] = [y, z];
  }
  return hp;
}

function applyCsv(text) {
  let count = 0;
  for (const raw of text.split(/\r?\n/)) {
    const line = raw.trim();
    if (!line || line.startsWith("#")) continue;
    const parts = line.split(",");
    if (parts.length < 3 || parts[0].trim().toLowerCase() === "name") continue;
    const name = parts[0].trim();
    const yEl = el(`hp-${name}-y`), zEl = el(`hp-${name}-z`);
    if (yEl && zEl) {
      yEl.value = parseFloat(parts[1]);
      zEl.value = parseFloat(parts[2]);
      count++;
    }
  }
  status(`loaded ${count} hardpoints from CSV`);
}

// --- server call ----------------------------------------------------------- //
let trajectory = null, kin = null, frameCount = 0, dt = 1 / 60;

async function run() {
  const params = {
    hardpoints: readHardpoints(),
    travel: parseFloat(el("travel").value) / 1000.0, // mm -> m
  };
  status("Simulating…");
  el("run").disabled = true;
  try {
    const res = await fetch(`${SERVER}/api/simulate`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ scene: "suspension", n_frames: 120, params }),
    });
    if (!res.ok) throw new Error(await res.text());
    trajectory = await res.json();
    kin = trajectory.meta.kinematics;
    frameCount = trajectory.frames.length;
    dt = trajectory.meta.dt || 1 / 60;

    buildScene(trajectory);
    drawCharts(kin);
    showReadouts(kin);
    status(`ok — ${frameCount} frames`);
  } catch (e) {
    status(`Failed: ${e.message}` + ` (is simulation_server running at ${SERVER}?)`, true);
  } finally {
    el("run").disabled = false;
  }
}

// --- SVG linkage scene (front view: y right, z up) ------------------------- //
const svg = el("linkage");
let scene = null; // { toPx, circles:Map, lines:[] }

function computeTransform(frames) {
  let ymin = Infinity, ymax = -Infinity, zmin = 0, zmax = -Infinity;
  for (const fr of frames) {
    for (const id in fr.p) {
      const [, y, z] = fr.p[id];
      ymin = Math.min(ymin, y); ymax = Math.max(ymax, y);
      zmin = Math.min(zmin, z); zmax = Math.max(zmax, z);
    }
  }
  const W = 600, H = 440, pad = 40;
  const span = Math.max(ymax - ymin, zmax - zmin) || 1;
  const scale = (Math.min(W, H) - 2 * pad) / span;
  const cy = (ymin + ymax) / 2, cz = (zmin + zmax) / 2;
  // map world (y,z) -> svg (px,py); z is up so flip vertical
  return (y, z) => [
    W / 2 + (y - cy) * scale,
    H / 2 - (z - cz) * scale,
  ];
}

function buildScene(traj) {
  while (svg.firstChild) svg.removeChild(svg.firstChild);
  const toPx = computeTransform(traj.frames);

  // ground line at z = 0
  const yGround = toPx(0, 0)[1];
  const ground = document.createElementNS(SVGNS, "line");
  ground.setAttribute("x1", 0); ground.setAttribute("x2", 600);
  ground.setAttribute("y1", yGround); ground.setAttribute("y2", yGround);
  ground.setAttribute("stroke", "#2a2e36"); ground.setAttribute("stroke-width", 1);
  svg.appendChild(ground);

  const lines = [];
  for (const lk of traj.links) {
    const ln = document.createElementNS(SVGNS, "line");
    ln.setAttribute("stroke", "#6b7280");
    ln.setAttribute("stroke-width", 2.5);
    ln.setAttribute("stroke-linecap", "round");
    svg.appendChild(ln);
    lines.push({ ln, from: lk.from, to: lk.to });
  }

  const circles = new Map();
  for (const b of traj.bodies) {
    const c = document.createElementNS(SVGNS, "circle");
    c.setAttribute("r", b.static ? 5 : 6);
    c.setAttribute("fill", b.color || (b.static ? "#5b8def" : "#ff5a3c"));
    svg.appendChild(c);
    circles.set(b.id, c);
  }

  scene = { toPx, circles, lines };
  applyFrame(0);
}

function applyFrame(index) {
  if (!scene || !trajectory) return;
  const i = Math.max(0, Math.min(Math.round(index), frameCount - 1));
  const p = trajectory.frames[i].p;
  const px = {};
  for (const id in p) px[id] = scene.toPx(p[id][1], p[id][2]);

  for (const [id, c] of scene.circles) {
    if (!px[id]) continue;
    c.setAttribute("cx", px[id][0]);
    c.setAttribute("cy", px[id][1]);
  }
  for (const { ln, from, to } of scene.lines) {
    if (!px[from] || !px[to]) continue;
    ln.setAttribute("x1", px[from][0]); ln.setAttribute("y1", px[from][1]);
    ln.setAttribute("x2", px[to][0]); ln.setAttribute("y2", px[to][1]);
  }
}

// --- charts (metric vs travel) -------------------------------------------- //
function lineChart(xs, ys, color, xlabel) {
  const W = 260, H = 150, pad = 28;
  const xmin = Math.min(...xs), xmax = Math.max(...xs);
  const ymin = Math.min(...ys), ymax = Math.max(...ys);
  const sx = (x) => pad + (x - xmin) / ((xmax - xmin) || 1) * (W - 2 * pad);
  const sy = (y) => (H - pad) - (y - ymin) / ((ymax - ymin) || 1) * (H - 2 * pad);

  const pts = xs.map((x, i) => `${sx(x).toFixed(1)},${sy(ys[i]).toFixed(1)}`).join(" ");
  const zeroY = ymin < 0 && ymax > 0 ? sy(0) : null;
  const zeroX = xmin < 0 && xmax > 0 ? sx(0) : null;

  let g = `<svg viewBox="0 0 ${W} ${H}" xmlns="${SVGNS}">`;
  g += `<line x1="${pad}" y1="${H - pad}" x2="${W - pad}" y2="${H - pad}" stroke="#2a2e36"/>`;
  g += `<line x1="${pad}" y1="${pad}" x2="${pad}" y2="${H - pad}" stroke="#2a2e36"/>`;
  if (zeroY !== null) g += `<line x1="${pad}" y1="${zeroY}" x2="${W - pad}" y2="${zeroY}" stroke="#3a3f48" stroke-dasharray="3"/>`;
  if (zeroX !== null) g += `<line x1="${zeroX}" y1="${pad}" x2="${zeroX}" y2="${H - pad}" stroke="#3a3f48" stroke-dasharray="3"/>`;
  g += `<polyline points="${pts}" fill="none" stroke="${color}" stroke-width="2"/>`;
  g += `<text x="${W / 2}" y="${H - 6}" fill="#9aa0a8" font-size="9" text-anchor="middle">${xlabel}</text>`;
  g += `<text x="6" y="${pad - 6}" fill="#9aa0a8" font-size="9">travel↑ bump</text>`;
  g += `<text x="${W - pad}" y="${pad - 6}" fill="#9aa0a8" font-size="9" text-anchor="end">${ymax.toFixed(1)}</text>`;
  g += `<text x="${W - pad}" y="${H - pad + 10}" fill="#9aa0a8" font-size="9" text-anchor="end">${ymin.toFixed(1)}</text>`;
  g += `</svg>`;
  return g;
}

function drawCharts(k) {
  const t = k.travel_mm;
  el("chart-camber").innerHTML = lineChart(t, k.camber_deg, "#ff5a3c", "travel [mm] vs camber [deg]");
  el("chart-scrub").innerHTML = lineChart(t, k.scrub_mm, "#5b8def", "travel [mm] vs scrub [mm]");
  el("chart-rc").innerHTML = lineChart(t, k.rc_height_mm, "#3cba7a", "travel [mm] vs RC height [mm]");
  if (k.motion_ratio) {
    el("chart-mr").innerHTML = lineChart(t, k.motion_ratio, "#b07cff", "travel [mm] vs MR (d/w)");
  } else {
    el("chart-mr").innerHTML = `<svg viewBox="0 0 260 150"><text x="130" y="75" fill="#6b7280" font-size="11" text-anchor="middle">add rocker hardpoints</text></svg>`;
  }
}

function showReadouts(k) {
  const t = k.travel_mm, c = k.camber_deg;
  const mid = Math.floor(t.length / 2);
  const gain = (c[mid + 1] - c[mid - 1]) / (t[mid + 1] - t[mid - 1]); // deg/mm
  const item = (label, val) => `<span>${label}</span> <b>${val}</b>`;
  const parts = [
    item("camber gain", `${(gain * 25).toFixed(3)}°/25mm`),
    item("roll center @ ride", `${k.rc_height_mm[mid].toFixed(1)} mm`),
  ];
  if (k.motion_ratio) parts.push(item("motion ratio @ ride", k.motion_ratio[mid].toFixed(3)));
  if (k.kpi_deg) parts.push(item("KPI", `${k.kpi_deg[mid].toFixed(2)}°`));
  if (k.scrub_radius_mm) parts.push(item("scrub radius", `${k.scrub_radius_mm[mid].toFixed(1)} mm`));
  if (k.fvsa_m && k.fvsa_m[mid] != null) parts.push(item("FVSA", `${k.fvsa_m[mid].toFixed(2)} m`));
  el("readouts").innerHTML = parts.join("&nbsp;&nbsp; ");
}

// --- animation loop -------------------------------------------------------- //
let playhead = 0, last = performance.now();
function animate(now) {
  requestAnimationFrame(animate);
  const delta = (now - last) / 1000; last = now;
  if (trajectory && frameCount > 1) {
    playhead = (playhead + delta / dt) % frameCount;
    applyFrame(playhead);
  }
}

// --- wiring + boot --------------------------------------------------------- //
buildInputs();
el("run").addEventListener("click", run);
el("csv").addEventListener("change", (e) => {
  const file = e.target.files[0];
  if (!file) return;
  const reader = new FileReader();
  reader.onload = () => applyCsv(reader.result);
  reader.readAsText(file);
});

requestAnimationFrame(animate);
run(); // run the default geometry on load
