// RPM viewer: replays a run exported by `rpm-sim export` on the real printed parts.
//
// Frames (docs/interfaces.md): world x = horizontal outer axis, z up, gravity (0, 0, -1) g.
// Sample orientation R = Rx(theta_outer) @ Rz(theta_inner), so the scene graph is
//   base (fixed) -> innerPivot (rotation.x = outer) -> platformPivot (rotation.z = inner).
// STL files are in mm with +Y up and both axes through (0, 0, 2.5) mm; one fixed transform
// (+90 deg about x, then shift the axis crossing to the origin) brings them into the world.

import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { STLLoader } from "three/addons/loaders/STLLoader.js";
import { RoomEnvironment } from "three/addons/environments/RoomEnvironment.js";

const STL_DIR = "../../../rpm_printables/";
const DATA_DIR = "data/";
const DEFAULT_RUNS = ["clinostat.json", "random_walk.json", "random_setpoints.json"];
const FLOOR_Z = -91.19; // underside of the stands, STL y = -91.19 mm (PLAN.md 2.1)
const GEAR_CENTRES = 37.5; // module 1, 60 + 15 teeth
const GEAR_RATIO = 4;
const OUTER_GEAR_X = -89; // outside the left post; not fixed by the STLs (PLAN.md 2.3)
const INNER_GEAR_Z = 64; // on the platform shaft above the inner frame (STL y 60..67)
const TRAIL_TICKS = 300; // 30 s of trail
const LIMIT_G = 0.01;

const COLORS = {
  outer: 0x59616d,
  inner: 0xe07a2f,
  platform: 0x22a699,
  gear: 0xd8d2c4,
  pinion: 0xb7ae9c,
  cap: 0x8c939e,
  axisX: 0xe5484d,
  axisY: 0x46a758,
  axisZ: 0x3e63dd,
};

const $ = (id) => document.getElementById(id);
const css = (name) => getComputedStyle(document.documentElement).getPropertyValue(name).trim();

// ---------------------------------------------------------------------------------------------
// Main scene: the machine

const stage = $("stage");
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
stage.prepend(renderer.domElement);

const scene = new THREE.Scene();
const pmrem = new THREE.PMREMGenerator(renderer);
scene.environment = pmrem.fromScene(new RoomEnvironment(renderer), 0.04).texture;

const camera = new THREE.PerspectiveCamera(35, 1, 5, 5000);
camera.up.set(0, 0, 1); // before OrbitControls, which orbits around camera.up
camera.position.set(330, -420, 210);
const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0, 0, -5);
controls.enableDamping = true;
controls.minDistance = 150;
controls.maxDistance = 1500;

scene.add(new THREE.HemisphereLight(0xffffff, 0x444444, 0.6));
const sun = new THREE.DirectionalLight(0xffffff, 2.2);
sun.position.set(220, -260, 420);
sun.castShadow = true;
sun.shadow.mapSize.set(2048, 2048);
Object.assign(sun.shadow.camera, { left: -260, right: 260, top: 260, bottom: -260, near: 50, far: 1200 });
sun.shadow.bias = -0.0005;
scene.add(sun);
const rim = new THREE.DirectionalLight(0x9fc4ff, 0.8);
rim.position.set(-300, 250, 150);
scene.add(rim);

const floor = new THREE.Mesh(
  new THREE.PlaneGeometry(2000, 2000),
  new THREE.ShadowMaterial({ opacity: 0.25 }),
);
floor.position.z = FLOOR_Z;
floor.receiveShadow = true;
scene.add(floor);
let grid = null;

const base = new THREE.Group(); // fixed: outer frame, stands, outer pinion
const innerPivot = new THREE.Group(); // turns about world x by theta_outer
const platformPivot = new THREE.Group(); // turns about the inner frame's z by theta_inner
scene.add(base);
base.add(innerPivot);
innerPivot.add(platformPivot);

function material(color, opts = {}) {
  return new THREE.MeshStandardMaterial({ color, roughness: 0.55, metalness: 0.05, ...opts });
}

// STL (mm, +Y up, axes through (0, 0, 2.5)) -> world (z up, axes through the origin).
function placeStl(mesh) {
  mesh.rotation.x = Math.PI / 2;
  mesh.position.set(0, 2.5, 0);
  return mesh;
}

// Simple stand-in shapes in STL coordinates, used when an STL cannot be loaded.
function ringGeometry(x0, x1, y0, y1, wall, z0, z1) {
  const outer = new THREE.Shape();
  outer.moveTo(x0, y0); outer.lineTo(x1, y0); outer.lineTo(x1, y1); outer.lineTo(x0, y1);
  const hole = new THREE.Path();
  hole.moveTo(x0 + wall, y0 + wall); hole.lineTo(x0 + wall, y1 - wall);
  hole.lineTo(x1 - wall, y1 - wall); hole.lineTo(x1 - wall, y0 + wall);
  outer.holes.push(hole);
  const geo = new THREE.ExtrudeGeometry(outer, { depth: z1 - z0, bevelEnabled: false });
  geo.translate(0, 0, z0);
  return geo;
}
function boxGeometry(x0, x1, y0, y1, z0, z1) {
  const geo = new THREE.BoxGeometry(x1 - x0, y1 - y0, z1 - z0);
  geo.translate((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2);
  return geo;
}
function gearGeometry(radius, x0, x1) {
  // Cylinder along local x, like the gear STLs.
  const geo = new THREE.CylinderGeometry(radius, radius, x1 - x0, 48);
  geo.rotateZ(Math.PI / 2);
  geo.translate((x0 + x1) / 2, 0, 0);
  return geo;
}

const PARTS = [
  // name, file, parent, colour, placement, fallback geometry (bounding boxes measured from the STLs)
  {
    file: "01_outer_frame.stl", parent: base, color: COLORS.outer, place: placeStl,
    fallback: () => ringGeometry(-84.61, 84.61, -89.94, 89.94, 9, -3, 21.23),
  },
  {
    file: "02_inner_frame.stl", parent: innerPivot, color: COLORS.inner, place: placeStl,
    fallback: () => ringGeometry(-75.87, 75.87, -60, 60, 8, -3, 17.9),
  },
  {
    file: "03_experiment_platform.stl", parent: platformPivot, color: COLORS.platform, place: placeStl,
    fallback: () => boxGeometry(-31, 31, -63.5, 67, -1, 6),
  },
  {
    // Large gear on the inner frame, on the outer axis (-x side).
    file: "04_spur_gear_large_outer.stl", parent: innerPivot, color: COLORS.gear,
    place: (m) => m.position.set(OUTER_GEAR_X, 0, 0),
    fallback: () => gearGeometry(31.5, -2.5, 3),
  },
  {
    // Large gear on the platform shaft; its local x becomes the inner axis (world z).
    file: "05_spur_gear_large_inner.stl", parent: platformPivot, color: COLORS.gear,
    place: (m) => { m.rotation.y = -Math.PI / 2; m.position.set(0, 0, INNER_GEAR_Z); },
    fallback: () => gearGeometry(31.5, -4, 3),
  },
  {
    // End cap on the lower end of the platform shaft.
    file: "07_end_cap.stl", parent: platformPivot, color: COLORS.cap,
    place: (m) => { m.rotation.x = Math.PI; m.position.set(0, 0, -63.5); },
    fallback: () => new THREE.CylinderGeometry(5, 5, 1.5, 24).rotateX(Math.PI / 2).translate(0, 0, 0.75),
  },
];

// Pinions spin with the servos: -4 x the axis angle, about an axis parallel to their gear's.
const outerPinion = new THREE.Group();
outerPinion.position.set(OUTER_GEAR_X - 2, 0, GEAR_CENTRES); // servo holes are above the axis
base.add(outerPinion);
const innerPinionAxis = new THREE.Group();
innerPinionAxis.position.set(GEAR_CENTRES, 0, INNER_GEAR_Z - 3); // servo holes at x 29..57
innerPivot.add(innerPinionAxis);
PARTS.push(
  {
    file: "06_spur_gear_small.stl", parent: outerPinion, color: COLORS.pinion,
    place: () => {}, fallback: () => gearGeometry(8.6, 0, 5),
  },
  {
    file: "06_spur_gear_small.stl", parent: innerPinionAxis, color: COLORS.pinion,
    place: (m) => { m.rotation.y = -Math.PI / 2; }, fallback: () => gearGeometry(8.6, 0, 5),
  },
);

const stlCache = new Map();
const loader = new STLLoader();
function loadStl(file) {
  if (!stlCache.has(file)) stlCache.set(file, loader.loadAsync(STL_DIR + file));
  return stlCache.get(file);
}

async function buildMachine() {
  const failed = [];
  await Promise.all(PARTS.map(async (part) => {
    let geometry;
    try {
      geometry = await loadStl(part.file);
    } catch (err) {
      console.warn(`STL ${part.file} not loaded, using a simple shape`, err);
      failed.push(part.file);
      geometry = part.fallback();
    }
    const mesh = new THREE.Mesh(geometry, material(part.color));
    mesh.castShadow = mesh.receiveShadow = true;
    part.place(mesh);
    part.parent.add(mesh);
  }));
  // Optional stands: STL x = 0..3 must move to the posts, the left one mirrored (PLAN.md 2.4).
  try {
    const geometry = await loadStl("optional_stand/Stand.stl");
    for (const side of [1, -1]) {
      const mesh = placeStl(new THREE.Mesh(geometry, material(COLORS.outer, { side: THREE.DoubleSide })));
      mesh.position.x = side * 84.61;
      mesh.scale.x = side;
      mesh.castShadow = mesh.receiveShadow = true;
      base.add(mesh);
    }
  } catch {
    failed.push("optional_stand/Stand.stl");
  }
  const unique = [...new Set(failed)];
  $("status").textContent = unique.length
    ? `Could not load ${unique.join(", ")}: simple shapes shown instead. Serve the repo root (rpm-sim viewer).`
    : "Drag to orbit, scroll to zoom. Space: play/pause, arrow keys: step.";
}

// Sample frame axes on the platform, drawn on top so they stay visible inside the frame.
function overlayArrow(dir, length, color) {
  const arrow = new THREE.ArrowHelper(dir, new THREE.Vector3(), length, color, 9, 5);
  arrow.traverse((o) => {
    if (o.material) { o.material.depthTest = false; o.material.transparent = true; }
    o.renderOrder = 10;
  });
  return arrow;
}

function textSprite(text, color, height) {
  const canvas = document.createElement("canvas");
  const ctx = canvas.getContext("2d");
  const px = 64;
  ctx.font = `600 ${px}px ui-sans-serif, system-ui, sans-serif`;
  canvas.width = Math.ceil(ctx.measureText(text).width) + 16;
  canvas.height = px + 16;
  ctx.font = `600 ${px}px ui-sans-serif, system-ui, sans-serif`;
  ctx.fillStyle = color;
  ctx.textBaseline = "middle";
  ctx.fillText(text, 8, canvas.height / 2);
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: texture, depthTest: false, transparent: true }));
  sprite.scale.set((height * canvas.width) / canvas.height, height, 1);
  sprite.renderOrder = 11;
  sprite.userData.setText = (t, c = color) => {
    ctx.font = `600 ${px}px ui-sans-serif, system-ui, sans-serif`;
    const w = Math.ceil(ctx.measureText(t).width) + 16;
    if (w !== canvas.width) {
      canvas.width = w;
      sprite.scale.set((height * w) / canvas.height, height, 1);
    }
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.font = `600 ${px}px ui-sans-serif, system-ui, sans-serif`;
    ctx.fillStyle = c;
    ctx.textBaseline = "middle";
    ctx.fillText(t, 8, canvas.height / 2);
    texture.dispose();
    sprite.material.map = new THREE.CanvasTexture(canvas);
    sprite.material.map.colorSpace = THREE.SRGBColorSpace;
  };
  return sprite;
}

const sampleAxes = new THREE.Group();
for (const [dir, color, label] of [
  [new THREE.Vector3(1, 0, 0), COLORS.axisX, "x"],
  [new THREE.Vector3(0, 1, 0), COLORS.axisY, "y"],
  [new THREE.Vector3(0, 0, 1), COLORS.axisZ, "z"],
]) {
  sampleAxes.add(overlayArrow(dir, 48, color));
  const s = textSprite(label, `#${color.toString(16).padStart(6, "0")}`, 12);
  s.position.copy(dir).multiplyScalar(56);
  sampleAxes.add(s);
}
platformPivot.add(sampleAxes);

const worldGravity = new THREE.Group();
worldGravity.position.set(150, 0, 80);
scene.add(worldGravity);
let gravityArrow = null;
let gravityLabel = null;

// ---------------------------------------------------------------------------------------------
// Sphere view: gravity direction in the sample frame

const sphereCanvas = $("sphere");
const sRenderer = new THREE.WebGLRenderer({ canvas: sphereCanvas, antialias: true, alpha: true });
sRenderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
const sScene = new THREE.Scene();
const sCamera = new THREE.PerspectiveCamera(32, 1, 0.1, 50);
sCamera.up.set(0, 0, 1);
sCamera.position.set(3.1, -3.6, 2.2);
const sControls = new OrbitControls(sCamera, sphereCanvas);
sControls.enableDamping = true;
sControls.enablePan = false;
sControls.minDistance = 3;
sControls.maxDistance = 9;

const sphereLines = new THREE.Group();
sScene.add(sphereLines);
const sphereShell = new THREE.Mesh(
  new THREE.SphereGeometry(1, 48, 32),
  new THREE.MeshBasicMaterial({ transparent: true, opacity: 0.07, depthWrite: false }),
);
sScene.add(sphereShell);

function circle(radius, z, segments = 96) {
  const pts = [];
  for (let k = 0; k <= segments; k++) {
    const a = (k / segments) * Math.PI * 2;
    pts.push(new THREE.Vector3(radius * Math.cos(a), radius * Math.sin(a), z));
  }
  return new THREE.BufferGeometry().setFromPoints(pts);
}
function buildSphereLines(color) {
  sphereLines.clear();
  const thin = new THREE.LineBasicMaterial({ color, transparent: true, opacity: 0.45 });
  const strong = new THREE.LineBasicMaterial({ color, transparent: true, opacity: 0.9 });
  for (let lat = -60; lat <= 60; lat += 30) {
    const r = Math.cos((lat * Math.PI) / 180);
    const z = Math.sin((lat * Math.PI) / 180);
    sphereLines.add(new THREE.Line(circle(r, z), lat === 0 ? strong : thin));
  }
  for (let lon = 0; lon < 180; lon += 30) {
    const line = new THREE.Line(circle(1, 0), thin);
    line.rotation.x = Math.PI / 2; // meridian through the x axis, then turned about z
    const holder = new THREE.Group();
    holder.add(line);
    holder.rotation.z = (lon * Math.PI) / 180;
    sphereLines.add(holder);
  }
}

for (const [dir, color, label] of [
  [new THREE.Vector3(1, 0, 0), COLORS.axisX, "x"],
  [new THREE.Vector3(0, 1, 0), COLORS.axisY, "y"],
  [new THREE.Vector3(0, 0, 1), COLORS.axisZ, "z"],
]) {
  sScene.add(new THREE.ArrowHelper(dir, new THREE.Vector3(), 1.3, color, 0.09, 0.05));
  const s = textSprite(label, `#${color.toString(16).padStart(6, "0")}`, 0.16);
  s.position.copy(dir).multiplyScalar(1.42);
  sScene.add(s);
}

const gravityColor = new THREE.Color();
const dot = new THREE.Mesh(new THREE.SphereGeometry(0.05, 24, 16), new THREE.MeshBasicMaterial());
sScene.add(dot);
const rayGeometry = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3(0, 0, -1)]);
const ray = new THREE.Line(rayGeometry, new THREE.LineBasicMaterial({ transparent: true, opacity: 0.6 }));
sScene.add(ray);

const trailPositions = new Float32Array(TRAIL_TICKS * 3);
const trailColors = new Float32Array(TRAIL_TICKS * 4);
const trailGeometry = new THREE.BufferGeometry();
trailGeometry.setAttribute("position", new THREE.BufferAttribute(trailPositions, 3).setUsage(THREE.DynamicDrawUsage));
trailGeometry.setAttribute("color", new THREE.BufferAttribute(trailColors, 4).setUsage(THREE.DynamicDrawUsage));
const trail = new THREE.Line(trailGeometry, new THREE.LineBasicMaterial({ vertexColors: true, transparent: true, depthWrite: false }));
trail.frustumCulled = false;
sScene.add(trail);

let cloud = null; // every direction visited so far (coverage)
const meanArrow = new THREE.ArrowHelper(new THREE.Vector3(0, 0, -1), new THREE.Vector3(), 0.5, 0xffffff, 0.12, 0.07);
sScene.add(meanArrow);
const meanLabel = textSprite("mean", "#ffffff", 0.17);
sScene.add(meanLabel);
const targetArrow = new THREE.ArrowHelper(new THREE.Vector3(0, 0, -1), new THREE.Vector3(), 0.5, 0xffffff, 0.12, 0.07);
targetArrow.visible = false;
sScene.add(targetArrow);
const targetLabel = textSprite("target", "#ffffff", 0.17);
targetLabel.visible = false;
sScene.add(targetLabel);

// ---------------------------------------------------------------------------------------------
// Residual chart (log scale)

const chart = $("chart");
const chartCtx = chart.getContext("2d");
const CHART_PAD = { left: 44, right: 16, top: 10, bottom: 22 };
let chartBackdrop = null; // full run, faint, redrawn on resize/theme/data change

function chartScale() {
  const w = chart.clientWidth;
  const h = chart.clientHeight;
  const { lo, hi } = run.logRange;
  const x = (t) => CHART_PAD.left + (t / run.duration) * (w - CHART_PAD.left - CHART_PAD.right);
  const y = (v) => {
    const l = Math.log10(Math.max(v, 10 ** lo));
    return CHART_PAD.top + ((hi - l) / (hi - lo)) * (h - CHART_PAD.top - CHART_PAD.bottom);
  };
  return { w, h, x, y };
}

function drawChartBackdrop() {
  if (!run) return;
  const dpr = Math.min(window.devicePixelRatio, 2);
  const { w, h, x, y } = chartScale();
  chart.width = Math.round(w * dpr);
  chart.height = Math.round(h * dpr);
  chartBackdrop = document.createElement("canvas");
  chartBackdrop.width = chart.width;
  chartBackdrop.height = chart.height;
  const ctx = chartBackdrop.getContext("2d");
  ctx.scale(dpr, dpr);
  ctx.font = "11px ui-sans-serif, system-ui, sans-serif";
  ctx.textBaseline = "middle";
  ctx.textAlign = "right";
  for (let e = run.logRange.lo; e <= run.logRange.hi; e++) {
    const yy = y(10 ** e);
    ctx.strokeStyle = css("--border");
    ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(CHART_PAD.left, yy); ctx.lineTo(w - CHART_PAD.right, yy); ctx.stroke();
    ctx.fillStyle = css("--muted");
    ctx.fillText(e === 0 ? "1 g" : `1e${e}`, CHART_PAD.left - 6, yy);
  }
  ctx.textAlign = "center";
  ctx.textBaseline = "top";
  const step = niceStep(run.duration / 5);
  for (let t = 0; t <= run.duration + 1e-9; t += step) {
    const last = t + step > run.duration + 1e-9;
    ctx.textAlign = last ? "right" : "center";
    ctx.fillText(formatClock(t), last ? x(t) + 4 : x(t), h - CHART_PAD.bottom + 6);
  }
  // Acceptance limit.
  ctx.strokeStyle = css("--limit");
  ctx.setLineDash([5, 4]);
  ctx.beginPath(); ctx.moveTo(CHART_PAD.left, y(LIMIT_G)); ctx.lineTo(w - CHART_PAD.right, y(LIMIT_G)); ctx.stroke();
  ctx.setLineDash([]);
  // Whole run, faint.
  ctx.globalAlpha = 0.25;
  tracePath(ctx, run.n - 1, x, y);
  ctx.strokeStyle = css("--mean");
  ctx.lineWidth = 1.5;
  ctx.stroke();
  ctx.globalAlpha = 1;
}

function tracePath(ctx, last, x, y) {
  ctx.beginPath();
  for (let k = 0; k <= last; k++) {
    const px = x(run.t[k]);
    const py = y(run.deviation[k]);
    if (k === 0) ctx.moveTo(px, py); else ctx.lineTo(px, py);
  }
}

function drawChart(idx, simT) {
  if (!run || !chartBackdrop) return;
  const dpr = chart.width / chart.clientWidth;
  const { h, x, y } = chartScale();
  chartCtx.setTransform(1, 0, 0, 1, 0, 0);
  chartCtx.clearRect(0, 0, chart.width, chart.height);
  chartCtx.drawImage(chartBackdrop, 0, 0);
  chartCtx.setTransform(dpr, 0, 0, dpr, 0, 0);
  if (idx >= 0) {
    tracePath(chartCtx, idx, x, y);
    chartCtx.strokeStyle = css("--mean");
    chartCtx.lineWidth = 2;
    chartCtx.stroke();
    chartCtx.fillStyle = css("--mean");
    chartCtx.beginPath();
    chartCtx.arc(x(run.t[idx]), y(run.deviation[idx]), 3.5, 0, Math.PI * 2);
    chartCtx.fill();
  }
  chartCtx.strokeStyle = css("--muted");
  chartCtx.lineWidth = 1;
  chartCtx.beginPath();
  chartCtx.moveTo(x(simT), CHART_PAD.top);
  chartCtx.lineTo(x(simT), h - CHART_PAD.bottom);
  chartCtx.stroke();
}

chart.addEventListener("pointerdown", (ev) => {
  if (!run) return;
  const rect = chart.getBoundingClientRect();
  const frac = (ev.clientX - rect.left - CHART_PAD.left) / (rect.width - CHART_PAD.left - CHART_PAD.right);
  seek(Math.min(Math.max(frac, 0), 1) * run.duration);
});

function niceStep(raw) {
  const p = 10 ** Math.floor(Math.log10(raw));
  for (const m of [1, 2, 5, 10]) if (m * p >= raw) return m * p;
  return 10 * p;
}

// ---------------------------------------------------------------------------------------------
// Run data and playback

let run = null;
let simT = 0;
let playing = false;
let lastFrame = performance.now();

function norm3(a, b, c) { return Math.sqrt(a * a + b * b + c * c); }

function prepare(doc) {
  const n = doc.t.length;
  const target = (doc.target_g || [0, 0, 0]).map(Number);
  const hasTarget = norm3(...target) > 1e-9;
  const deviation = new Float64Array(n);
  let minPos = Infinity;
  let max = 0;
  for (let k = 0; k < n; k++) {
    const m = doc.mean_g[k];
    deviation[k] = norm3(m[0] - target[0], m[1] - target[1], m[2] - target[2]);
    if (deviation[k] > 0) minPos = Math.min(minPos, deviation[k]);
    max = Math.max(max, deviation[k]);
  }
  // Down to 1e-4 g at most: the running mean swings through zero, deeper dips are just noise.
  const lo = Math.max(-4, Math.min(-3, Math.floor(Math.log10(minPos))));
  const hi = Math.max(0, Math.ceil(Math.log10(max || 1) - 0.01)); // 1.00001 g is rounding, not 10 g
  const cloudPositions = new Float32Array(n * 3);
  doc.gravity.forEach((g, k) => cloudPositions.set([g[0] * 1.003, g[1] * 1.003, g[2] * 1.003], k * 3));
  return {
    ...doc, n, target, hasTarget, deviation, logRange: { lo, hi }, cloudPositions,
    dt: doc.dt || 0.1, duration: doc.t[n - 1],
  };
}

async function loadRun(file) {
  $("message").style.display = "none";
  let doc;
  try {
    const res = await fetch(DATA_DIR + file);
    if (!res.ok) throw new Error(`${res.status} ${res.statusText}`);
    doc = await res.json();
  } catch (err) {
    showMessage(`Could not load <code>data/${file}</code> (${err.message}).<br>
      Create it with <code>cd simulation/jannik &amp;&amp; uv run rpm-sim viewer</code>.`);
    return;
  }
  run = prepare(doc);
  location.hash = file.replace(/\.json$/, "");

  if (cloud) { sScene.remove(cloud); cloud.geometry.dispose(); }
  const cloudGeometry = new THREE.BufferGeometry();
  cloudGeometry.setAttribute("position", new THREE.BufferAttribute(run.cloudPositions, 3));
  cloud = new THREE.Points(cloudGeometry, new THREE.PointsMaterial({
    size: 2, sizeAttenuation: false, transparent: true, opacity: 0.22, depthWrite: false,
  }));
  cloud.frustumCulled = false;
  sScene.add(cloud);

  $("slider").max = run.duration;
  $("slider").step = run.dt;
  $("target-caption").hidden = !run.hasTarget;
  $("res-title").textContent = run.hasTarget ? "Deviation from target |mean g - target|" : "Residual gravity |mean g|";
  $("chart-caption").textContent = run.hasTarget ? "|mean g - target|" : "|mean g|";
  renderParams();
  applyTheme();
  seek(0);
  setPlaying(true);
}

function renderParams() {
  const p = run.params || {};
  const pct = (v) => `${(v * 100).toFixed(0)} %`;
  const short = (v) => v.toFixed(3).replace(/\.?0+$/, "") || "0";
  const rows = [
    ["strategy", `${run.strategy}, seed ${p.seed ?? "-"}, ${formatClock(run.duration)} min`],
    ["max speed", p.max_speed != null ? `${p.max_speed} deg/s` : null],
    ["servo error / noise", p.speed_error != null ? `${pct(p.speed_error)} / ${pct(p.noise ?? 0)}` : null],
    ["target", run.hasTarget ? `(${run.target.map(short).join(", ")}) g` : "0 g (microgravity)"],
    ["final", formatG(run.deviation[run.n - 1])],
    ["coverage", p.coverage_range ? `${p.coverage_range[0].toFixed(2)} .. ${p.coverage_range[1].toFixed(2)}` : null],
  ].filter(([, v]) => v != null);
  $("params").innerHTML = rows.map(([k, v]) => `<dt>${k}</dt><dd class="num">${v}</dd>`).join("");
}

function showMessage(html) {
  $("message").innerHTML = `<div>${html}</div>`;
  $("message").style.display = "grid";
}

function formatClock(t) {
  const m = Math.floor(t / 60);
  const s = Math.floor(t % 60);
  return `${m}:${String(s).padStart(2, "0")}`;
}

function formatG(v) {
  if (v >= 0.1) return `${v.toFixed(3)} g`;
  if (v >= 0.001) return `${v.toFixed(4)} g`;
  return `${v.toExponential(1)} g`;
}

function wrapDelta(d) {
  return d - Math.round(d / (2 * Math.PI)) * 2 * Math.PI;
}

// Angles at simulated time t, interpolated between ticks (the run starts at 0/0 at t = 0).
function anglesAt(t) {
  const f = t / run.dt - 1; // tick k holds the state at t = (k + 1) * dt
  const k = Math.floor(f);
  const frac = f - k;
  const at = (j) => (j < 0 ? [0, 0] : [run.theta_outer[j], run.theta_inner[j]]);
  const a = at(Math.min(k, run.n - 1));
  const b = at(Math.min(k + 1, run.n - 1));
  return [a[0] + frac * wrapDelta(b[0] - a[0]), a[1] + frac * wrapDelta(b[1] - a[1])];
}

function seek(t) {
  simT = Math.min(Math.max(t, 0), run ? run.duration : 0);
  if (run && simT >= run.duration) setPlaying(false);
}

function setPlaying(on) {
  playing = on && run != null;
  if (playing && run && simT >= run.duration) simT = 0;
  $("play").innerHTML = playing ? "&#10074;&#10074;" : "&#9654;";
  $("play").setAttribute("aria-label", playing ? "Pause" : "Play");
}

function update() {
  if (!run) return;
  const [o, i] = anglesAt(simT);
  innerPivot.rotation.x = o;
  platformPivot.rotation.z = i;
  outerPinion.rotation.x = -GEAR_RATIO * o;
  innerPinionAxis.rotation.z = -GEAR_RATIO * i;

  // Gravity in the sample frame, from the interpolated angles (same formula as rpm.core).
  const g = new THREE.Vector3(-Math.sin(i) * Math.sin(o), -Math.cos(i) * Math.sin(o), -Math.cos(o));
  dot.position.copy(g);
  rayGeometry.attributes.position.setXYZ(1, g.x, g.y, g.z);
  rayGeometry.attributes.position.needsUpdate = true;

  const idx = Math.min(run.n - 1, Math.floor(simT / run.dt + 1e-6) - 1);
  updateTrail(idx, g);
  if (cloud) cloud.geometry.setDrawRange(0, Math.max(0, idx + 1));
  updateMean(idx);

  $("hud-t").textContent = `${simT.toFixed(1)} s`;
  $("hud-o").textContent = `${((o * 180) / Math.PI).toFixed(0)}°`;
  $("hud-i").textContent = `${((i * 180) / Math.PI).toFixed(0)}°`;
  $("clock").textContent = `${formatClock(simT)} / ${formatClock(run.duration)}`;
  if (document.activeElement !== $("slider")) $("slider").value = simT;
  drawChart(idx, simT);
}

function updateTrail(idx, g) {
  const count = Math.min(TRAIL_TICKS - 1, idx + 1);
  const start = idx - count + 1;
  const c = gravityColor;
  for (let k = 0; k < count; k++) {
    const v = run.gravity[start + k];
    trailPositions.set([v[0] * 1.006, v[1] * 1.006, v[2] * 1.006], k * 3);
    trailColors.set([c.r, c.g, c.b, ((k + 1) / (count + 1)) ** 1.5], k * 4);
  }
  // Last point is the current (interpolated) direction, so the trail ends at the dot.
  trailPositions.set([g.x * 1.006, g.y * 1.006, g.z * 1.006], count * 3);
  trailColors.set([c.r, c.g, c.b, 1], count * 4);
  trailGeometry.setDrawRange(0, count + 1);
  trailGeometry.attributes.position.needsUpdate = true;
  trailGeometry.attributes.color.needsUpdate = true;
}

const tmpMean = new THREE.Vector3();
const tmpTarget = new THREE.Vector3();
function updateMean(idx) {
  const dev = idx >= 0 ? run.deviation[idx] : NaN;
  if (idx < 0) {
    meanArrow.visible = meanLabel.visible = false;
    $("res").textContent = "-";
    $("res-badge").textContent = "";
    $("res-sub").innerHTML = "&nbsp;";
    return;
  }
  tmpMean.fromArray(run.mean_g[idx]);
  tmpTarget.fromArray(run.target);
  const len = tmpMean.length();
  // Linear scale in decades so a 0.002 g mean is still visible; both arrows share it.
  const ref = Math.max(len, run.hasTarget ? tmpTarget.length() : 0, 1e-6);
  const scale = Math.min(1000, 10 ** Math.max(0, Math.floor(-Math.log10(ref))));
  meanArrow.visible = meanLabel.visible = len > 1e-9;
  if (meanArrow.visible) {
    const l = Math.max(len * scale, 0.05);
    meanArrow.setDirection(tmpMean.clone().normalize());
    meanArrow.setLength(l, Math.min(0.12, l * 0.4), Math.min(0.07, l * 0.25));
    meanLabel.position.copy(tmpMean).normalize().multiplyScalar(l + 0.16);
    const text = `mean ${formatG(len)}${scale > 1 ? `  (×${scale})` : ""}`;
    if (meanLabel.userData.text !== text) {
      meanLabel.userData.text = text;
      meanLabel.userData.setText(text, css("--mean"));
    }
  }
  targetArrow.visible = targetLabel.visible = run.hasTarget;
  if (run.hasTarget) {
    const l = tmpTarget.length() * scale;
    targetArrow.setDirection(tmpTarget.clone().normalize());
    targetArrow.setLength(l, Math.min(0.12, l * 0.4), Math.min(0.07, l * 0.25));
    targetLabel.position.copy(tmpTarget).normalize().multiplyScalar(l + 0.16);
    targetLabel.position.x += 0.25;
  }
  $("res").innerHTML = `${formatG(dev).replace(" g", "")} <small>g</small>`;
  const ok = dev < LIMIT_G;
  $("res-badge").className = `badge ${ok ? "ok" : "bad"}`;
  $("res-badge").textContent = ok ? "below 0.01 g" : "above 0.01 g";
  const m = run.mean_g[idx];
  $("res-sub").textContent = `mean g = (${m.map((v) => v.toFixed(4)).join(", ")}) after ${formatClock(run.t[idx])}`;
}

// ---------------------------------------------------------------------------------------------
// Theme, layout, controls

const THEMES = ["auto", "light", "dark"];
let themeIndex = 0;
try {
  themeIndex = Math.max(0, THEMES.indexOf(localStorage.getItem("rpm-viewer-theme") || "auto"));
} catch { /* storage blocked: stay on auto */ }

function applyTheme() {
  const theme = THEMES[themeIndex];
  if (theme === "auto") delete document.documentElement.dataset.theme;
  else document.documentElement.dataset.theme = theme;
  $("theme").textContent = theme;
  const bg = new THREE.Color(css("--scene-bg"));
  scene.background = bg;
  scene.fog = new THREE.Fog(bg, 900, 2200);
  if (grid) { scene.remove(grid); grid.geometry.dispose(); }
  grid = new THREE.GridHelper(800, 32, new THREE.Color(css("--grid")), new THREE.Color(css("--grid-2")));
  grid.rotation.x = Math.PI / 2;
  grid.position.z = FLOOR_Z + 0.2;
  scene.add(grid);

  gravityColor.set(css("--gravity"));
  if (gravityArrow) worldGravity.remove(gravityArrow, gravityLabel);
  gravityArrow = new THREE.ArrowHelper(new THREE.Vector3(0, 0, -1), new THREE.Vector3(), 110, gravityColor, 22, 12);
  gravityLabel = textSprite("g", css("--gravity"), 22);
  gravityLabel.position.set(14, 0, -40);
  worldGravity.add(gravityArrow, gravityLabel);

  buildSphereLines(new THREE.Color(css("--sphere-line")));
  sphereShell.material.color.set(css("--sphere-line"));
  dot.material.color.copy(gravityColor);
  ray.material.color.copy(gravityColor);
  if (cloud) cloud.material.color.copy(gravityColor);
  meanArrow.setColor(new THREE.Color(css("--mean")));
  targetArrow.setColor(new THREE.Color(css("--target")));
  meanLabel.userData.text = null;
  targetLabel.userData.setText("target", css("--target"));
  drawChartBackdrop();
  if (run) update();
}

$("theme").addEventListener("click", () => {
  themeIndex = (themeIndex + 1) % THEMES.length;
  try { localStorage.setItem("rpm-viewer-theme", THEMES[themeIndex]); } catch { /* ignore */ }
  applyTheme();
});
matchMedia("(prefers-color-scheme: dark)").addEventListener("change", () => applyTheme());

function resize() {
  const w = stage.clientWidth;
  const h = stage.clientHeight;
  renderer.setSize(w, h, false);
  camera.aspect = w / Math.max(h, 1);
  camera.fov = camera.aspect < 1.2 ? Math.min(70, (35 * 1.2) / camera.aspect) : 35; // keep it in frame on phones
  camera.updateProjectionMatrix();
  const sw = sphereCanvas.clientWidth;
  const sh = sphereCanvas.clientHeight;
  sRenderer.setSize(sw, sh, false);
  sCamera.aspect = sw / Math.max(sh, 1);
  sCamera.updateProjectionMatrix();
  drawChartBackdrop();
}
new ResizeObserver(resize).observe(document.body);

$("play").addEventListener("click", () => setPlaying(!playing));
$("slider").addEventListener("input", (ev) => seek(Number(ev.target.value)));
$("run").addEventListener("change", (ev) => loadRun(ev.target.value));
let speed = Number($("speed").value);
$("speed").addEventListener("change", (ev) => { speed = Number(ev.target.value); });
window.addEventListener("keydown", (ev) => {
  if (!run || ev.target.tagName === "SELECT") return;
  if (ev.code === "Space") { ev.preventDefault(); setPlaying(!playing); }
  if (ev.code === "ArrowRight") seek(simT + (ev.shiftKey ? 10 : 1));
  if (ev.code === "ArrowLeft") seek(simT - (ev.shiftKey ? 10 : 1));
});

function frame(now) {
  const elapsed = Math.min((now - lastFrame) / 1000, 0.1);
  lastFrame = now;
  if (playing && run) seek(simT + elapsed * speed);
  update();
  controls.update();
  sControls.update();
  renderer.render(scene, camera);
  sRenderer.render(sScene, sCamera);
  requestAnimationFrame(frame);
}

async function listRuns() {
  try {
    const res = await fetch(DATA_DIR + "index.json", { cache: "no-store" });
    if (res.ok) {
      const { runs } = await res.json();
      if (Array.isArray(runs) && runs.length) return runs;
    }
  } catch { /* fall back to the default names */ }
  return DEFAULT_RUNS;
}

async function main() {
  applyTheme();
  resize();
  requestAnimationFrame(frame);
  buildMachine();
  const runs = await listRuns();
  const select = $("run");
  select.innerHTML = runs.map((f) => `<option value="${f}">${f.replace(/\.json$/, "").replaceAll("_", " ")}</option>`).join("");
  const wanted = `${decodeURIComponent(location.hash.slice(1))}.json`;
  select.value = runs.includes(wanted) ? wanted : runs[0];
  await loadRun(select.value);
}

main();
