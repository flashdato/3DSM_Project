// Sensor 3D Modeler — web port of the Python human-model demo.
// Mirrors src/human_model.py and src/animations.py: same joint tree, segment
// dimensions, and animation math, so behaviour matches the local Python version.
// Current demo: right arm only (upper arm + forearm with the two IMU nodes of
// the Phase 3 hardware bring-up). The full-body model is kept underneath.

import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";

// ---------- kinematic tree (mirrors JOINT_TREE in human_model.py) ----------

const JOINT_TREE = {
  pelvis:     { parent: null,         offset: [ 0.00, 0.0,  0.00] },
  neck:       { parent: "pelvis",     offset: [ 0.00, 0.0,  0.55] },
  head_base:  { parent: "neck",       offset: [ 0.00, 0.0,  0.06] },
  l_shoulder: { parent: "neck",       offset: [-0.19, 0.0, -0.01] },
  r_shoulder: { parent: "neck",       offset: [ 0.19, 0.0, -0.01] },
  l_elbow:    { parent: "l_shoulder", offset: [ 0.00, 0.0, -0.30] },
  r_elbow:    { parent: "r_shoulder", offset: [ 0.00, 0.0, -0.30] },
  l_hip:      { parent: "pelvis",     offset: [-0.09, 0.0, -0.02] },
  r_hip:      { parent: "pelvis",     offset: [ 0.09, 0.0, -0.02] },
  l_knee:     { parent: "l_hip",      offset: [ 0.00, 0.0, -0.42] },
  r_knee:     { parent: "r_hip",      offset: [ 0.00, 0.0, -0.42] },
};

const JOINT_ORDER = [
  "pelvis", "neck", "head_base",
  "l_shoulder", "r_shoulder", "l_elbow", "r_elbow",
  "l_hip", "r_hip", "l_knee", "r_knee",
];

// dir: "up" (+Z from joint) or "down" (-Z from joint)
// inset gap keeps the joint hinge visible instead of two boxes intersecting.
const SEGMENTS = {
  head:        { joint: "head_base",  dir: "up",   inset: 0.00,  L: 0.22,  W: 0.16,  D: 0.19,  color: 0xf2c8a8 },
  neck_seg:    { joint: "neck",       dir: "up",   inset: 0.005, L: 0.05,  W: 0.09,  D: 0.10,  color: 0xe8bfa0 },
  torso:       { joint: "pelvis",     dir: "up",   inset: 0.03,  L: 0.52,  W: 0.28,  D: 0.18,  color: 0x4a7fc7 },
  l_upper_arm: { joint: "l_shoulder", dir: "down", inset: 0.015, L: 0.275, W: 0.085, D: 0.085, color: 0xe8834a },
  r_upper_arm: { joint: "r_shoulder", dir: "down", inset: 0.015, L: 0.275, W: 0.085, D: 0.085, color: 0xe8834a },
  l_forearm:   { joint: "l_elbow",    dir: "down", inset: 0.02,  L: 0.26,  W: 0.07,  D: 0.07,  color: 0xc9612a },
  r_forearm:   { joint: "r_elbow",    dir: "down", inset: 0.02,  L: 0.26,  W: 0.07,  D: 0.07,  color: 0xc9612a },
  l_thigh:     { joint: "l_hip",      dir: "down", inset: 0.015, L: 0.395, W: 0.13,  D: 0.13,  color: 0x5cb85c },
  r_thigh:     { joint: "r_hip",      dir: "down", inset: 0.015, L: 0.395, W: 0.13,  D: 0.13,  color: 0x5cb85c },
  l_shin:      { joint: "l_knee",     dir: "down", inset: 0.02,  L: 0.40,  W: 0.10,  D: 0.10,  color: 0x3a8a3a },
  r_shin:      { joint: "r_knee",     dir: "down", inset: 0.02,  L: 0.40,  W: 0.10,  D: 0.10,  color: 0x3a8a3a },
};

const REST_PELVIS_HEIGHT = 0.86;

// ---------- right-arm demo (Phase 3 bring-up: two IMU nodes on the right arm) ----------
// The full-body model is still built. Segments outside DEMO_SEGMENTS are hidden,
// or drawn as a faint ghost when "Body outline" is on.
const DEMO_SEGMENTS = new Set(["r_upper_arm", "r_forearm"]);

// Where the two wearable nodes sit (see HARDWARE.md, "Placement for step 1").
// along = fraction of the segment length from the proximal joint.
const SENSOR_NODES = [
  { id: 1, joint: "r_shoulder", seg: "r_upper_arm", along: 0.5 },  // outside of upper arm
  { id: 2, joint: "r_elbow",    seg: "r_forearm",   along: 0.8 },  // back of forearm near wrist
];

// Sensor axes in the segment frame (columns): x toward the hand, y forward, z out of the skin.
// Same as R_MOUNT in src/virtual_imu.py.
const R_MOUNT = new THREE.Matrix4().makeBasis(
  new THREE.Vector3(0, 0, -1), new THREE.Vector3(0, 1, 0), new THREE.Vector3(1, 0, 0));

// ---------- virtual MPU-6050 (mirrors src/virtual_imu.py) ----------
// Ideal signals from central differences on the animation, then the same
// error model: fixed bias per node, white noise, int16 quantization.
const G = 9.80665;
const ACC_LSB = 4096;   // ±8 g
const GYR_LSB = 32.8;   // ±1000 °/s
const TEMP_RAW = Math.round((25 - 36.53) * 340);
function gauss() {
  const u = 1 - Math.random(), v = Math.random();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
}
const SENSOR_ERRORS = [   // placeholders until the v0.3 static recording
  { gb: [0.8, -0.5, 0.3], ab: [0.012, -0.020, 0.008] },
  { gb: [-0.6, 0.4, 1.1], ab: [-0.015, 0.010, 0.018] },
];
const clampI16 = x => Math.max(-32768, Math.min(32767, Math.round(x)));

function simulateSensors(human, fn, t, h = 1e-3) {
  const before = human.sensorPoses(fn(t - h));
  const after = human.sensorPoses(fn(t + h));
  const now = human.sensorPoses(fn(t));
  const gW = new THREE.Vector3(0, 0, -G);
  return now.map((s, i) => {
    // angular rate in the sensor frame: q(t-h)^-1 * q(t+h) ≈ [1, omega*h]
    const dq = before[i].q.clone().invert().multiply(after[i].q);
    const sgn = dq.w < 0 ? -1 : 1;
    const omega = new THREE.Vector3(dq.x, dq.y, dq.z).multiplyScalar(sgn / h);   // rad/s
    const aW = after[i].p.clone().add(before[i].p).addScaledVector(s.p, -2).divideScalar(h * h);
    const f = aW.sub(gW).applyQuaternion(s.q.clone().invert()).divideScalar(G);   // g
    const e = SENSOR_ERRORS[i];
    const acc = [f.x, f.y, f.z].map((v, k) => clampI16((v + e.ab[k] + 0.004 * gauss()) * ACC_LSB));
    const gyr = [omega.x, omega.y, omega.z].map((v, k) =>
      clampI16((THREE.MathUtils.radToDeg(v) + e.gb[k] + 0.05 * gauss()) * GYR_LSB));
    return { acc, gyr, temp: TEMP_RAW + Math.round(3 * gauss()) };
  });
}

// ---------- rotation helpers (right-handed, matching numpy Rx/Ry/Rz) ----------

function Rx(a) { const m = new THREE.Matrix4(); m.makeRotationX(a); return m; }
function Ry(a) { const m = new THREE.Matrix4(); m.makeRotationY(a); return m; }
function Rz(a) { const m = new THREE.Matrix4(); m.makeRotationZ(a); return m; }

// ---------- scene bones ----------

class Human {
  constructor(scene) {
    this.jointGroups = {};      // name -> THREE.Group (transform node)
    this.jointLocalRot = {};    // name -> Matrix4 (per-frame local rotation)
    this.root = new THREE.Group();
    this.root.position.set(0, 0, REST_PELVIS_HEIGHT);
    scene.add(this.root);

    // Build the joint hierarchy.
    for (const name of JOINT_ORDER) {
      const { parent, offset } = JOINT_TREE[name];
      const g = new THREE.Group();
      g.position.set(offset[0], offset[1], offset[2]);
      this.jointGroups[name] = g;
      this.jointLocalRot[name] = new THREE.Matrix4();
      if (parent === null) this.root.add(g);
      else this.jointGroups[parent].add(g);
    }

    // Attach one box per segment, offset along local Z.
    this.ghostMeshes = [];
    for (const [segName, s] of Object.entries(SEGMENTS)) {
      const geom = new THREE.BoxGeometry(s.W, s.D, s.L);
      const inDemo = DEMO_SEGMENTS.has(segName);
      // Simple flat colour so the look matches matplotlib. Non-demo segments
      // become a faint see-through ghost.
      const mat = new THREE.MeshStandardMaterial({
        color: inDemo ? s.color : 0x8b949e, roughness: 0.85, metalness: 0.0,
        transparent: !inDemo, opacity: inDemo ? 1.0 : 0.07, depthWrite: inDemo,
      });
      const mesh = new THREE.Mesh(geom, mat);
      const sign = s.dir === "up" ? 1 : -1;
      // Box is centered on its origin; shift so its near end sits at joint + inset.
      mesh.position.z = sign * (s.inset + s.L / 2);
      mesh.castShadow = inDemo;
      mesh.receiveShadow = inDemo;

      // Faint edges so segments read clearly against the dark background.
      const edges = new THREE.LineSegments(
        new THREE.EdgesGeometry(geom),
        new THREE.LineBasicMaterial({
          color: inDemo ? 0x0d1117 : 0x8b949e, transparent: true, opacity: inDemo ? 0.6 : 0.25,
        }),
      );
      mesh.add(edges);

      if (!inDemo) {
        mesh.visible = false;
        this.ghostMeshes.push(mesh);
      }
      this.jointGroups[s.joint].add(mesh);
    }

    // Hinge markers at the shoulder and elbow.
    for (const [joint, r] of [["r_shoulder", 0.034], ["r_elbow", 0.028]]) {
      const hinge = new THREE.Mesh(
        new THREE.SphereGeometry(r, 24, 16),
        new THREE.MeshStandardMaterial({ color: 0x8b949e, roughness: 0.6 }),
      );
      hinge.castShadow = true;
      this.jointGroups[joint].add(hinge);
    }

    // IMU node boxes strapped to the arm. The node group IS the sensor frame
    // (mounting as in HARDWARE.md: board flat on the outside of the limb,
    // components out, X arrow toward the hand). Axes: x red, y green, z blue.
    this.sensorGroups = [];
    for (const n of SENSOR_NODES) {
      const s = SEGMENTS[n.seg];
      const node = new THREE.Group();
      node.position.set(s.W / 2 + 0.013, 0, -(s.inset + s.L * n.along));
      node.quaternion.setFromRotationMatrix(R_MOUNT);
      const body = new THREE.Mesh(
        new THREE.BoxGeometry(0.052, 0.036, 0.024),
        new THREE.MeshStandardMaterial({ color: 0x2b3138, roughness: 0.5 }),
      );
      body.castShadow = true;
      const led = new THREE.Mesh(
        new THREE.SphereGeometry(0.0055, 12, 8),
        new THREE.MeshBasicMaterial({ color: 0x3fb950 }),
      );
      led.position.set(-0.015, 0, 0.0125);
      const axes = new THREE.AxesHelper(0.085);
      axes.position.z = 0.013;
      node.add(body, led, axes);
      this.jointGroups[n.joint].add(node);
      this.sensorGroups.push(node);
    }
  }

  // World position + orientation of each sensor frame for a given pose.
  sensorPoses(poseDict) {
    this.setPose(poseDict);
    this.root.updateMatrixWorld(true);
    return this.sensorGroups.map(g => {
      const p = new THREE.Vector3(), q = new THREE.Quaternion(), sc = new THREE.Vector3();
      g.matrixWorld.decompose(p, q, sc);
      return { p, q };
    });
  }

  setGhost(on) {
    for (const m of this.ghostMeshes) m.visible = on;
  }

  resetPose() {
    this.root.position.set(0, 0, REST_PELVIS_HEIGHT);
    for (const name of JOINT_ORDER) {
      this.jointLocalRot[name].identity();
      this.jointGroups[name].quaternion.identity();
    }
  }

  setPose(poseDict) {
    this.resetPose();
    const q = new THREE.Quaternion();
    for (const [name, mat] of Object.entries(poseDict)) {
      this.jointLocalRot[name].copy(mat);
      q.setFromRotationMatrix(mat);
      this.jointGroups[name].quaternion.copy(q);
    }
  }

  // Drop the root so the lowest shin-box corner sits at world z = 0.
  groundLock() {
    // Ensure Three.js has propagated our rotations into world matrices.
    this.root.updateMatrixWorld(true);

    let lowest = Infinity;
    for (const shinName of ["l_shin", "r_shin"]) {
      const s = SEGMENTS[shinName];
      const g = this.jointGroups[s.joint];
      const sign = s.dir === "up" ? 1 : -1;
      const hw = s.W / 2, hd = s.D / 2;
      const zNear = sign * s.inset;
      const zFar  = sign * (s.inset + s.L);
      const local = new THREE.Vector3();
      for (const z of [zNear, zFar]) {
        for (const [x, y] of [[-hw,-hd],[hw,-hd],[hw,hd],[-hw,hd]]) {
          local.set(x, y, z);
          const world = local.clone().applyMatrix4(g.matrixWorld);
          if (world.z < lowest) lowest = world.z;
        }
      }
    }
    this.root.position.z -= lowest;
    this.root.updateMatrixWorld(true);
  }
}

// ---------- animations (mirrors animations.py) ----------

function waveArm(t) {
  const pose = {};
  pose.r_shoulder = Ry(THREE.MathUtils.degToRad(-90));
  const osc = THREE.MathUtils.degToRad(35) * Math.sin(2 * Math.PI * 1.5 * t);
  const forearm = new THREE.Matrix4();
  forearm.multiplyMatrices(Rz(osc), Ry(THREE.MathUtils.degToRad(-90)));
  pose.r_elbow = forearm;
  return pose;
}

function squat(t) {
  const period = 3.0;
  const depth = (1.0 - Math.cos((2 * Math.PI * t) / period)) / 2.0;
  const hip = THREE.MathUtils.degToRad(80) * depth;
  const pose = {};
  pose.l_hip = Rx(hip);
  pose.r_hip = Rx(hip);
  pose.l_knee = Rx(-hip);
  pose.r_knee = Rx(-hip);
  return pose;
}

// Smooth 0 -> 1 -> 0 profile, one cycle per `period` seconds.
function cycle01(t, period) {
  return (1.0 - Math.cos((2 * Math.PI * t) / period)) / 2.0;
}

// Elbow flexion 0 -> 135 deg: the first thing the two arm nodes will measure.
function elbowCurl(t) {
  const d = cycle01(t, 3.0);
  return { r_elbow: Rx(THREE.MathUtils.degToRad(135) * d) };
}

// Shoulder flexion: raise the straight arm forward to 150 deg.
function armRaise(t) {
  const d = cycle01(t, 3.0);
  return {
    r_shoulder: Rx(THREE.MathUtils.degToRad(150) * d),
    r_elbow: Rx(THREE.MathUtils.degToRad(10) * d),
  };
}

// Shoulder abduction: raise the arm sideways to 90 deg.
function sideRaise(t) {
  const d = cycle01(t, 3.0);
  return { r_shoulder: Ry(THREE.MathUtils.degToRad(-90) * d) };
}

// Full-body movements (not shown in the right-arm demo, kept for later).
const FULL_BODY_MOVEMENTS = [
  { key: "wave",  label: "Waving right arm", fn: waveArm },
  { key: "squat", label: "Squatting",        fn: squat  },
];

// Right-arm demo movements.
const MOVEMENTS = [
  { key: "curl",  label: "Elbow curl",              fn: elbowCurl },
  { key: "raise", label: "Arm raise (forward)",     fn: armRaise  },
  { key: "side",  label: "Side raise (abduction)",  fn: sideRaise },
  { key: "wave",  label: "Wave",                    fn: waveArm   },
];
const CYCLE_SECONDS = 6.0;

// Rotation angle of a joint's local rotation (degrees), i.e. how far the child
// segment is rotated relative to its parent. For the elbow this is what the two
// IMU nodes give via q_UF = q_WU^-1 * q_WF.
function jointAngleDeg(mat) {
  const e = mat.elements;
  const c = Math.min(1, Math.max(-1, (e[0] + e[5] + e[10] - 1) / 2));
  return THREE.MathUtils.radToDeg(Math.acos(c));
}

// ---------- Three.js scene setup ----------

const canvas = document.getElementById("c");
const stage = document.getElementById("stage");
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.setClearColor(0x0d1117, 1);
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;

const scene = new THREE.Scene();
scene.fog = new THREE.Fog(0x0d1117, 6, 18);

// Z-up world (matches the Python coord convention).
// Framed on the right arm, seen from the front-right.
const camera = new THREE.PerspectiveCamera(35, 1, 0.05, 100);
camera.up.set(0, 0, 1);
camera.position.set(1.75, 2.35, 1.75);

const controls = new OrbitControls(camera, canvas);
controls.target.set(0.2, 0, 1.25);
controls.enableDamping = true;
controls.minDistance = 0.8;
controls.maxDistance = 10;

// Lights.
const hemi = new THREE.HemisphereLight(0xbcd4ff, 0x0a0d12, 0.5);
scene.add(hemi);
const key = new THREE.DirectionalLight(0xffffff, 1.2);
key.position.set(3, 4, 6);
key.castShadow = true;
key.shadow.mapSize.set(2048, 2048);
key.shadow.camera.left = -3; key.shadow.camera.right = 3;
key.shadow.camera.top = 3;   key.shadow.camera.bottom = -3;
key.shadow.camera.near = 0.5; key.shadow.camera.far = 20;
scene.add(key);
const fill = new THREE.DirectionalLight(0x8899aa, 0.35);
fill.position.set(-4, -3, 2);
scene.add(fill);

// Subtle ground disc (shadow catcher, world is Z-up so keep the disc in the XY plane).
const groundGeom = new THREE.CircleGeometry(2.2, 64);
const groundMat = new THREE.ShadowMaterial({ opacity: 0.35 });
const ground = new THREE.Mesh(groundGeom, groundMat);
ground.receiveShadow = true;
scene.add(ground);

// Faint disc under the model so it doesn't float in a void.
const bgGeom = new THREE.CircleGeometry(2.4, 64);
const bgMat = new THREE.MeshBasicMaterial({ color: 0x141a24, transparent: true, opacity: 0.55 });
const bgDisc = new THREE.Mesh(bgGeom, bgMat);
bgDisc.position.z = -0.001;
scene.add(bgDisc);

const human = new Human(scene);

// ---------- UI wiring ----------

let mode = "cycle";
const moveLabel = document.getElementById("move");
const timeLabel = document.getElementById("time");
const elbowLabel = document.getElementById("elbow");
const shoulderLabel = document.getElementById("shoulder");
const btns = document.querySelectorAll("#controls button[data-move]");
btns.forEach(btn => btn.addEventListener("click", () => {
  mode = btn.dataset.move;
  btns.forEach(b => b.classList.toggle("active", b === btn));
}));

const sensorCells = [1, 2].map(n =>
  ["ax", "ay", "az", "gx", "gy", "gz", "acc", "gyr"].map(k => document.getElementById(`n${n}-${k}`)));
const serialLine = document.getElementById("serial");
let lastSensorT = -1;
let sampleSeq = 0;
let lastScopeT = -1;
let latestOut = null;

// ---------- live signal charts (converted from the simulated raw counts) ----------
const SCOPE_SECONDS = 6;
const SCOPE_HZ = 50;
const AXIS_COLORS = ["#3987e5", "#d95926", "#199e70"];   // x, y, z (fixed order)
const INK = "#e6edf3", MUTED = "#8b949e", GRID = "#21262d", ZERO = "#3a414a";
const scopes = [
  { id: "sc-1-acc", node: 0, key: "acc", range: 2.5, ticks: [-2, -1, 0, 1, 2], dec: 2 },
  { id: "sc-1-gyr", node: 0, key: "gyr", range: 400, ticks: [-400, -200, 0, 200, 400], dec: 0 },
  { id: "sc-2-acc", node: 1, key: "acc", range: 2.5, ticks: [-2, -1, 0, 1, 2], dec: 2 },
  { id: "sc-2-gyr", node: 1, key: "gyr", range: 400, ticks: [-400, -200, 0, 200, 400], dec: 0 },
].map(s => ({ ...s, canvas: document.getElementById(s.id), buf: [] }));

function pushScopeSample(t, out) {
  for (const sc of scopes) {
    const s = out[sc.node];
    const v = sc.key === "acc" ? s.acc.map(c => c / ACC_LSB) : s.gyr.map(c => c / GYR_LSB);
    sc.buf.push({ t, v });
    while (sc.buf.length && sc.buf[0].t < t - SCOPE_SECONDS) sc.buf.shift();
  }
}

function drawScopes(t) {
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  for (const sc of scopes) {
    const c = sc.canvas;
    if (!c || !c.getContext) continue;
    const w = c.clientWidth, h = c.clientHeight;
    if (!w || !h) continue;
    if (c.width !== Math.round(w * dpr) || c.height !== Math.round(h * dpr)) {
      c.width = Math.round(w * dpr);
      c.height = Math.round(h * dpr);
    }
    const ctx = c.getContext("2d");
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);
    const padL = 36, padR = 58, padT = 6, padB = 6;
    const pw = w - padL - padR, ph = h - padT - padB;
    const yOf = v => padT + ph / 2 - (Math.max(-sc.range, Math.min(sc.range, v)) / sc.range) * (ph / 2);
    const xOf = tt => padL + ((tt - (t - SCOPE_SECONDS)) / SCOPE_SECONDS) * pw;

    // recessive grid + tick labels
    ctx.font = "10px ui-monospace, SFMono-Regular, Menlo, Consolas, monospace";
    ctx.textAlign = "right";
    ctx.textBaseline = "middle";
    for (const tk of sc.ticks) {
      const y = Math.round(yOf(tk)) + 0.5;
      ctx.strokeStyle = tk === 0 ? ZERO : GRID;
      ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(padL, y); ctx.lineTo(padL + pw, y); ctx.stroke();
      ctx.fillStyle = MUTED;
      ctx.fillText(String(tk), padL - 6, y);
    }

    // x, y, z traces
    ctx.lineWidth = 1.6;
    ctx.lineJoin = "round";
    for (let k = 0; k < 3; k++) {
      ctx.strokeStyle = AXIS_COLORS[k];
      ctx.beginPath();
      sc.buf.forEach((b, i) => {
        const x = xOf(b.t), y = yOf(b.v[k]);
        if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
      });
      ctx.stroke();
    }

    // latest values: colored key dot + value in text ink
    const last = sc.buf[sc.buf.length - 1];
    if (last) {
      ctx.textAlign = "left";
      for (let k = 0; k < 3; k++) {
        const y = padT + 10 + k * 14;
        ctx.fillStyle = AXIS_COLORS[k];
        ctx.beginPath(); ctx.arc(padL + pw + 10, y, 3, 0, 2 * Math.PI); ctx.fill();
        ctx.fillStyle = INK;
        ctx.fillText(last.v[k].toFixed(sc.dec), padL + pw + 17, y);
      }
    }
  }
}

let ghost = false;
const ghostBtn = document.getElementById("ghost");
ghostBtn.addEventListener("click", () => {
  ghost = !ghost;
  human.setGhost(ghost);
  ghostBtn.classList.toggle("active", ghost);
});

// ---------- animation loop ----------

function resize() {
  const w = stage.clientWidth;
  const h = stage.clientHeight;
  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
window.addEventListener("resize", resize);
resize();

const startTs = performance.now();
function tick() {
  const t = (performance.now() - startTs) / 1000;

  let currentMove;
  let localT;
  if (mode === "cycle") {
    const idx = Math.floor(t / CYCLE_SECONDS) % MOVEMENTS.length;
    currentMove = MOVEMENTS[idx];
    localT = t - idx * CYCLE_SECONDS;
  } else {
    currentMove = MOVEMENTS.find(m => m.key === mode) || MOVEMENTS[0];
    localT = t;
  }

  // Simulated sensor output: sampled at SCOPE_HZ for the charts; the number
  // panel refreshes 10x per second so it stays readable.
  if (t - lastScopeT >= 1 / SCOPE_HZ) {
    lastScopeT = t;
    latestOut = simulateSensors(human, currentMove.fn, localT);
    pushScopeSample(t, latestOut);
  }
  if (latestOut && t - lastSensorT >= 0.1) {
    lastSensorT = t;
    sampleSeq += 10;
    const out = latestOut;
    out.forEach((s, i) => {
      const cells = sensorCells[i];
      [...s.acc, ...s.gyr].forEach((v, k) => { cells[k].textContent = v; });
      cells[6].textContent = (s.acc.map(v => (v / ACC_LSB).toFixed(2)).join(" ")) + " g";
      cells[7].textContent = (s.gyr.map(v => (v / GYR_LSB).toFixed(0)).join(" ")) + " °/s";
    });
    serialLine.textContent =
      `D,1,2,${sampleSeq % 65536},${5000000 + Math.round(t * 1e6)},${out[0].acc.join(",")},` +
      `${out[0].gyr.join(",")},0,0,0,${out[0].temp},1,3900,-45`;
  }

  drawScopes(t);

  human.setPose(currentMove.fn(localT));
  human.groundLock();

  moveLabel.textContent = currentMove.label;
  timeLabel.textContent = `t = ${t.toFixed(2)} s`;
  elbowLabel.textContent = `${jointAngleDeg(human.jointLocalRot.r_elbow).toFixed(0)}°`;
  shoulderLabel.textContent = `${jointAngleDeg(human.jointLocalRot.r_shoulder).toFixed(0)}°`;

  controls.update();
  renderer.render(scene, camera);
  requestAnimationFrame(tick);
}
tick();
