// Sensor 3D Modeler — web port of the Python human-model demo.
// Mirrors src/human_model.py and src/animations.py: same joint tree, segment
// dimensions, and animation math, so behaviour matches the local Python version.

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
    for (const [segName, s] of Object.entries(SEGMENTS)) {
      const geom = new THREE.BoxGeometry(s.W, s.D, s.L);
      // Simple flat colour so the look matches matplotlib.
      const mat = new THREE.MeshStandardMaterial({
        color: s.color, roughness: 0.85, metalness: 0.0,
      });
      const mesh = new THREE.Mesh(geom, mat);
      const sign = s.dir === "up" ? 1 : -1;
      // Box is centered on its origin; shift so its near end sits at joint + inset.
      mesh.position.z = sign * (s.inset + s.L / 2);
      mesh.castShadow = true;
      mesh.receiveShadow = true;

      // Faint edges so segments read clearly against the dark background.
      const edges = new THREE.LineSegments(
        new THREE.EdgesGeometry(geom),
        new THREE.LineBasicMaterial({ color: 0x0d1117, transparent: true, opacity: 0.6 }),
      );
      mesh.add(edges);

      this.jointGroups[s.joint].add(mesh);
    }
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

const MOVEMENTS = [
  { key: "wave",  label: "Waving right arm", fn: waveArm },
  { key: "squat", label: "Squatting",        fn: squat  },
];
const CYCLE_SECONDS = 6.0;

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
const camera = new THREE.PerspectiveCamera(35, 1, 0.1, 100);
camera.up.set(0, 0, 1);
camera.position.set(0, 4.5, 1.4);

const controls = new OrbitControls(camera, canvas);
controls.target.set(0, 0, 0.95);
controls.enableDamping = true;
controls.minDistance = 2;
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
const btns = document.querySelectorAll("#controls button");
btns.forEach(btn => btn.addEventListener("click", () => {
  mode = btn.dataset.move;
  btns.forEach(b => b.classList.toggle("active", b === btn));
}));

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

  human.setPose(currentMove.fn(localT));
  human.groundLock();

  moveLabel.textContent = currentMove.label;
  timeLabel.textContent = `t = ${t.toFixed(2)} s`;

  controls.update();
  renderer.render(scene, camera);
  requestAnimationFrame(tick);
}
tick();
