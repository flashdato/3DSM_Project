# Project Roadmap

**Intelligent Sensor Fusion for 3D Human Modelling** — masters project.

Five sequential phases, each unlocks the next. Every phase produces a runnable
artifact so progress is verifiable, not theoretical.

---

## Phase 1 — Visualization & Public Web Demo

**Goal:** a live 3D human model, viewable in the browser, that anyone can spin
and inspect. This becomes the visible face of the project and the reference
renderer for every later phase.

- 10-segment articulated body (head, torso, upper arms, forearms, thighs, shins)
  built as a kinematic tree rooted at the pelvis.
- Predefined animations (wave, squat) proving the joints work correctly.
- Two implementations, same model, kept in sync:
  - **Python + matplotlib** — the local development version (`src/`).
  - **JavaScript + Three.js** — the web version (`docs/`), deployed to
    GitHub Pages so the demo is *live code running client-side*, not a
    recorded video.
- Deliverables:
  - `docs/index.html` interactive page (mouse-drag rotates the view).
  - Working GitHub Pages URL.

**Definition of done:** anyone opening the GitHub Pages URL sees the human
model cycling through the predefined motions in real time.

---

## Phase 2 — Data Pipeline & Model Driver

**Goal:** decouple pose generation from pose rendering, so anything that can
produce per-joint rotations can drive the model — including, later, real sensor
data.

- Define a stream format: `{timestamp, joint_id, quaternion}` at ~50–100 Hz.
- Write a **replay tool** that loads a recorded stream (CSV or JSONL) and
  drives the visualizer in real time or accelerated.
- Write a **synthetic generator** that samples the existing Phase 1 animations
  and emits streams in the same format — used both as a testbed and as
  ground-truth data for Phase 4 fusion algorithms.
- Split animation logic (`animations.py`) into a *poser* interface so the same
  main loop can consume either canned poses or a live stream.

**Definition of done:** the model can be driven from a file the same way it
will later be driven from a sensor, with no code changes to the renderer.

---

## Phase 3 — First Real Sensors (right arm)

**Goal:** get physical IMUs driving the right arm of the model. Prove the full
loop end-to-end: hardware → radio → receiver → parser → renderer.
Details: [`HARDWARE.md`](HARDWARE.md).

- **Node:** one **ESP32 + GY-87 (MPU-6050)** + LiPo per body segment. Raw
  accelerometer + gyroscope at 100 Hz (200 Hz possible), timestamped at the
  MPU-6050 data-ready interrupt. No on-chip fusion: orientation filters run on
  the host so they can be compared on the same recordings.
- **Radio:** **ESP-NOW** from each node to a **receiver ESP32** on the laptop's
  USB (no router needed). The receiver broadcasts a **time beacon** every second.
  Nodes sync their clocks to it, so samples from different limbs share one
  timeline.
- **Step 1:** 2 nodes, right upper arm + right forearm → elbow angle from the
  relative rotation of the two segments. Firmware in `firmware/`, recorder in
  `tools/log_serial.py`.
- **Next:** Python bridge that converts raw counts to units, runs an
  orientation filter per node, maps sensor frame → segment frame and drives the
  right arm of the model live.
- Calibration routine: stand in the N-pose (arms hanging) for 3 s and capture the
  sensor-to-segment offset, so later readings are relative to the model's rest
  frame.
- First measurements: 15-min static recording (real gyro bias/noise for the
  simulator's noise model), tap test for time-sync error, elbow flexion vs
  protractor.

**Definition of done:** moving the real arm moves the model's right arm on
screen in real time, with acceptable latency (< 100 ms), node-to-node sync
error well under one sample period, and no obvious drift over a few minutes.

---

## Phase 4 — Multi-Sensor Fusion

**Goal:** move from one limb to a full-body pose reconstructed from multiple
IMUs at once, with the kinematic constraints of the model enforcing
consistency.

- Sensor layout: 6–7 IMUs — pelvis (root), both thighs, both shins, both
  upper arms (optionally forearms). Head + torso can share one, or be
  derived from the pelvis sensor.
- Hardware (chosen): one wireless ESP32 + IMU + battery node per segment,
  up to **10 nodes** (pelvis, chest, both upper arms, forearms, thighs, shins),
  ESP-NOW to the receiver ESP32 with beacon time sync, and a **Raspberry Pi**
  hub for recording, filtering and the kinematic solver. At 10 nodes the ESP-NOW
  PHY rate and batching and the receiver → host link need upgrading (see
  HARDWARE.md, "Scaling to 10 nodes").
  - Rejected alternative: one ESP32 with an I²C mux (TCA9548A). Long I²C wires
    to the limbs are unreliable, and every GY-87 has the same I²C addresses.
- Fusion approach — pick one, benchmark the others:
  - **Constraint-based** — take each IMU's independent orientation, then
    project through the kinematic tree so parent/child rotations are
    consistent (fast, no training data needed).
  - **Filter-based** — Extended Kalman filter over the full state vector
    of joint angles, IMU biases, and orientation offsets.
  - **Learned** — small neural network trained on synthetic streams from
    Phase 2 with added realistic sensor noise.
- Robustness: missing-sensor / dead-sensor handling, T-pose recalibration
  hotkey.

**Definition of done:** a person wearing the sensor suit moves, and the
model on screen mirrors them within acceptable error (compare against
video ground truth).

---

## Phase 5 — Application & Fine-Tuning

**Goal:** pick the real-world application and take the system from a demo
to something evaluable on the chosen task.

Two candidate directions — one will be selected once Phase 4 produces usable
data:

- **Movement / activity recognition** — classify what the person is doing
  (walking, running, sitting, standing, waving, falling). Straightforward
  benchmark, lots of public datasets, easy to demo.
- **Medical / health monitoring** — fall detection, gait analysis for
  rehabilitation, posture warnings for the elderly. Higher social impact,
  harder to evaluate, needs collaboration with a clinical contact or a
  public medical dataset.

Regardless of which is chosen:

- Data collection protocol (subjects, environments, motion vocabulary,
  ground-truth labeling).
- Train + evaluate a classifier (or a regression model for gait metrics).
- Iterate on: sensor count and placement, sample rate, window length,
  model architecture.
- Compare against public baselines on the chosen dataset.

**Definition of done:** the final report can state, with numbers, "on task
X, our sensor-fusion system achieves Y on public dataset Z, compared to
prior work W."

---

## Cross-cutting workstreams

Kept in sight across all phases so they don't become end-of-project fires:

- **Reproducibility** — every experiment scripted, config in files not code,
  seeds set.
- **Latency budget** — target end-to-end < 100 ms sensor→screen; measure at
  each phase.
- **Documentation** — README + this plan updated as decisions are made; each
  phase closes with a short "what worked / what didn't" note.
- **Data hygiene** — recorded sessions timestamped, labeled with subject +
  environment, stored outside the repo (repo tracks only tiny sample clips).
