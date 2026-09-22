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

## Phase 3 — Single Real Sensor

**Goal:** get one physical IMU driving one limb of the model. Prove the full
loop end-to-end: hardware → wire → parser → renderer.

- Choose the IMU: **BNO055** (Bosch, has on-chip sensor fusion, exposes
  quaternion directly — removes the need for a filter on day one) or
  **MPU-9250** (cheaper, but requires implementing Madgwick/Mahony ourselves).
- Micro: **ESP32** — reads IMU over I²C, streams `{timestamp, quaternion}`
  either over USB serial or BLE / Wi-Fi UDP.
- Python bridge: `sensor_bridge.py` reads the stream, converts the sensor
  frame to the corresponding joint frame, injects it into the Phase 2 pipeline
  as the pose for a single limb (e.g., right forearm), leaves the rest at
  rest.
- Calibration routine: hold limb in a known pose, capture the offset
  quaternion so subsequent readings are relative to the model's rest frame.

**Definition of done:** rotating the physical IMU by hand rotates the
corresponding limb on screen in real time, with acceptable latency
(< 100 ms) and no obvious drift over a few minutes.

---

## Phase 4 — Multi-Sensor Fusion

**Goal:** move from one limb to a full-body pose reconstructed from multiple
IMUs at once, with the kinematic constraints of the model enforcing
consistency.

- Sensor layout: 6–7 IMUs — pelvis (root), both thighs, both shins, both
  upper arms (optionally forearms). Head + torso can share one, or be
  derived from the pelvis sensor.
- Hardware options:
  - Multiple ESP32s each with one IMU, syncing over Wi-Fi/UDP to a hub.
  - Or one ESP32 driving an I²C mux (TCA9548A) to talk to several IMUs
    through the same bus.
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
