# Sensor_3D_Modeler

**Intelligent Sensor Fusion for 3D Human Modelling** — Masters research project.

A software-first pipeline that will grow into a real-time system for reconstructing
3D human pose and activity from fused wearable sensor data (starting with IMU,
extensible to other modalities). Development is staged: visualisation and simulation
come first, hardware is deferred until the model, animation, and evaluation loop are
solid.

---

## Motivation

Wearable inertial sensors are cheap, unobtrusive, and privacy-preserving compared to
cameras, but a single IMU cannot recover full-body pose on its own. Multi-sensor
fusion — combining several IMUs (and optionally other modalities) with a kinematic
body model — is the standard route to robust human-motion understanding.

Downstream applications under consideration:

- **Activity / gesture recognition** (walking, sitting, waving, falling).
- **Health monitoring** (fall detection, gait analysis, rehab tracking) — the most
  global-impact framing and the current front-runner for the final application.

The direction between generic activity classification and health-focused monitoring
will be finalised once the simulation pipeline produces enough signal to evaluate
both.

---

## Live web demo

The Phase 1 kinematic model runs directly in the browser as live client-side
code (Three.js, no build step, no video recording):

- Local preview: `cd docs && python -m http.server 8000`, then open
  `http://localhost:8000/`.
- GitHub Pages: enable Pages for this repo with source set to
  **`main` branch, `/docs` folder** — the page will publish at
  `https://<your-github-username>.github.io/Sensor_3D_Modeler/`.

Drag to rotate, scroll to zoom, use the buttons to switch between Wave, Squat,
and Auto-cycle.

The full staged roadmap lives in [`PLAN.md`](PLAN.md).

---

## Current status — Phase 1: Kinematic model & animation prototype

A 10-segment articulated human body (head, torso, upper arms, forearms, thighs,
shins) is rendered in 3D and driven through predefined joint trajectories. No
hardware is involved yet — the goal of this phase is to lock down the body model,
the coordinate conventions, and the animation loop that later phases will hook real
sensor data into.

Predefined demo movements implemented so far:

- **Wave** — right-arm raise + forearm oscillation.
- **Squat** — symmetric hip and knee flexion, with automatic pelvis-drop so the
  feet stay on the ground.

---

## Roadmap

| Phase | Scope | Hardware |
|-------|-------|----------|
| 1 (current) | Kinematic body model, forward kinematics, canned animations, 3D viz | none |
| 2 | Synthetic IMU streams generated from the kinematic model (ground-truth data for algorithm development) | none |
| 3 | Real IMU integration (e.g. MPU-6050 / BNO055 over serial or BLE), single-sensor pose | 1–2 IMUs |
| 4 | Multi-IMU fusion (Madgwick / Kalman / learned filter) driving the body model in real time | 5–7 IMUs |
| 5 | Application layer — activity recognition or health monitoring (fall / gait) | wearable rig |

---

## Repository layout

```
Sensor_3D_Modeler/
├── README.md
├── PLAN.md                # full 5-phase roadmap
├── requirements.txt
├── .gitignore
├── src/                   # local Python dev version (matplotlib)
│   ├── human_model.py     # 10-segment kinematic tree, forward kinematics, box rendering
│   ├── animations.py      # predefined joint trajectories (wave, squat) + ground-lock helper
│   └── main.py            # matplotlib interactive 3D animation loop
└── docs/                  # public web demo (GitHub Pages)
    ├── index.html         # dark-themed viewer with movement controls
    └── main.js            # Three.js port of the same model + animations
```

---

## Running the Phase 1 demo

Requires Python 3.10+.

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python src/main.py
```

An interactive matplotlib window opens showing the model cycling through the
predefined movements. Rotate the view with the mouse. Close the window to exit.

---

## Coordinate conventions

- World frame: right-handed. **+X right**, **+Y forward**, **+Z up**.
- Rest pose: standing upright, arms hanging along −Z at the sides, feet on the
  ground plane (z = 0).
- Each joint stores a local 3×3 rotation applied on top of its parent's world
  orientation; segments are drawn as rectangular boxes extending from the joint
  along the joint's local −Z (or +Z for head and torso).

---

## Planned sensor modality

- **IMU (accelerometer + gyroscope)** — primary modality for the fusion pipeline.

Additional modalities (radar, depth camera, RGB pose estimators) are not in scope
right now but the model is deliberately built so that any per-limb rotation source
can drive it.

---

## License

TBD.
