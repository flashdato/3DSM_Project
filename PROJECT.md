# 3DSM — Project Reference

Background and reference material that doesn't change from version to version.
For the latest update see [`README.md`](README.md). For the version history see
[`CHANGELOG.md`](CHANGELOG.md).

---

## What 3DSM is

**Intelligent Sensor Fusion for 3D Human Modelling**, a master's research project.

A software-first pipeline that will grow into a real-time system for reconstructing
3D human pose and activity from wearable IMU data. Development is staged: simulation
and visualisation come first, then real hardware, then multi-sensor fusion, and last
the application layer. The direction is **IMU-only**; cameras are at most an optional
ground-truth reference.

## Motivation

Wearable inertial sensors are cheap, unobtrusive and privacy-preserving compared to
cameras, but a single IMU cannot recover full-body pose on its own. Multi-sensor
fusion, which combines several IMUs with a kinematic body model, is the standard
route to robust human-motion understanding.

Downstream applications under consideration:

- **Activity / gesture recognition** (walking, sitting, waving, falling).
- **Health monitoring** (fall detection, gait analysis, rehab tracking).

The choice between them is made once pose reconstruction is validated.
Reconstruction accuracy is the main contribution, and the application sits on top.

---

## Documents

| File | What it holds |
|------|---------------|
| [`README.md`](README.md) | The current version only: what's new, what's next |
| [`CHANGELOG.md`](CHANGELOG.md) | Every version, newest first, plus the release checklist |
| [`PLAN.md`](PLAN.md) | The phased roadmap |
| [`HARDWARE.md`](HARDWARE.md) | Wireless IMU nodes: diagram, rates, time sync, packet format, wiring |
| [`firmware/README.md`](firmware/README.md) | Flashing the sensor nodes and receiver |
| `PROJECT.md` | This file: background, conventions, layout, how to run |

---

## Coordinate conventions

- World frame: right-handed. **+X right**, **+Y forward**, **+Z up**.
- Rest pose: standing upright, arms hanging along −Z at the sides, feet on the
  ground plane (z = 0).
- Each joint stores a local 3×3 rotation applied on top of its parent's world
  orientation. Segments are drawn as rectangular boxes extending from the joint
  along the joint's local −Z (or +Z for head and torso).

## Body model

An 11-joint kinematic tree rooted at the pelvis, with box segments: head, neck,
torso, upper arms, forearms, thighs and shins. The web demo currently shows only
the right arm (see the README). The full body is still in the code and returns
with `--full` (Python) or "Body outline" (web).

---

## Repository layout

```
3DSM_Project/
├── README.md              # current version only
├── CHANGELOG.md           # version history + release checklist
├── PROJECT.md             # this reference
├── PLAN.md                # phased roadmap
├── HARDWARE.md            # wireless IMU nodes
├── requirements.txt
├── src/                   # Python dev version (matplotlib)
│   ├── human_model.py     # kinematic tree, forward kinematics, box rendering
│   ├── animations.py      # joint trajectories (right-arm set, wave, squat) + helpers
│   ├── main.py            # 3D animation loop (right arm by default, --full)
│   └── virtual_imu.py     # simulated MPU-6050 raw data from model motion
├── firmware/              # Arduino sketches (ESP32)
│   ├── sensor_node/       # worn node: MPU-6050 sampling, beacon sync, ESP-NOW send
│   └── receiver/          # USB receiver: time beacons, decode, CSV over serial
├── tools/
│   └── log_serial.py      # records the receiver's stream to recordings/<session>/
└── docs/                  # public web demo (GitHub Pages)
    ├── index.html
    └── main.js            # Three.js port of the same model + animations
```

Recordings (`recordings/`), the professor's materials and IDE settings are kept out
of git (see `.gitignore`).

---

## How to run

Requires Python 3.10+.

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt

python src/main.py                               # right-arm demo
python src/main.py --full                        # full-body demo
python src/main.py --snapshot arm.png --t 1.5    # save one frame, no window
python tools/log_serial.py --port /dev/ttyUSB0   # record from the receiver ESP32
python src/virtual_imu.py --move all --plot       # simulated recording (same format)
```

**Web demo, local preview:** `cd docs && python -m http.server 8000`, then open
`http://localhost:8000/`.

**Web demo, published:** GitHub Pages is served from the `main` branch, `/docs`
folder, at <https://flashdato.github.io/3DSM_Project/>.

---

## License

TBD.
