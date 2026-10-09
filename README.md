# 3DSM — Sensor 3D Modeler

### ▶ [Open Live Demo](https://flashdato.github.io/3DSM_Project/)

**Intelligent Sensor Fusion for 3D Human Modelling**, a master's research project.
Wearable IMUs on each body segment, fused with a kinematic body model to reconstruct
3D human motion in real time.

[Project reference](PROJECT.md) · [Roadmap](PLAN.md) · [Hardware](HARDWARE.md) · [Changelog](CHANGELOG.md)

<!-- VERSION START: replace this block on every release; older versions go to CHANGELOG.md -->

## Current version: v0.5 — Two-arm live tracking + guided calibration + drift-free rest-anchor

*9 Oct 2026*

Both the right upper arm and right forearm are now driven live by their own
ESP-NOW IMU node. Calibration is a 3-pose guided flow with a ghost preview on
the viewer, and gyro-only yaw drift between the two sensors (which previously
appeared as the elbow "curling by itself" after a few motion cycles) is
zeroed out automatically every time the arm returns to rest.

### What works end-to-end

- **Two wireless nodes.** `firmware/esp32_sensor_now/` compiled per sensor
  with `NODE_ID = 1` (right forearm) and `NODE_ID = 2` (right upper arm),
  broadcasting raw IMU at 200 Hz (50 Hz packet rate, 4 samples per packet)
  over ESP-NOW. `firmware/esp32_receiver_now/` bridges to USB at 1 Mbaud and
  tags every CSV line with `D,<node_id>,…` so the host routes interleaved
  streams to the right per-node filter.
- **Per-sensor mounting calibration** (`tools/visualize_box_gy87.py`,
  unchanged solver). One-time 6-face capture per node → Kabsch solves
  `R_sb` → persisted to `calibrations/box_calib_node<N>.json`. The viewer
  renders the box accel-only (yaw rotation around world +Z is unobservable
  without a magnetometer and the display no longer pretends otherwise).
- **3-pose guided arm calibration** (`tools/visualize_arm_now.py`). Press
  `1`/`2`/`3` to capture REST / FORWARD-90° / SIDE-90°; each capture is a
  2-second averaged hold with the target shown as a green translucent ghost
  on the HumanModel. Yaw for each node is solved from the circular mean of
  two independent axis alignments — robust to small arm-position errors.
- **Live two-joint tracking.** Node 2 drives `r_shoulder` from its
  world-frame delta; node 1 drives `r_elbow` as
  `r_shoulder.T @ delta_forearm` so the forearm's rotation becomes
  elbow-local without double-counting the shoulder rotation. Mahony filter's
  accel correction gated to `0.85 < |a| < 1.15` so arm motion no longer
  corrupts pitch/roll via linear acceleration.
- **Auto re-anchor at rest.** When both nodes are stationary *and* their
  accel direction in box frame matches the stored rest reference (within
  ~11°), both nodes' `W_0` and `yaw_rad` are updated in-place to absorb the
  accumulated filter drift. Math is exact (not a workaround): δθ is read
  back from `W_new @ W_old.T` and the yaw correction is bumped by δθ. Zeros
  the elbow-curl symptom whenever the user returns their arm to rest.

### Next

- Hardware mounting (3D-printed cradle + wide elastic sports band) to kill
  the remaining drift source — sensors shifting on the arm during motion.
- Measurement runs vs. protractor (shoulder + elbow angle error) once
  mounting is stable.
- Decide magnetometer: deferred for now (clone QMC5883P was unusable the
  first time); auto-re-anchor is bridging the gap acceptably.

<!-- VERSION END -->

## Quick start

```bash
pip install -r requirements.txt
python src/main.py                               # right-arm demo (--full for whole body)
python tools/log_serial.py --port /dev/ttyUSB0   # record from the receiver ESP32
```

Flashing the ESP32s: [`firmware/README.md`](firmware/README.md). More commands, the
repository layout and conventions: [`PROJECT.md`](PROJECT.md).
