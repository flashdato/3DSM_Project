# Changelog

All versions of 3DSM, newest first. The README only shows the current version.
Everything older lives here.

---

## v0.4 — Wireless IMU + per-sensor box & arm viz · 2026-10-08

Real GY-87 hardware is now driving the model wirelessly, and a two-step
mounting calibration persists per sensor so the user does not re-calibrate
every launch. Mag channel is skipped on this board (the clone QMC5883P is
unreliable); orientation is pure accel + gyro.

**Added**
- `firmware/esp32_sensor_now/`: ESP-NOW IMU sender. MPU-6050 at 200 Hz,
  4 samples per packet (50 Hz pkt rate), fixed WiFi channel, broadcast peer
  (no pairing — a power-cycled sensor streams again instantly). 3-second
  gyro-bias calibration at boot. Compile-time `NODE_ID` for multi-node setups.
- `firmware/esp32_receiver_now/`: ESP-NOW receiver → USB serial bridge at
  1 Mbaud, in the CSV format the Python tools already parse. Per-node
  sequence tracking, reconnect notice (`# node X online (seq N)`), drop
  stats every 2 s.
- `tools/visualize_box_gy87.py`: live 3D box visualizer with a static legend
  and keys 1–6 to learn each face's "down" accel direction. Kabsch solves
  the sensor-to-box mounting `R_sb` from the learned accels, then a
  Mahony-style accel+gyro complementary filter drives the display (no mag,
  no yaw singularity). Online gyro-bias refinement when stationary.
  Calibration persists to `calibrations/box_calib_node<N>.json`,
  auto-loaded on launch.
- `tools/visualize_arm_now.py`: HumanModel right-arm visualizer driven by
  the same filter. Loads the box tool's `R_sb` so the sensor arrives already
  calibrated. Two-step pose anchor: `c` captures the shoulder zero at rest,
  `t` captures a forward-raise and solves `yaw = atan2(-ay, ax)` to align
  filter's arbitrary horizontal frame with HumanModel's +X=right / +Y=forward.
  Both saved per node to `calibrations/arm_rest_node<N>.json`.
- `docs/images/box_viz.png` and `docs/images/arm_viz.png`: screenshots of
  the two tools running live.

**Changed**
- `.gitignore` now excludes `calibrations/` so personal sensor data stays
  local.

**Fixed**
- Yaw drift (≈180° / minute) on the box viz: added firmware boot-time
  bias calibration and Python-side online bias refinement.
- Firmware quaternion was unusable because of the GY-87 clone's noisy
  magnetometer. All orientation now computed host-side from accel + gyro.

**Verified**
- Box visualizer tracks physical face flips 1-to-1 after 6-face mounting
  calibration (user confirmed "working perfectly").
- Arm visualizer mirrors shoulder motion after `c`+`t` capture; previously
  stuck sideways-raise/forward-raise mapping now correct.

---

## v0.3 — Virtual IMU + live sensor charts · 2026-09-25

**Added**
- `src/virtual_imu.py`: simulated MPU-6050 output from the model's motion. It computes ideal accel/gyro by central differences on the animation, adds per-node bias, noise, int16 quantization and optional packet loss, and writes the **same `imu_raw.csv` format as a real recording**, plus `truth.csv` (ideal signals, true orientation, joint angles), `meta.json` and an optional `signals.png`.
- Web demo: live **Simulated MPU-6050 output** panel with raw counts per node (and the receiver's `D,...` line), plus 4 scrolling charts (accel and gyro per node, last 6 s) drawn from the same simulated numbers. Node boxes now show the real board orientation.

**Changed**
- Unit conversion uses the datasheet sensitivities (gyro ±1000 °/s = 32.8 LSB per °/s, not 32768/1000). Board mounting convention defined in `HARDWARE.md`.

---

## v0.2 — Hardware bring-up plan · 2026-09-25

**Direction.** IMU-only, following the professor's research design: simulation and
quantitative checks first, real sensors second, fusion third, and the health or
activity application last. Camera fusion is out of scope.

**Added**
- `HARDWARE.md`: the full picture for the first hardware step. It covers 2 wearable nodes (ESP32 + GY-87 / MPU-6050 + LiPo) on the right upper arm and forearm, ESP-NOW to a receiver ESP32, and USB to the laptop. It also has sensor and radio rate budgets, the time-sync protocol, the packet format, wiring, placement, and scaling to 10 nodes + Raspberry Pi.
- `firmware/sensor_node`: samples the MPU-6050 at 100 Hz (up to 500 Hz), stamps each sample at the data-ready interrupt, syncs its clock to the receiver's beacons, learns the receiver's address automatically, and sends raw accel/gyro in batches.
- `firmware/receiver`: sends a 1 Hz time beacon, decodes all nodes, and prints CSV over USB with per-node stats (packets/s, lost samples, RSSI, latency, sync state).
- `tools/log_serial.py`: records a session to `recordings/<time>/` (raw CSV + metadata + stats).
- Right-arm movement set: elbow curl, forward arm raise, side raise, wave.

**Changed**
- Web demo and Python viewer show **the right arm only**, with both IMU nodes, their sensor axes, and live elbow and shoulder angles. The full-body model is kept: use `--full` or "Body outline".
- `PLAN.md` Phase 3 was rewritten for the wireless node design, and Phase 4 now records the 10-node + Raspberry Pi choice.
- README reduced to the current version. Background moved to `PROJECT.md`, and history to this file.

**Verified**
- Both sketches compile for ESP32 and ESP32-S3 (Arduino ESP32 core 3.3.2). They have not yet run on real boards.
- The recorder was tested against a simulated receiver, and the Python viewer renders.

---

## v0.1 — Kinematic model & web demo · 2026-09-22

**Added**
- Articulated body model (head, torso, arms, forearms, thighs, shins) as a pelvis-rooted kinematic tree with forward kinematics.
- Wave and squat animations, with automatic ground lock so the feet stay on the floor.
- Python + matplotlib viewer (`src/`) and a Three.js web demo (`docs/`) on GitHub Pages.
- `PLAN.md` with the 5-phase roadmap.

---

## How to release a new version

Do this every time an update is pushed, so the README always shows only the latest
state:

1. **Add a new section at the top of this file:** `## vX.Y — short name · YYYY-MM-DD`, with **Added / Changed / Fixed / Verified** as needed. Leave the older sections untouched.
2. **Replace the README's version block** (the part between the `VERSION` markers) with the new version's summary and its "Next" plan. Don't append; the old block now lives here.
3. **Update anything else that moved,** such as `PLAN.md` phase status, `PROJECT.md` layout or commands, and `HARDWARE.md`.
4. **Commit** with the message `3DSM vX.Y: short name`, then tag it: `git tag vX.Y && git push && git push --tags`.

Numbering: bump the minor number (0.2 → 0.3) for each update, and the patch number
(0.2 → 0.2.1) for small fixes. 1.0 is the first full working system (Phase 4 done).
