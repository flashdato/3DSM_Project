# Changelog

All versions of 3DSM, newest first. The README only shows the current version.
Everything older lives here.

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
