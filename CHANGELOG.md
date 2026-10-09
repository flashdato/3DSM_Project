# Changelog

All versions of 3DSM, newest first. The README only shows the current version.
Everything older lives here.

---

## v0.5 — Two-arm live tracking + guided calibration + drift-free rest-anchor · 2026-10-09

Second IMU node is live on the right upper arm; the right arm's shoulder *and*
elbow are now driven by sensors in real time. Calibration was rewritten as a
3-pose guided flow with a ghost preview on the viewer, and the gyro-only yaw
drift that was manifesting as the elbow "curling by itself" after a few motion
cycles is now zeroed out automatically every time the arm returns to rest.

**Added**
- `tools/visualize_arm_now.py`: full rewrite for two concurrent nodes.
  Per-node `R_sb`, Mahony filter, `rest_W`, `yaw_rad`. Node 2 (upper arm)
  drives `r_shoulder`; node 1 (forearm) drives `r_elbow` computed as
  `r_shoulder.T @ delta_forearm` so the forearm's world-frame rotation
  becomes an elbow-local rotation (parent-chain consistency).
- 3-pose guided calibration: keys `1`=REST, `2`=FORWARD-90°, `3`=SIDE-90°.
  Each a 2-second averaged capture (first 0.5s discarded so the Mahony
  filter has time to settle). Target pose drawn as a green translucent
  ghost arm overlaid on the HumanModel; the live model stays in rest pose
  during calibration so the user focuses on matching the ghost. Yaw per
  node is solved from the circular mean of two independent axis alignments
  (FORWARD delta's rotation axis → +X, SIDE delta's rotation axis → -Y) —
  robust to small arm-position errors.
- **Auto re-anchor at rest** — the drift fix. For each node, detect
  (|a| ≈ 1g, |gyro| < 10 °/s) AND (accel direction in box frame matches
  stored rest reference within ~11° cosine). When both nodes satisfy this
  for 0.5 s with a 2 s cooldown, re-anchor: `W_0 ← current filter W` and
  `yaw += -atan2(M[1,0], M[0,0])` where `M = W_new @ W_old.T`. Math is
  exact: δθ (accumulated filter yaw drift) is readable from M and the two
  updates together make the delta formula return `R_WL` cleanly from the
  anchor forward. Rest detection uses raw accel (not filter W), so it
  stays reliable regardless of drift.

**Changed**
- Receiver CSV format: each sample line now prefixed with `D,<node_id>,` so
  interleaved streams from multiple nodes route correctly on the host.
  `tools/visualize_box_gy87.py` parses both old and new formats; the arm
  viewer uses the new format exclusively. Receiver re-flash required once.
- `tools/visualize_box_gy87.py`: display switched from Mahony-filter-driven
  to accel-only. The previous display picked an arbitrary yaw on each face
  capture (filter init math) which looked like the rendered box was
  "wrong" even when `R_sb` was correct. Yaw rotation around world +Z is
  not observable from accel alone (and there's no magnetometer), so the
  display convention is now explicit about this instead of pretending to
  know.
- Mahony filter in the arm viewer: accel-based pitch/roll correction is
  now gated to `0.85 < |a| < 1.15`. Previously the correction ran every
  tick, so during arm motion the filter was being fed `gravity +
  linear_accel` as if it were pure gravity — perturbing the quaternion
  differently per sensor (upper arm and forearm swing through different
  arcs) and *contributing* to the elbow-curl drift.

**Fixed**
- Interleaved samples from two concurrent nodes no longer get mis-routed
  (receiver now tags node_id per line).
- Elbow "curling by itself" over motion is largely eliminated by
  auto-re-anchor (verified by user: "much much better... much more stable").

**Known limitations**
- Strap slippage during motion: the sensor shifts on the arm between
  rest-returns, changing the mounting rotation that the calibration
  assumed. No software fix — needs mounting hardware (3D-printed cradle +
  wide elastic strap, or sewn-in pocket on a compression sleeve).
- Residual yaw drift between rest-returns (gyro bias, scale factor).
  Magnetometer would eliminate it; deferred.

**Verified**
- Both ESP-NOW nodes online simultaneously, receiver tagging node_id per
  sample.
- Live model mirrors shoulder + elbow motion in real time; auto-re-anchor
  resets accumulated drift when the arm returns to rest.

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
