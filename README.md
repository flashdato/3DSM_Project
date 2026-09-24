# 3DSM — Sensor 3D Modeler

### ▶ [Open Live Demo](https://flashdato.github.io/3DSM_Project/)

**Intelligent Sensor Fusion for 3D Human Modelling**, a master's research project.
Wearable IMUs on each body segment, fused with a kinematic body model to reconstruct
3D human motion in real time.

[Project reference](PROJECT.md) · [Roadmap](PLAN.md) · [Hardware](HARDWARE.md) · [Changelog](CHANGELOG.md)

<!-- VERSION START: replace this block on every release; older versions go to CHANGELOG.md -->

## Current version: v0.2 — Hardware bring-up plan

*25 Sep 2026*

### Progress so far

- **Phase 1 is done.** The articulated body model with forward kinematics runs in Python and in the browser (live demo above).
- **Direction is fixed: IMU-only.** It follows the professor's research design: simulation and quantitative checks first, real sensors next, fusion after that, and the health/activity application last.
- **Hardware is designed.** Each body segment gets its own wireless node (ESP32 + GY-87 / MPU-6050 + LiPo), and the nodes send raw data over ESP-NOW to a receiver ESP32. The receiver broadcasts a time beacon every second so all nodes share one clock. Details are in [`HARDWARE.md`](HARDWARE.md).
- **Firmware and recorder are written.** The node and receiver sketches compile for ESP32 and ESP32-S3, and `tools/log_serial.py` saves sessions to `recordings/`. They haven't run on real boards yet.
- **The demo is now right-arm only.** It shows the upper arm and forearm with the two IMU nodes, live elbow and shoulder angles, and four moves: elbow curl, arm raise, side raise and wave.

### Next: v0.3 plan

**Hardware track (2 nodes, right arm)**
1. Build the 2 nodes and the receiver, flash them, and confirm 100 samples/s per node with 0 lost and both nodes synced.
2. Make a 15-minute static recording to get the real gyro bias and noise.
3. Run a tap test to measure node-to-node sync error, at the start and after 10 minutes.
4. Record elbow flexion 0→90→0° against a protractor.

**Software track (runs in parallel)**
1. Quaternion utilities with unit tests.
2. A virtual IMU generator that turns the demo animations into synthetic accel/gyro data with configurable bias and noise, using the numbers from hardware step 2.
3. An orientation filter (complementary first) and the N-pose sensor-to-segment calibration.
4. A Python bridge that plays back a recording or live stream on the model's right arm.
5. The first metrics: orientation error and elbow-angle error against ground truth.

**Done when:** moving the real arm moves the model's right arm on screen with under
100 ms latency, and the elbow angle error has a measured number.

<!-- VERSION END -->

## Quick start

```bash
pip install -r requirements.txt
python src/main.py                               # right-arm demo (--full for whole body)
python tools/log_serial.py --port /dev/ttyUSB0   # record from the receiver ESP32
```

Flashing the ESP32s: [`firmware/README.md`](firmware/README.md). More commands, the
repository layout and conventions: [`PROJECT.md`](PROJECT.md).
