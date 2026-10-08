# esp32_visualize_gy87

Wired visualizer for the GY-87 sensor on the right upper arm. Streams full
3-axis orientation (relative to a 5-second N-pose calibration) over USB to
`tools/visualize_arm_gy87.py`, which renders the project's **full articulated
body model** (`src/human_model.py`) with the right arm driven by the sensor
so you can read motion relative to the rest of the body.

This is the full-10DOF successor to `../esp32_visualize/` (which is MPU-6050
only and shows just roll + pitch). The magnetometer in the GY-87 lets us
report **yaw too** without drift — rotating around the vertical axis now
moves the on-screen arm correctly.

## Boot sequence

1. **~2 s warmup.** After power/reset, the sketch gives the board a moment
   to settle and the serial link to come up. You'll see `# warmup 2000 ms...`.
2. **5 s N-pose calibration.** You'll see `# CALIBRATING: hold arm hanging
   down, palm toward thigh, 5 s...` followed by a per-second countdown.
   Stand upright, right arm hanging straight along your side with the palm
   facing your thigh. Hold still. The sketch averages accel, gyro and mag
   over that window to capture (a) gyro bias and (b) the body→world
   rotation at rest.
3. **Streaming at 50 Hz** after calibration. CSV format:

   ```
   qw,qx,qy,qz,ax,ay,az,gx,gy,gz,mx,my,mz
   ```

   `qw..qz` is a unit quaternion representing the WORLD rotation that
   moves the rest-pose segment to its current pose. At N-pose it is
   identity (1, 0, 0, 0). The other 9 fields are raw accel [g],
   debiased gyro [dps] and mag [counts] for any downstream processing.

## Wiring

Same 5V-side layout as `../esp32_gy87_test/`:

| GY-87 pin | ESP32 pin |
|-----------|-----------|
| VCC_IN    | VIN / 5V  |
| GND       | GND       |
| SDA       | GPIO 26   |
| SCL       | GPIO 25   |

INT, DRDY and FSYNC left floating.

## Flash & run

1. Open `esp32_visualize_gy87.ino` in Arduino IDE, pick your ESP32 board,
   upload. No extra libraries needed beyond the built-in `Wire.h`.
2. Close the Arduino Serial Monitor (only one program can hold the port).
3. In a terminal:

   ```bash
   cd Sensor_3D_Modeler
   source .venv/bin/activate
   python tools/visualize_arm_gy87.py --port /dev/ttyUSB0
   ```

   (Windows: `--port COM5` or similar.)

4. Stand upright at N-pose through the 7 s warmup + calibration window.
   Once the Python window shows the full body at rest (arms hanging, feet
   on the ground), start moving your right arm — the model's right arm
   should mirror it. You can switch which joint is driven by passing
   `--driven-joint r_elbow` etc. for later experiments.

## What good output looks like

```
# esp32_visualize_gy87
# warmup 2000 ms...
# CALIBRATING: hold arm hanging down, palm toward thigh, 5 s...
#   5s left
#   4s left
#   3s left
#   2s left
#   1s left
# N-pose captured: a=(-0.998,0.014,-0.021) m=(1312,-3287,1498)
# gyro bias dps: (-0.21, +0.08, -0.14) from 833 samples
# format: qw,qx,qy,qz,ax,ay,az,gx,gy,gz,mx,my,mz
1.0000,0.0000,0.0000,0.0000,-1.001,0.013,-0.020,-0.1,0.0,0.1,1304,-3285,1502
...
```

Key sanity checks:

- `a=(-1, 0, 0)` at N-pose → sensor X points down (toward the hand), which
  matches the mounting convention in `../../HARDWARE.md`.
- First few `qw` values should be ~1.0 and `qx,qy,qz` should be ~0 — the
  arm is at rest, so the "rotation from rest" is identity.
- Rotate your arm 90° outward: `qw` drops, one of `qx/qy/qz` grows.

## Common issues

- **Arm starts in the wrong orientation.** The N-pose average captured
  during the 5-second window IS the rest pose, so if the model doesn't
  look right at rest, you weren't actually at N-pose when calibration ran.
  Reset the ESP32 and redo the capture.
- **Model drifts when standing still.** Expected to be small (accel + mag
  are drift-free, we're not integrating gyro here). If it's large, you're
  near a magnetic disturbance — stand away from laptops, large motors or
  metal desks for the first bring-up.
- **Model flips suddenly.** Can happen if the magnetometer reading was
  dominated by a stray field during calibration. Walk a step and recalibrate.

## Next

- When this looks clean, we switch from this one-off wired path to the
  real pipeline: `../sensor_node/` (which already does the ESP-NOW +
  beacon-sync layer for 2 nodes) will be extended to carry the mag
  channel so the quaternion math moves to the host.
