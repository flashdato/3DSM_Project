# esp32_visualize

ESP32 + GY-521 → serial stream of **roll & pitch** for the Python 3D
visualizer at `tools/visualize_rectangle.py`.

Complementary filter (98% gyro / 2% accel) on the ESP32. Yaw is
intentionally not sent — with no magnetometer it would drift.

## Wiring

Same as `esp32_mpu6050_test`: SDA=21, SCL=22, VCC=3V3, GND=GND, AD0 floating.

## Flash & run

1. Upload `esp32_visualize.ino`.
2. Put the board flat and still on the desk. On boot the sketch calibrates
   the gyro bias for ~1.5 s. You'll see `# bias dps: ...` on the serial
   monitor once it's done.
3. Close the Arduino Serial Monitor (only one program can hold the port),
   then in a terminal:

   ```bash
   cd Sensor_3D_Modeler
   source .venv/bin/activate    # if not already
   python tools/visualize_rectangle.py --port /dev/ttyUSB0
   ```

   (Windows uses `--port COM5` or similar; find the number in Device Manager.)

4. Tilt the board. The rectangle in the window should mirror it.

## Serial protocol

Baud 115200. One CSV line per sample at ~50 Hz:

```
roll_deg,pitch_deg,ax,ay,az,gx,gy,gz
```

Lines starting with `#` are comments (boot info). The visualizer ignores them.

## Notes

- If the rectangle sits at a nonzero angle when the board is flat, the
  initial calibration ran while the board was tilted. Reset the ESP32
  with the board level.
- Roll = rotation around the board's long axis (X). Pitch = rotation
  around the short axis (Y).
- To flip which axis is roll vs pitch in the visualization, swap the
  matrix order or negate one of the angles in `visualize_rectangle.py`.
