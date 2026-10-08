# esp32_gy87_test

Full bring-up for ESP32 + **GY-87 10DOF** module. The GY-87 is what Phase 3
actually ships with (see `../../HARDWARE.md`), while `esp32_mpu6050_test`
only exercised the MPU-6050 half of it. This sketch extends the same style:
pure `Wire.h` for the MPU-6050 and the magnetometer, Adafruit's BMP085
library for the barometer (the BMP180 compensation maths are too long to be
worth hand-writing for a bring-up test).

Prints an I²C scan, verifies each of the three chips and then streams all
sensors at ~20 Hz for a human to eyeball.

## Why this exists

The MPU-6050 keeps its auxiliary I²C bus **hidden** from the host by default,
so the magnetometer (HMC5883L at 0x1E, or the QMC5883L clone at 0x0D) and
the BMP180 (0x77) don't show up in a plain scan. This sketch enables I²C
bypass mode so all three devices become directly addressable, then
auto-detects whichever magnetometer variant is on your board.

## Wiring

All four wires come off the **5V side** of the ESP32 DevKit (opposite the
default `sensor_node` wiring, so this test board can live on a breadboard
with everything leaving the same edge):

| GY-87 pin | ESP32 pin |
|-----------|-----------|
| VCC_IN    | **VIN / 5V** — the GY-87 drops it to 3.3V on-board |
| GND       | GND (the one next to VIN) |
| SDA       | **GPIO 26** |
| SCL       | **GPIO 25** |

INT, DRDY and FSYNC are left floating.

GPIO 26 and 25 were chosen because they sit on the same edge as VIN/GND,
they aren't strapping pins and don't conflict with on-chip flash. Any
other output-capable pair works too — change `PIN_SDA` and `PIN_SCL` at
the top of the sketch. Just avoid the strapping / flash pins (GPIO 0, 2,
5, 6–11, 12, 15) and the input-only pins (34–39).

Note: the real `../sensor_node/` and `../receiver/` sketches still use
GPIO 21 / 22. If you want to standardise the whole project on the 5V
side later, change `PIN_SDA` / `PIN_SCL` there too.

## Flash & run

1. Arduino IDE → Library Manager → install **Adafruit BMP085 Library**
   (works for BMP180 too). `Wire.h` is built in; no other library needed.
2. Open `esp32_gy87_test.ino`, select your ESP32 board, upload.
3. Serial Monitor at **115200 baud**.

## What you should see

```
=== ESP32 + GY-87 10DOF bring-up ===
MPU-6050 WHO_AM_I = 0x68  (expect 0x68 or 0x72 on clones)
I2C scan:
  found 0x0D         (or 0x1E on a genuine HMC5883L)
  found 0x68
  found 0x77
QMC5883L ok at 0x0D
BMP180 ok at 0x77

Streaming (a [g] | g [dps] | m [counts] | P [hPa] alt [m] | T [C]):
a:  +0.01  +0.02  +1.00 | g:   +0.3   -0.2   +0.1 | m:   +134    -82   +421 | 1012.42 hPa   +8.3 m | 25.4 C
```

Sanity checks (do these in order):

- **Flat on the desk** → one accel axis reads ~+1.00 g, the other two ~0.
- **Rotate 90°** → the +1 g moves to a different axis.
- **Hold still** → gyro reads within a few dps of zero (that's bias).
- **Spin the board in the horizontal plane** → `mx` and `my` swing through
  thousands of counts; one grows while the other shrinks each quarter turn.
- **Blow lightly on the baro hole** → pressure drops a few hPa, altitude
  jumps up a few metres. Pressure near 1013 hPa at sea level.

## Common issues

- **I²C scan shows only 0x68.** Bypass bit didn't take. Power-cycle; check
  that nothing on the GY-87's AUX pads is shorted.
- **WHO_AM_I = 0xFF.** SDA/SCL swapped, or no 3V3 reaching the module.
- **WHO_AM_I = 0x72.** You have an MPU-6500 or clone. Register map matches,
  sketch still works.
- **No magnetometer found.** Three magnetometer variants ship on "GY-87"
  boards and this sketch tries all three in order: HMC5883L (0x1E),
  QMC5883L (0x0D), QMC5883P (0x2C, the newest QST part). If the scan shows
  a device at an *unlisted* address, open an issue — common oddball parts
  are IST8310 (0x0E) and AK09916 (0x0C). If no mag address appears at
  all, the chip may be absent on your clone or the bypass bit didn't
  stick (power-cycle).
- **Pressure flat or NaN.** `bmp.begin()` failed even though the scan saw
  0x77. Usually resolved by a clean power-cycle.

## What this unlocks over the GY-521 (6DoF) test

| Capability                                          | GY-521 (6DoF) | **GY-87 (10DoF)** |
|-----------------------------------------------------|:-:|:-:|
| 3-axis accel + 3-axis gyro                          | ✓ | ✓ |
| Absolute heading (yaw) with no drift                | ✗ | **✓**  (magnetometer) |
| Tilt-compensated compass heading                    | ✗ | ✓ |
| Vertical motion (jumps, squats, falls) separable    | ✗ | **✓**  (barometer) |
| Altitude reference for the pelvis node (Phase 4)    | ✗ | ✓ |
| Fall detection via altitude drop + impact spike     | hard | **easy** |
| Yaw drift stops compounding across body nodes       | ✗ | ✓ |

**Why those matter for 3DSM:**

- **Phase 3 (right arm, elbow angle).** Doesn't strictly need mag or baro —
  elbow flexion is the *relative* orientation of two accel+gyro sensors.
  But adding the mag removes the "stand still so the upper arm has a
  reference" constraint in `HARDWARE.md`, because each node now has an
  absolute heading. Useful for ground-truth checks.
- **Phase 4 (full body).** Without a magnetometer, every node's yaw drifts
  independently and the model slowly rotates around Z. With mag, each node
  sits in a shared absolute frame — the kinematic solver stops fighting
  gyro drift.
- **Phase 5 (fall / activity detection).** The barometer is the single most
  useful extra channel. A fall is a rapid altitude drop followed by an
  impact spike; distinguishing a fall from a hard sit is much easier with
  the vertical axis than from gyro signatures alone.

The packet format in `../sensor_node/imu_protocol.h` already reserves
`mag[3]` fields and a `FLAG_MAG_VALID` bit — the `sensor_node` sketch will
be extended to populate them once Phase 3 bring-up on real boards is clean.

## Next

Once this streams clean numbers:

1. Switch to `../sensor_node/`. Same MPU-6050 setup, plus data-ready
   interrupt timestamps, beacon time sync and ESP-NOW transport.
2. Later, extend `sensor_node` to also read the mag and baro on each
   sample (slower than the gyro — poll both at ~50 Hz, interpolate on the
   host).
