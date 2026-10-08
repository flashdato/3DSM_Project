# esp32_mpu6050_test

First bring-up sketch for ESP32 + **GY-521 (MPU-6050)**. No libraries besides
`Wire.h`, no radio, no time-sync — just prove the sensor is alive and reads
sensible numbers on the Serial Monitor. Once this works we move on to the
real `sensor_node/` sketch (ESP-NOW + timestamps).

## Wiring

| GY-521 pin | ESP32 pin |
|------------|-----------|
| VCC        | 3V3 (5V also OK on GY-521, it has a regulator) |
| GND        | GND       |
| SCL        | GPIO 22   |
| SDA        | GPIO 21   |
| AD0        | leave floating -> I²C address 0x68 |

INT, XCL, XDA are unused for this test.

## Flash

1. Arduino IDE → Boards Manager → install **esp32 by Espressif**.
2. Board: your ESP32 dev board (e.g. "ESP32 Dev Module").
3. Open `esp32_mpu6050_test.ino`, upload.

## What you should see (Serial Monitor, 115200 baud)

```
=== ESP32 + MPU-6050 (GY-521) bring-up ===
I2C scan:
  0x68
WHO_AM_I = 0x68  (expect 0x68 or 0x72 on some clones)
Configured: 100 Hz, +/-8 g, +/-1000 dps
Streaming (ax ay az | gx gy gz | tempC):
a:  +0.01  +0.02  +1.00 g | g:   +0.3   -0.2   +0.1 dps | 30.4 C
```

Sanity checks:
- With the board sitting flat, one axis of `a` reads ~**+1.00 g**, the others ~0.
- Rotate 90°: the +1 g moves from `az` to `ax` or `ay`.
- Sitting still, gyro reads within a few dps of zero (the bias — normal).

## Common issues

- **`WHO_AM_I = 0xFF` or I²C scan empty:** SDA/SCL swapped, or the board is
  not powered. Check the pins and the 3V3 rail.
- **`WHO_AM_I = 0x72`:** you likely have an MPU-6500 or a clone. Same
  registers, sketch still works.
- **Accel reads all zeros:** the sensor is still in sleep. Power-cycle; the
  sketch clears the SLEEP bit in `PWR_MGMT_1` on setup.

## Next

Once this streams clean numbers, switch to `firmware/sensor_node/` — same
sensor, but samples at 100 Hz on the data-ready interrupt and sends the
raw counts to the receiver ESP32 over ESP-NOW.
