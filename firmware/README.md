# Firmware

Two Arduino sketches for the wireless IMU nodes. The design, rates and time-sync
protocol are explained in [`../HARDWARE.md`](../HARDWARE.md).

| Sketch | Runs on | Job |
|--------|---------|-----|
| [`sensor_node/`](sensor_node/) | each worn ESP32 + GY-87 | sample MPU-6050, sync clock to beacons, send raw samples over ESP-NOW |
| [`receiver/`](receiver/) | one ESP32 on the laptop's USB | send time beacons, receive all nodes, print CSV over serial |

`imu_protocol.h` exists in both folders and **must stay identical**. Arduino can't
include headers from outside the sketch folder.

## Flashing

1. Arduino IDE 2 (or `arduino-cli`) with the **esp32 by Espressif** board package, core 3.x recommended (2.x also compiles).
2. **Receiver:** open `receiver/receiver.ino`, pick your board, upload.
3. **Node 1:** open `sensor_node/sensor_node.ino` and set:
   ```c
   #define NODE_ID     1
   #define SEGMENT_ID  SEG_R_UPPER_ARM
   ```
   Check `PIN_SDA`, `PIN_SCL`, `PIN_MPU_INT` and `PIN_LED` match your board, then upload.
4. **Node 2:** same sketch with `NODE_ID 2` and `SEG_R_FOREARM`, then upload.
5. Power everything. A node's LED blinks while it searches for the receiver and goes solid once it has received a beacon (paired + synced).

No MAC addresses need to be typed in: nodes learn the receiver's MAC from its beacon.

## Checking it works

- **Node serial monitor (115200 baud):** the I²C scan, `WHO_AM_I`, then once per second `samples/s=100 sent=50 fail=0 ... beacons=N offset_us=...`.
- **Receiver:** use the logger rather than a serial monitor, since it prints ~200 lines/s:
  ```bash
  pip install -r requirements.txt
  python tools/log_serial.py --port /dev/ttyUSB0      # Windows: COM5, macOS: /dev/cu.usbserial-*
  ```
  Expect `S` lines like `node 1: 50.0 pkt/s, 100.0 samples/s, lost 0, rssi -45 dBm, latency 11.2 ms, synced`.

## Settings worth knowing (top of `sensor_node.ino`)

| Constant | Default | Notes |
|----------|---------|-------|
| `SAMPLE_HZ` | 100 | must divide 1000 (100, 125, 200, 250, 500) |
| `BATCH` | 2 | samples per packet. Higher = fewer packets, more latency. |
| `ACC_FS_G` / `GYR_FS_DPS` | 8 / 1000 | measurement ranges |
| `USE_DRDY_INT` | 1 | 0 = poll the MPU instead of using the INTA pin |
| `ESPNOW_CHANNEL` | 1 | in `imu_protocol.h`; must match on every board |
