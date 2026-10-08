# esp32_visualize_wifi

Wireless version of `esp32_visualize`. Streams roll/pitch over **WiFi UDP**
instead of USB serial. Same MPU-6050 wiring (SDA=21, SCL=22).

## Setup

1. Open `esp32_visualize_wifi.ino`. At the top, edit:
   ```c
   #define WIFI_SSID   "your-wifi"
   #define WIFI_PASS   "your-password"
   ```
2. Upload. Open the Serial Monitor (115200) *just to see the ESP32 connect*.
   You should see something like:
   ```
   WiFi OK. IP=192.168.1.42  RSSI=-45 dBm
   UDP broadcasting to 192.168.1.255:5005
   # bias dps: gx=... gy=... gz=...
   # streaming over UDP.
   ```
3. Close the Serial Monitor. On the laptop (same WiFi network):
   ```bash
   python tools/visualize_arm.py --udp-port 5005
   ```
4. Unplug USB, plug into a phone charger / USB battery / anything with 5 V —
   the ESP32 keeps sending. That's it, no cable to the laptop.

## If nothing arrives

- Check the ESP32 IP is on the same subnet as the laptop (compare `ip addr`
  on the laptop with the printed ESP32 IP).
- Some routers drop broadcasts to `x.x.x.255`. In that case set
  `#define BROADCAST_MODE 0` and put the laptop IP in `LAPTOP_IP`.
- Firewall on the laptop: allow UDP 5005 in. On Arch with `iptables`/`nftables`
  or with `ufw allow 5005/udp`.

## What it sends

Same CSV as the wired sketch, one line per packet, 50 Hz:

```
roll_deg,pitch_deg,ax,ay,az,gx,gy,gz
```

The Python tool accepts either serial or UDP; the parser is shared.
