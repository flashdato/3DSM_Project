// esp32_sensor_now.ino
// Wireless IMU sensor node for the live box visualizer.
// Reads GY-87 MPU-6050 (accel + gyro only — mag ignored on this clone board)
// and broadcasts raw samples via ESP-NOW to the receiver.
//
// Pair with esp32_receiver_now.ino (plugged into the PC over USB).
// Python side: tools/visualize_box_gy87.py --port <receiver's TTY> --baud 1000000
//
// --- Design choices ---
//  * Broadcast peer (FF:FF:FF:FF:FF:FF) and fixed channel => no pairing
//    handshake, so a power-cycled sensor streams again the instant it boots.
//  * Up to several sensors can run in parallel; each gets a unique NODE_ID at
//    compile time and the receiver tags its output.
//  * SAMPLE_HZ * 12 B per sample fits comfortably in ESP-NOW's 250-byte MTU
//    with headroom for the 20-byte header. Batching keeps the packet rate
//    reasonable so radio contention stays low.
//  * No encryption, WiFi power-save disabled, max TX power.
//
// --- Wiring (same as esp32_visualize_gy87, 5V-edge of DevKit) ---
//   VCC_IN -> 5V/VIN
//   GND    -> GND
//   SDA    -> GPIO 26
//   SCL    -> GPIO 25

#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// ------------- Config -------------
static const int      PIN_SDA         = 26;
static const int      PIN_SCL         = 25;
static const uint8_t  MPU_ADDR        = 0x68;
static const uint8_t  ACC_FS_SEL      = 2;    // +/- 8 g
static const uint8_t  GYR_FS_SEL      = 2;    // +/- 1000 dps
static const uint16_t SAMPLE_HZ       = 200;  // IMU read rate
static const uint8_t  SAMPLES_PER_PKT = 4;    // => 50 Hz packet rate
static const uint8_t  WIFI_CHANNEL    = 1;
static const uint8_t  NODE_ID         = 1;    // change per sensor

static const uint8_t BCAST[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

// ------------- Packet format (little-endian, shared with receiver) -------------
#pragma pack(push, 1)
struct SampleRaw {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
};  // 12 B

struct Packet {
  uint16_t magic;           // 0x3D5D
  uint8_t  version;         // 2
  uint8_t  node_id;
  uint32_t pkt_seq;
  uint32_t t_us;            // micros() at first sample in batch
  uint16_t sample_hz;
  uint8_t  n_samples;
  uint8_t  acc_fs_sel;
  uint8_t  gyr_fs_sel;
  uint8_t  _pad[3];
  SampleRaw samples[SAMPLES_PER_PKT];
};
#pragma pack(pop)

static Packet   pkt;
static uint8_t  sample_idx = 0;
static uint32_t pkt_seq    = 0;
static uint32_t next_us    = 0;
static int16_t  biasGx = 0, biasGy = 0, biasGz = 0;

// ------------- I2C -------------
static void writeMPU(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg); Wire.write(val);
  Wire.endTransmission();
}

static bool readIMU(SampleRaw &s) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B);          // ACCEL_XOUT_H
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)14) != 14) return false;
  s.ax = (Wire.read() << 8) | Wire.read();
  s.ay = (Wire.read() << 8) | Wire.read();
  s.az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();                                     // temp
  s.gx = (Wire.read() << 8) | Wire.read();
  s.gy = (Wire.read() << 8) | Wire.read();
  s.gz = (Wire.read() << 8) | Wire.read();
  // Subtract gyro bias measured at boot. Accel is left raw — we rely on the
  // host-side mounting learn to cancel the accel-axis orientation.
  s.gx -= biasGx;
  s.gy -= biasGy;
  s.gz -= biasGz;
  return true;
}

// 3-sec stationary gyro bias calibration — must be run before readIMU() is
// used in the main loop so that the subtraction above is meaningful.
static void calibrateGyroBias(uint32_t duration_ms = 3000) {
  Serial.printf("# calibrating gyro bias (%u ms, hold STILL)...\n", duration_ms);
  delay(300);                                                   // let IMU settle
  uint32_t t0 = millis();
  double sx = 0, sy = 0, sz = 0;
  uint32_t n = 0;
  while (millis() - t0 < duration_ms) {
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x43);         // GYRO_XOUT_H
    if (Wire.endTransmission(false) != 0) { delay(2); continue; }
    if (Wire.requestFrom(MPU_ADDR, (uint8_t)6) != 6) { delay(2); continue; }
    int16_t gx = (Wire.read() << 8) | Wire.read();
    int16_t gy = (Wire.read() << 8) | Wire.read();
    int16_t gz = (Wire.read() << 8) | Wire.read();
    sx += gx; sy += gy; sz += gz; n++;
    delay(2);
  }
  if (n > 100) {
    biasGx = (int16_t)lroundf(sx / n);
    biasGy = (int16_t)lroundf(sy / n);
    biasGz = (int16_t)lroundf(sz / n);
  }
  const float lsb = 32.8f;                                      // +/- 1000 dps
  Serial.printf("# gyro bias: gx=%d gy=%d gz=%d counts (%.2f %.2f %.2f dps) n=%u\n",
                biasGx, biasGy, biasGz,
                biasGx / lsb, biasGy / lsb, biasGz / lsb, n);
}

// ------------- Setup -------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.printf("\n# esp32_sensor_now  node=%u  ch=%u  %u Hz  %u sample/pkt\n",
                NODE_ID, WIFI_CHANNEL, SAMPLE_HZ, SAMPLES_PER_PKT);

  // MPU-6050 bring-up (no mag/baro bypass needed — we don't use them).
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  writeMPU(0x6B, 0x01);                        // PWR_MGMT_1: wake, PLL gyro X
  delay(30);
  writeMPU(0x1A, 0x03);                        // CONFIG: DLPF ~44 Hz
  writeMPU(0x19, 0x00);                        // SMPLRT_DIV = 0 (1 kHz src)
  writeMPU(0x1B, (GYR_FS_SEL & 3) << 3);       // GYRO_CONFIG
  writeMPU(0x1C, (ACC_FS_SEL & 3) << 3);       // ACCEL_CONFIG
  writeMPU(0x37, 0x00);                        // INT_PIN_CFG
  writeMPU(0x6A, 0x00);                        // USER_CTRL

  // WiFi / ESP-NOW
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_max_tx_power(84);               // ~21 dBm

  if (esp_now_init() != ESP_OK) {
    Serial.println("# ESP-NOW init failed, halt");
    while (1) delay(1000);
  }
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BCAST, 6);
  peer.channel = WIFI_CHANNEL;
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  pkt.magic      = 0x3D5D;
  pkt.version    = 2;
  pkt.node_id    = NODE_ID;
  pkt.sample_hz  = SAMPLE_HZ;
  pkt.n_samples  = SAMPLES_PER_PKT;
  pkt.acc_fs_sel = ACC_FS_SEL;
  pkt.gyr_fs_sel = GYR_FS_SEL;

  Serial.printf("# own MAC: %s   pkt size %u B\n",
                WiFi.macAddress().c_str(), (unsigned)sizeof(Packet));

  calibrateGyroBias(3000);

  next_us = micros();
}

// ------------- Loop -------------
void loop() {
  const uint32_t now_us = micros();
  if ((int32_t)(now_us - next_us) < 0) return;
  next_us += 1000000UL / SAMPLE_HZ;

  if (sample_idx == 0) {
    pkt.t_us    = now_us;
    pkt.pkt_seq = pkt_seq++;
  }
  if (!readIMU(pkt.samples[sample_idx])) return;
  sample_idx++;

  if (sample_idx >= SAMPLES_PER_PKT) {
    esp_now_send(BCAST, (uint8_t*)&pkt, sizeof(pkt));
    sample_idx = 0;
  }
}
