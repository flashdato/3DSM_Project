// 3DSM sensor node: ESP32 + GY-87 (MPU-6050) worn on one body segment.
//
// - Samples the MPU-6050 at SAMPLE_HZ, timestamped at the data-ready interrupt.
// - Syncs its clock to the receiver's beacons (see HARDWARE.md, "Time sync").
// - Learns the receiver's MAC from the first beacon (no hard-coded addresses).
// - Sends raw accel/gyro counts in batches of BATCH samples over ESP-NOW.
//
// Build one binary per node: change NODE_ID and SEGMENT_ID below.
// Tested target: Arduino-ESP32 core 3.x (also compiles on 2.x).

#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_timer.h>
#include "imu_protocol.h"

// ======================= per-node configuration =======================
#define NODE_ID       1                 // 1 = right upper arm, 2 = right forearm
#define SEGMENT_ID    SEG_R_UPPER_ARM   // SEG_R_FOREARM for node 2

#define SAMPLE_HZ     100   // must divide 1000: 100, 125, 200, 250, 500
#define BATCH         2     // samples per ESP-NOW packet (1..MAX_SAMPLES_PER_PKT)
#define ACC_FS_G      8     // 2, 4, 8, 16
#define GYR_FS_DPS    1000  // 250, 500, 1000, 2000

// Pins: defaults for a classic ESP32 DevKit. Change for other boards. For example:
//   ESP32-S3: GPIO 22 doesn't exist and 19/20 are the USB pins, so try SDA 8, SCL 9, INT 7.
//   ESP32-C3: avoid strapping pins 2/8/9, so try SDA 6, SCL 7, INT 5.
// Any free GPIOs work; check your board's pinout.
#define PIN_SDA       21
#define PIN_SCL       22
#define PIN_MPU_INT   19    // GY-87 "INTA" -> this pin. Set USE_DRDY_INT 0 if not wired.
#define USE_DRDY_INT  1
#define PIN_BATT      -1    // ADC pin behind a 1:2 divider, -1 = not fitted
#define PIN_LED       2     // on-board LED, -1 = none

#define SYNC_TIMEOUT_US  10000000LL  // clear FLAG_SYNCED if no beacon for 10 s
// ======================================================================

static_assert(1000 % SAMPLE_HZ == 0, "SAMPLE_HZ must divide 1000");
static_assert(BATCH >= 1 && BATCH <= MAX_SAMPLES_PER_PKT, "bad BATCH");

// ---------------- MPU-6050 registers ----------------
static const uint8_t MPU_ADDR        = 0x68;
static const uint8_t REG_SMPLRT_DIV  = 0x19;
static const uint8_t REG_CONFIG      = 0x1A;
static const uint8_t REG_GYRO_CFG    = 0x1B;
static const uint8_t REG_ACCEL_CFG   = 0x1C;
static const uint8_t REG_INT_PIN_CFG = 0x37;
static const uint8_t REG_INT_ENABLE  = 0x38;
static const uint8_t REG_INT_STATUS  = 0x3A;
static const uint8_t REG_ACCEL_XOUT  = 0x3B;  // 14 bytes: accel(6) temp(2) gyro(6)
static const uint8_t REG_PWR_MGMT_1  = 0x6B;
static const uint8_t REG_WHO_AM_I    = 0x75;

static bool mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool mpuRead(uint8_t reg, uint8_t *buf, size_t len) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU_ADDR, (int)len) != (int)len) return false;
  for (size_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

static uint8_t fsBits(int value, const int *table) {
  for (uint8_t i = 0; i < 4; i++) if (table[i] == value) return i;
  return 0;
}

static bool mpuInit() {
  uint8_t who = 0;
  if (!mpuRead(REG_WHO_AM_I, &who, 1)) return false;
  Serial.printf("MPU WHO_AM_I = 0x%02X %s\n", who,
                who == 0x68 ? "(MPU-6050)" : "(not 0x68: clone or other chip, continuing)");

  static const int accTable[4] = {2, 4, 8, 16};
  static const int gyrTable[4] = {250, 500, 1000, 2000};

  bool ok = true;
  ok &= mpuWrite(REG_PWR_MGMT_1, 0x01);                      // wake, clock = PLL on gyro X
  delay(50);
  ok &= mpuWrite(REG_CONFIG, 0x03);                          // DLPF: accel 44 Hz / gyro 42 Hz, 1 kHz base rate
  ok &= mpuWrite(REG_SMPLRT_DIV, (1000 / SAMPLE_HZ) - 1);    // sample rate = 1 kHz / (1 + div)
  ok &= mpuWrite(REG_GYRO_CFG,  fsBits(GYR_FS_DPS, gyrTable) << 3);
  ok &= mpuWrite(REG_ACCEL_CFG, fsBits(ACC_FS_G,   accTable) << 3);
  ok &= mpuWrite(REG_INT_PIN_CFG, 0x02);                     // active-high 50 us pulse, I2C bypass ON (mag reachable)
  ok &= mpuWrite(REG_INT_ENABLE, 0x01);                      // data-ready interrupt
  return ok;
}

static void i2cScan() {
  Serial.print("I2C devices:");
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) Serial.printf(" 0x%02X", a);
  }
  // GY-87: 0x68 MPU-6050, 0x77 BMP180, 0x1E HMC5883L or 0x0D QMC5883L (visible thanks to bypass)
  Serial.println();
}

// ---------------- data-ready timestamping ----------------
static portMUX_TYPE g_isrMux = portMUX_INITIALIZER_UNLOCKED;
static volatile int64_t  g_drdyTime  = 0;
static volatile uint32_t g_drdyCount = 0;

static void IRAM_ATTR onDataReady() {
  int64_t t = esp_timer_get_time();
  portENTER_CRITICAL_ISR(&g_isrMux);
  g_drdyTime = t;
  g_drdyCount = g_drdyCount + 1;
  portEXIT_CRITICAL_ISR(&g_isrMux);
}

// ---------------- clock sync state (written by the ESP-NOW callback) ----------------
static portMUX_TYPE g_syncMux = portMUX_INITIALIZER_UNLOCKED;
static int64_t  g_offsetUs      = 0;      // receiver_time = node_time + offset
static int64_t  g_lastBeaconUs  = 0;      // node time of the last beacon
static bool     g_everSynced    = false;
static uint32_t g_beaconCount   = 0;

static uint8_t       g_rxMac[6];
static volatile bool g_peerPending = false;
static bool          g_havePeer    = false;

static void handleBeacon(const uint8_t *mac, const uint8_t *data, int len) {
  int64_t tLocal = esp_timer_get_time();          // take the timestamp first
  if (len != (int)sizeof(beacon_pkt_t)) return;
  beacon_pkt_t b;
  memcpy(&b, data, sizeof(b));
  if (b.magic != IMU_PROTO_MAGIC || b.version != IMU_PROTO_VERSION || b.type != PKT_BEACON) return;

  int64_t newOffset = (int64_t)b.rx_time_us - tLocal;
  portENTER_CRITICAL(&g_syncMux);
  int64_t diff = newOffset - g_offsetUs;
  if (!g_everSynced || diff > 2000 || diff < -2000) {
    g_offsetUs = newOffset;                         // first beacon, or receiver rebooted: jump
  } else {
    g_offsetUs += diff / 4;                         // otherwise smooth out beacon jitter
  }
  g_lastBeaconUs = tLocal;
  g_everSynced = true;
  g_beaconCount++;
  portEXIT_CRITICAL(&g_syncMux);

  if (!g_havePeer && !g_peerPending) {
    memcpy(g_rxMac, mac, 6);
    g_peerPending = true;                            // add the peer from loop(), not from the Wi-Fi task
  }
}

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
static void onEspNowRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  handleBeacon(info->src_addr, data, len);
}
#else
static void onEspNowRecv(const uint8_t *mac, const uint8_t *data, int len) {
  handleBeacon(mac, data, len);
}
#endif

// ---------------- helpers ----------------
static void setLed(bool on) {
  if (PIN_LED >= 0) digitalWrite(PIN_LED, on ? HIGH : LOW);
}

static uint16_t readBatteryMv() {
  if (PIN_BATT < 0) return 0;
  return (uint16_t)(analogReadMilliVolts(PIN_BATT) * 2);   // 1:2 divider
}

static void addReceiverPeer() {
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, g_rxMac, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.encrypt = false;
  peer.ifidx = WIFI_IF_STA;
  if (esp_now_add_peer(&peer) == ESP_OK) {
    g_havePeer = true;
    Serial.printf("Receiver found: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  g_rxMac[0], g_rxMac[1], g_rxMac[2], g_rxMac[3], g_rxMac[4], g_rxMac[5]);
  }
  g_peerPending = false;
}

// ---------------- state for batching / stats ----------------
static data_pkt_t g_pkt;
static uint8_t    g_nInPkt   = 0;
static uint8_t    g_pktFlags = 0;
static bool       g_batchSynced = false;
static uint16_t   g_pktSeq   = 0;
static uint16_t   g_sampleSeq = 0;
static uint32_t   g_lastCount = 0;
static uint16_t   g_battMv    = 0;

static uint32_t st_samples = 0, st_sent = 0, st_sendFail = 0, st_overruns = 0, st_i2cErr = 0;

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.printf("\n3DSM sensor node %d (segment %d), %d Hz, batch %d\n", NODE_ID, SEGMENT_ID, SAMPLE_HZ, BATCH);

  if (PIN_LED >= 0) pinMode(PIN_LED, OUTPUT);

  Wire.begin(PIN_SDA, PIN_SCL, 400000);
  i2cScan();
  while (!mpuInit()) {
    Serial.println("MPU-6050 not responding, check wiring. Retrying...");
    delay(1000);
  }

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_ps(WIFI_PS_NONE);                     // lowest latency (costs some battery)
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  if (esp_now_init() != ESP_OK) {
    Serial.println("esp_now_init failed");
    while (true) delay(1000);
  }
  esp_now_register_recv_cb(onEspNowRecv);
  Serial.printf("Node MAC %s, waiting for receiver beacon on channel %d\n",
                WiFi.macAddress().c_str(), ESPNOW_CHANNEL);

#if USE_DRDY_INT
  pinMode(PIN_MPU_INT, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_MPU_INT), onDataReady, RISING);
#endif

  memset(&g_pkt, 0, sizeof(g_pkt));
}

// Returns true (and the event time) when a new sample is ready.
static bool sampleReady(int64_t &tSample, uint32_t &missed) {
#if USE_DRDY_INT
  portENTER_CRITICAL(&g_isrMux);
  uint32_t count = g_drdyCount;
  int64_t  t     = g_drdyTime;
  portEXIT_CRITICAL(&g_isrMux);
  if (count == g_lastCount) return false;
  missed = count - g_lastCount - 1;
  g_lastCount = count;
  tSample = t;
  return true;
#else
  uint8_t st = 0;
  if (!mpuRead(REG_INT_STATUS, &st, 1) || !(st & 0x01)) return false;
  tSample = esp_timer_get_time();    // polling: timestamp jitter up to one loop pass
  missed = 0;
  return true;
#endif
}

void loop() {
  if (g_peerPending) addReceiverPeer();

  int64_t tLocal;
  uint32_t missed;
  if (sampleReady(tLocal, missed)) {
    uint8_t raw[14];
    if (!mpuRead(REG_ACCEL_XOUT, raw, sizeof(raw))) {
      st_i2cErr++;
      return;
    }
    if (missed) { st_overruns += missed; g_pktFlags |= FLAG_OVERRUN; }
    st_samples++;

    // Convert node time to receiver time. The sync state is fixed per packet so
    // one packet never mixes node-clock and receiver-clock timestamps.
    portENTER_CRITICAL(&g_syncMux);
    int64_t offset = g_offsetUs;
    bool synced = g_everSynced && (tLocal - g_lastBeaconUs) < SYNC_TIMEOUT_US;
    portEXIT_CRITICAL(&g_syncMux);
    if (g_nInPkt == 0) g_batchSynced = synced;
    int64_t tOut = g_batchSynced ? tLocal + offset : tLocal;

    imu_sample_t &s = g_pkt.s[g_nInPkt];
    s.t_us   = (uint32_t)tOut;
    s.seq    = g_sampleSeq++;
    s.acc[0] = (int16_t)(raw[0] << 8 | raw[1]);
    s.acc[1] = (int16_t)(raw[2] << 8 | raw[3]);
    s.acc[2] = (int16_t)(raw[4] << 8 | raw[5]);
    s.temp   = (int16_t)(raw[6] << 8 | raw[7]);
    s.gyr[0] = (int16_t)(raw[8] << 8 | raw[9]);
    s.gyr[1] = (int16_t)(raw[10] << 8 | raw[11]);
    s.gyr[2] = (int16_t)(raw[12] << 8 | raw[13]);
    s.mag[0] = s.mag[1] = s.mag[2] = 0;             // magnetometer comes later (optional channel)

    if (g_batchSynced) g_pktFlags |= FLAG_SYNCED;
    g_nInPkt++;

    if (g_nInPkt >= BATCH) {
      if (g_havePeer) {
        g_pkt.magic      = IMU_PROTO_MAGIC;
        g_pkt.version    = IMU_PROTO_VERSION;
        g_pkt.type       = PKT_DATA;
        g_pkt.node_id    = NODE_ID;
        g_pkt.segment_id = SEGMENT_ID;
        g_pkt.n          = g_nInPkt;
        g_pkt.flags      = g_pktFlags;
        g_pkt.pkt_seq    = g_pktSeq++;
        g_pkt.batt_mv    = g_battMv;
        g_pkt.sample_hz  = SAMPLE_HZ;
        g_pkt.acc_fs_g   = ACC_FS_G;
        g_pkt.gyr_fs_dps = GYR_FS_DPS;
        size_t len = DATA_PKT_HEADER_BYTES + sizeof(imu_sample_t) * g_nInPkt;
        if (esp_now_send(g_rxMac, (const uint8_t *)&g_pkt, len) == ESP_OK) st_sent++;
        else st_sendFail++;
      }
      g_nInPkt = 0;
      g_pktFlags = 0;
    }
  }

  // Once per second: battery, LED, status line.
  static uint32_t lastStatusMs = 0;
  uint32_t nowMs = millis();
  if (nowMs - lastStatusMs >= 1000) {
    lastStatusMs = nowMs;
    g_battMv = readBatteryMv();
    portENTER_CRITICAL(&g_syncMux);
    int64_t offset = g_offsetUs;
    uint32_t beacons = g_beaconCount;
    portEXIT_CRITICAL(&g_syncMux);
    Serial.printf("samples/s=%lu sent=%lu fail=%lu overruns=%lu i2cErr=%lu beacons=%lu offset_us=%lld peer=%d\n",
                  (unsigned long)st_samples, (unsigned long)st_sent, (unsigned long)st_sendFail,
                  (unsigned long)st_overruns, (unsigned long)st_i2cErr, (unsigned long)beacons,
                  (long long)offset, g_havePeer);
    st_samples = st_sent = st_sendFail = 0;
  }

  // LED: solid when paired and synced, blinking while searching for the receiver.
  if (g_havePeer && g_everSynced) setLed(true);
  else setLed((nowMs / 250) % 2);
}
