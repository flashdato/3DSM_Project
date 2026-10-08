// esp32_visualize_wifi.ino
// Same MPU-6050 sampling + complementary filter as esp32_visualize.ino, but
// streams the CSV over WiFi UDP (broadcast) instead of USB serial.
//
// 1) Fill in WIFI_SSID and WIFI_PASS below and flash.
// 2) Open Serial Monitor (115200) once to see the sensor calibrate and the
//    ESP32 print its IP + "UDP broadcasting on port 5005".
// 3) Close Serial Monitor. On the laptop (same WiFi network):
//        python tools/visualize_arm.py --udp-port 5005
//    No serial port needed — the ESP32 can even be powered from a USB battery.
//
// Wiring: same as before — MPU-6050 on SDA=21, SCL=22.

#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <math.h>

// ================= WiFi config =================
#define WIFI_SSID   "TP-Link_7DFA"
#define WIFI_PASS   "arduinonano"
#define UDP_PORT    5005
// Leave BROADCAST_MODE=1 for LAN broadcast (no laptop IP needed). If the
// router blocks broadcasts, set BROADCAST_MODE=0 and put the laptop IP below.
#define BROADCAST_MODE 1
static const IPAddress LAPTOP_IP(192, 168, 1, 100);   // used only if BROADCAST_MODE=0

// ================= MPU-6050 config =================
static const int PIN_SDA = 21;
static const int PIN_SCL = 22;
static const uint8_t MPU_ADDR = 0x68;

static const uint8_t ACC_FS_SEL = 2;
static const uint8_t GYR_FS_SEL = 2;
static const uint8_t SMPLRT_DIV = 9;      // 100 Hz

static const float ALPHA = 0.98f;
static const uint32_t CALIB_MS = 1500;
static const uint32_t SEND_MS = 10;       // 100 Hz on the wire

// ================= Registers =================
#define REG_SMPLRT_DIV   0x19
#define REG_CONFIG       0x1A
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B
#define REG_PWR_MGMT_1   0x6B

static const float ACC_LSB[4] = { 16384.0f, 8192.0f, 4096.0f, 2048.0f };
static const float GYR_LSB[4] = { 131.0f, 65.5f, 32.8f, 16.4f };

struct Sample { float ax, ay, az, gx, gy, gz; };

// ================= I2C helpers =================
static void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg); Wire.write(val);
  Wire.endTransmission();
}

static bool readBlock(uint8_t reg, uint8_t* buf, size_t n) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  size_t got = Wire.requestFrom((int)MPU_ADDR, (int)n);
  if (got != n) return false;
  for (size_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

static bool readSample(Sample& s) {
  uint8_t b[14];
  if (!readBlock(REG_ACCEL_XOUT_H, b, 14)) return false;
  int16_t ax = (b[0] << 8) | b[1];
  int16_t ay = (b[2] << 8) | b[3];
  int16_t az = (b[4] << 8) | b[5];
  int16_t gx = (b[8] << 8) | b[9];
  int16_t gy = (b[10] << 8) | b[11];
  int16_t gz = (b[12] << 8) | b[13];
  s.ax = ax / ACC_LSB[ACC_FS_SEL];
  s.ay = ay / ACC_LSB[ACC_FS_SEL];
  s.az = az / ACC_LSB[ACC_FS_SEL];
  s.gx = gx / GYR_LSB[GYR_FS_SEL];
  s.gy = gy / GYR_LSB[GYR_FS_SEL];
  s.gz = gz / GYR_LSB[GYR_FS_SEL];
  return true;
}

// ================= State =================
static float rollDeg = 0.0f, pitchDeg = 0.0f;
static float biasGx = 0.0f, biasGy = 0.0f, biasGz = 0.0f;
static uint32_t lastMicros = 0;

static WiFiUDP udp;
static IPAddress dstIp;

// ================= Setup =================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== esp32_visualize_wifi ===");

  // --- WiFi ---
  Serial.printf("Connecting to WiFi \"%s\" ...\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);          // biggest latency win: no beacon-driven pauses.
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
    delay(250);
    Serial.print('.');
  }
  Serial.println();
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi failed. Check WIFI_SSID / WIFI_PASS and reset.");
    while (true) delay(1000);
  }
  Serial.printf("WiFi OK. IP=%s  RSSI=%d dBm\n",
                WiFi.localIP().toString().c_str(), WiFi.RSSI());
  if (BROADCAST_MODE) {
    dstIp = WiFi.broadcastIP();
    Serial.printf("UDP broadcasting to %s:%d\n", dstIp.toString().c_str(), UDP_PORT);
  } else {
    dstIp = LAPTOP_IP;
    Serial.printf("UDP unicast to %s:%d\n", dstIp.toString().c_str(), UDP_PORT);
  }
  udp.begin(UDP_PORT);

  // --- MPU-6050 ---
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  writeReg(REG_PWR_MGMT_1, 0x01);
  delay(50);
  writeReg(REG_CONFIG, 0x03);
  writeReg(REG_SMPLRT_DIV, SMPLRT_DIV);
  writeReg(REG_GYRO_CONFIG,  (GYR_FS_SEL & 0x03) << 3);
  writeReg(REG_ACCEL_CONFIG, (ACC_FS_SEL & 0x03) << 3);

  Serial.println("# hold still: calibrating gyro bias...");
  uint32_t tc = millis();
  double sx = 0, sy = 0, sz = 0;
  int n = 0;
  while (millis() - tc < CALIB_MS) {
    Sample s;
    if (readSample(s)) { sx += s.gx; sy += s.gy; sz += s.gz; n++; }
    delay(5);
  }
  if (n > 0) { biasGx = sx / n; biasGy = sy / n; biasGz = sz / n; }
  Serial.printf("# bias dps: gx=%.2f gy=%.2f gz=%.2f (n=%d)\n", biasGx, biasGy, biasGz, n);

  Sample s0;
  if (readSample(s0)) {
    rollDeg  = atan2f(s0.ay, s0.az) * 180.0f / (float)M_PI;
    pitchDeg = atan2f(-s0.ax, sqrtf(s0.ay * s0.ay + s0.az * s0.az)) * 180.0f / (float)M_PI;
  }
  lastMicros = micros();
  Serial.println("# streaming over UDP.");
}

// ================= Loop =================
void loop() {
  Sample s;
  if (!readSample(s)) return;

  uint32_t now = micros();
  float dt = (now - lastMicros) * 1e-6f;
  lastMicros = now;
  if (dt <= 0.0f || dt > 0.2f) dt = 0.01f;

  float gxc = s.gx - biasGx;
  float gyc = s.gy - biasGy;

  float rollA  = atan2f(s.ay, s.az) * 180.0f / (float)M_PI;
  float pitchA = atan2f(-s.ax, sqrtf(s.ay * s.ay + s.az * s.az)) * 180.0f / (float)M_PI;
  rollDeg  = ALPHA * (rollDeg  + gxc * dt) + (1.0f - ALPHA) * rollA;
  pitchDeg = ALPHA * (pitchDeg + gyc * dt) + (1.0f - ALPHA) * pitchA;

  static uint32_t lastSend = 0;
  uint32_t nowMs = millis();
  if (nowMs - lastSend >= SEND_MS) {
    lastSend = nowMs;
    char buf[128];
    int n = snprintf(buf, sizeof(buf),
                     "%.2f,%.2f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f\n",
                     rollDeg, pitchDeg, s.ax, s.ay, s.az, gxc, gyc, s.gz - biasGz);
    if (n > 0) {
      udp.beginPacket(dstIp, UDP_PORT);
      udp.write((uint8_t*)buf, n);
      udp.endPacket();
    }
  }
}
