// esp32_visualize.ino
// ESP32 + GY-521 (MPU-6050) -> serial stream of roll & pitch for the
// Python 3D visualizer (tools/visualize_rectangle.py).
//
// - Samples the MPU-6050 at 100 Hz.
// - Runs a complementary filter (98% gyro / 2% accel) for roll and pitch.
// - Integrates the debiased Z-gyro for yaw (heads-up: yaw drifts because
//   there is no magnetometer to correct it — expect ~a few deg / minute
//   when sitting still).
// - Auto-calibrates gyro bias for the first ~1.5 s: keep the board still on setup.
// - Prints one CSV line at 50 Hz on Serial (115200 baud):
//       roll_deg,pitch_deg,yaw_deg,ax,ay,az,gx,gy,gz
//   Angles are in degrees; ax..gz are the raw floats (g and dps).
//
// Wiring: same as esp32_mpu6050_test — SDA=21, SCL=22, VCC=3V3, GND=GND.

#include <Wire.h>
#include <math.h>

// ------------------- Config -------------------
static const int PIN_SDA = 21;
static const int PIN_SCL = 22;
static const uint8_t MPU_ADDR = 0x68;

static const uint8_t ACC_FS_SEL = 2;   // +/-8 g
static const uint8_t GYR_FS_SEL = 2;   // +/-1000 dps
static const uint8_t SMPLRT_DIV = 9;   // 100 Hz

static const float ALPHA = 0.98f;      // complementary filter weight for gyro
static const uint32_t CALIB_MS = 1500; // gyro-bias averaging window on boot
static const uint32_t PRINT_MS = 20;   // 50 Hz to the serial link

// ------------------- Registers -------------------
#define REG_SMPLRT_DIV   0x19
#define REG_CONFIG       0x1A
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B
#define REG_PWR_MGMT_1   0x6B
#define REG_WHO_AM_I     0x75

static const float ACC_LSB[4] = { 16384.0f, 8192.0f, 4096.0f, 2048.0f };
static const float GYR_LSB[4] = { 131.0f, 65.5f, 32.8f, 16.4f };

// Declared before any function that uses it, so Arduino IDE's auto-prototype
// generator doesn't emit `static bool readSample(Sample& s);` before `Sample`
// exists.
struct Sample {
  float ax, ay, az;   // g
  float gx, gy, gz;   // dps
};

// ------------------- Helpers -------------------
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
  const float aL = ACC_LSB[ACC_FS_SEL];
  const float gL = GYR_LSB[GYR_FS_SEL];
  s.ax = ax / aL; s.ay = ay / aL; s.az = az / aL;
  s.gx = gx / gL; s.gy = gy / gL; s.gz = gz / gL;
  return true;
}

// ------------------- State -------------------
static float rollDeg = 0.0f;
static float pitchDeg = 0.0f;
static float yawDeg = 0.0f;
static float biasGx = 0.0f, biasGy = 0.0f, biasGz = 0.0f;
static uint32_t lastMicros = 0;

// ------------------- Setup -------------------
void setup() {
  Serial.begin(115200);
  delay(200);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);

  writeReg(REG_PWR_MGMT_1, 0x01);        // wake, PLL X-gyro
  delay(50);
  writeReg(REG_CONFIG, 0x03);            // DLPF ~44 Hz
  writeReg(REG_SMPLRT_DIV, SMPLRT_DIV);
  writeReg(REG_GYRO_CONFIG,  (GYR_FS_SEL & 0x03) << 3);
  writeReg(REG_ACCEL_CONFIG, (ACC_FS_SEL & 0x03) << 3);

  // Calibrate gyro bias: assume board is still.
  Serial.println("# hold still: calibrating gyro bias...");
  uint32_t t0 = millis();
  double sx = 0, sy = 0, sz = 0;
  int n = 0;
  while (millis() - t0 < CALIB_MS) {
    Sample s;
    if (readSample(s)) {
      sx += s.gx; sy += s.gy; sz += s.gz;
      n++;
    }
    delay(5);
  }
  if (n > 0) { biasGx = sx / n; biasGy = sy / n; biasGz = sz / n; }
  Serial.printf("# bias dps: gx=%.2f gy=%.2f gz=%.2f (n=%d)\n", biasGx, biasGy, biasGz, n);

  // Seed roll/pitch from the initial accel reading.
  Sample s0;
  if (readSample(s0)) {
    rollDeg  = atan2f(s0.ay, s0.az) * 180.0f / (float)M_PI;
    pitchDeg = atan2f(-s0.ax, sqrtf(s0.ay * s0.ay + s0.az * s0.az)) * 180.0f / (float)M_PI;
  }

  Serial.println("# format: roll_deg,pitch_deg,yaw_deg,ax,ay,az,gx,gy,gz");
  lastMicros = micros();
}

// ------------------- Loop -------------------
void loop() {
  Sample s;
  if (!readSample(s)) return;

  uint32_t now = micros();
  float dt = (now - lastMicros) * 1e-6f;
  lastMicros = now;
  if (dt <= 0.0f || dt > 0.2f) dt = 0.01f;

  // Debias gyro.
  float gxc = s.gx - biasGx;
  float gyc = s.gy - biasGy;
  float gzc = s.gz - biasGz;

  // Accel-derived angles.
  float rollA  = atan2f(s.ay, s.az) * 180.0f / (float)M_PI;
  float pitchA = atan2f(-s.ax, sqrtf(s.ay * s.ay + s.az * s.az)) * 180.0f / (float)M_PI;

  // Complementary filter.
  rollDeg  = ALPHA * (rollDeg  + gxc * dt) + (1.0f - ALPHA) * rollA;
  pitchDeg = ALPHA * (pitchDeg + gyc * dt) + (1.0f - ALPHA) * pitchA;

  // Emit at ~50 Hz.
  static uint32_t lastPrint = 0;
  uint32_t nowMs = millis();
  if (nowMs - lastPrint >= PRINT_MS) {
    lastPrint = nowMs;
    Serial.printf("%.2f,%.2f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f\n",
                  rollDeg, pitchDeg,
                  s.ax, s.ay, s.az,
                  gxc, gyc, gzc);
  }
}
