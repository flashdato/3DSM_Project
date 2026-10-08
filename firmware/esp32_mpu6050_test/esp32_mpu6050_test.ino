// esp32_mpu6050_test.ino
// Simple bring-up sketch for ESP32 + GY-521 (MPU-6050).
// Prints ax, ay, az [g], gx, gy, gz [deg/s], temp [C] on the Serial Monitor.
//
// Wiring (ESP32 DevKit v1 defaults):
//   GY-521 VCC -> 3V3   (5V also works on most GY-521 boards, they have a regulator)
//   GY-521 GND -> GND
//   GY-521 SCL -> GPIO 22
//   GY-521 SDA -> GPIO 21
//   (AD0 left floating -> I2C address 0x68; tie to 3V3 for 0x69)
//
// Open Serial Monitor at 115200 baud.

#include <Wire.h>

// ------------------- Pins & config -------------------
static const int PIN_SDA    = 21;
static const int PIN_SCL    = 22;
static const uint8_t MPU_ADDR = 0x68;

// Full-scale ranges (match HARDWARE.md defaults).
// ACC:  0=+/-2g, 1=+/-4g, 2=+/-8g, 3=+/-16g
// GYR:  0=+/-250, 1=+/-500, 2=+/-1000, 3=+/-2000 dps
static const uint8_t ACC_FS_SEL = 2;   // +/-8 g
static const uint8_t GYR_FS_SEL = 2;   // +/-1000 dps

// Sample-rate divider: rate = 1000 Hz / (1 + SMPLRT_DIV) with DLPF on.
// SMPLRT_DIV=9  -> 100 Hz
static const uint8_t SMPLRT_DIV = 9;

// Print interval (ms). Serial can't keep up with 100 Hz cleanly at 115200,
// so we sample fast but print every 20 ms (~50 Hz to the console).
static const uint32_t PRINT_MS = 20;

// ------------------- MPU-6050 registers -------------------
#define REG_SMPLRT_DIV   0x19
#define REG_CONFIG       0x1A
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B
#define REG_PWR_MGMT_1   0x6B
#define REG_WHO_AM_I     0x75

// LSB sensitivities (from MPU-6050 datasheet)
static const float ACC_LSB[4] = { 16384.0f, 8192.0f, 4096.0f, 2048.0f }; // counts per g
static const float GYR_LSB[4] = { 131.0f, 65.5f, 32.8f, 16.4f };         // counts per dps

// ------------------- Helpers -------------------
static void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

static uint8_t readReg(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((int)MPU_ADDR, 1);
  return Wire.read();
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

static void i2cScan() {
  Serial.println("I2C scan:");
  int found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  0x%02X\n", a);
      found++;
    }
  }
  if (!found) Serial.println("  (nothing found)");
}

// ------------------- Setup -------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("=== ESP32 + MPU-6050 (GY-521) bring-up ===");

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);

  i2cScan();

  uint8_t who = readReg(REG_WHO_AM_I);
  Serial.printf("WHO_AM_I = 0x%02X  (expect 0x68 or 0x72 on some clones)\n", who);

  // Wake up: clear SLEEP bit, use PLL with X-gyro as clock source.
  writeReg(REG_PWR_MGMT_1, 0x01);
  delay(50);

  // DLPF ~44 Hz  (CONFIG = 3): keeps signal clean and locks output rate to 1 kHz.
  writeReg(REG_CONFIG, 0x03);
  writeReg(REG_SMPLRT_DIV, SMPLRT_DIV);

  // Full-scale ranges.
  writeReg(REG_GYRO_CONFIG,  (GYR_FS_SEL & 0x03) << 3);
  writeReg(REG_ACCEL_CONFIG, (ACC_FS_SEL & 0x03) << 3);

  Serial.printf("Configured: %.0f Hz, +/-%d g, +/-%d dps\n",
                1000.0f / (1 + SMPLRT_DIV),
                (int)(2 << ACC_FS_SEL),
                (int)(250 << GYR_FS_SEL));
  Serial.println("Streaming (ax ay az | gx gy gz | tempC):");
}

// ------------------- Loop -------------------
void loop() {
  static uint32_t last = 0;
  uint32_t now = millis();
  if (now - last < PRINT_MS) return;
  last = now;

  uint8_t b[14];
  if (!readBlock(REG_ACCEL_XOUT_H, b, 14)) {
    Serial.println("i2c read failed");
    return;
  }

  int16_t ax = (b[0]  << 8) | b[1];
  int16_t ay = (b[2]  << 8) | b[3];
  int16_t az = (b[4]  << 8) | b[5];
  int16_t tRaw = (b[6] << 8) | b[7];
  int16_t gx = (b[8]  << 8) | b[9];
  int16_t gy = (b[10] << 8) | b[11];
  int16_t gz = (b[12] << 8) | b[13];

  float aLsb = ACC_LSB[ACC_FS_SEL];
  float gLsb = GYR_LSB[GYR_FS_SEL];

  float axg = ax / aLsb, ayg = ay / aLsb, azg = az / aLsb;
  float gxd = gx / gLsb, gyd = gy / gLsb, gzd = gz / gLsb;
  float tempC = tRaw / 340.0f + 36.53f;

  Serial.printf("a: %+6.2f %+6.2f %+6.2f g | g: %+7.1f %+7.1f %+7.1f dps | %.1f C\n",
                axg, ayg, azg, gxd, gyd, gzd, tempC);
}
