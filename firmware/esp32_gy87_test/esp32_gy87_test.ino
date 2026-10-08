// GY-87 10DOF bring-up test for 3DSM Phase 3.
// Verifies all three chips on the module and streams raw numbers.
//
//   MPU-6050    accel + gyro        I2C 0x68
//   HMC5883L    magnetometer        I2C 0x1E   (or QMC5883L clone at 0x0D)
//   BMP180      pressure + temp     I2C 0x77
//
// The magnetometer and BMP180 are wired to the MPU-6050's auxiliary I2C bus
// and are HIDDEN from the ESP32 by default. Setting I2C_BYPASS_EN in
// INT_PIN_CFG (0x37) and clearing USER_CTRL (0x6A) connects AUX as a
// pass-through so the ESP32 can address all three chips on one bus.
//
// Wiring (all four wires on the 5V side of the ESP32 DevKit):
//   VCC_IN -> VIN / 5V         (regulator on the GY-87 drops it to 3.3V)
//   GND    -> GND              (the one next to VIN)
//   SDA    -> GPIO 26
//   SCL    -> GPIO 25
// INT, DRDY and FSYNC are left unconnected for this test.
//
// GPIO 26 / 25 were picked because they sit on the same edge as VIN/GND,
// they aren't strapping pins and they don't conflict with the on-chip flash.
// Any other output-capable pair works too (e.g. 32/33, 13/14).

#include <Wire.h>
#include <Adafruit_BMP085.h>

#define PIN_SDA  26
#define PIN_SCL  25
#define I2C_HZ   400000

#define MPU_ADDR  0x68
#define HMC_ADDR  0x1E   // HMC5883L            (original Honeywell)
#define QMC_ADDR  0x0D   // QMC5883L            (first-gen QST clone)
#define QMCP_ADDR 0x2C   // QMC5883P            (newer QST, common on recent GY-87 clones)
#define BMP_ADDR  0x77

// MPU-6050 registers
#define MPU_SMPLRT_DIV    0x19
#define MPU_CONFIG        0x1A
#define MPU_GYRO_CONFIG   0x1B
#define MPU_ACCEL_CONFIG  0x1C
#define MPU_INT_PIN_CFG   0x37
#define MPU_ACCEL_XOUT_H  0x3B
#define MPU_USER_CTRL     0x6A
#define MPU_PWR_MGMT_1    0x6B
#define MPU_WHO_AM_I      0x75

Adafruit_BMP085 bmp;
enum MagKind { MAG_NONE, MAG_HMC, MAG_QMCL, MAG_QMCP };
uint8_t  magAddr = 0;
MagKind  magKind = MAG_NONE;
bool     bmpOk   = false;

// --- helpers --------------------------------------------------------------

uint8_t readReg(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, (uint8_t)1);
  return Wire.read();
}

void writeReg(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

bool ping(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

void i2cScan() {
  Serial.println("I2C scan:");
  for (uint8_t a = 1; a < 127; a++) {
    if (ping(a)) Serial.printf("  found 0x%02X\n", a);
  }
}

// --- MPU-6050 -------------------------------------------------------------

void mpuInit() {
  writeReg(MPU_ADDR, MPU_PWR_MGMT_1,   0x01);  // wake, clock = PLL on gyro X
  writeReg(MPU_ADDR, MPU_SMPLRT_DIV,   9);     // 1 kHz / (1+9) = 100 Hz
  writeReg(MPU_ADDR, MPU_CONFIG,       0x03);  // DLPF ~44 Hz accel / 42 Hz gyro
  writeReg(MPU_ADDR, MPU_GYRO_CONFIG,  0x10);  // +/-1000 dps
  writeReg(MPU_ADDR, MPU_ACCEL_CONFIG, 0x10);  // +/-8 g

  // Expose the auxiliary bus so HMC/QMC and BMP180 become visible.
  writeReg(MPU_ADDR, MPU_USER_CTRL,   0x00);
  writeReg(MPU_ADDR, MPU_INT_PIN_CFG, 0x02);
}

void mpuRead(int16_t &ax, int16_t &ay, int16_t &az,
             int16_t &gx, int16_t &gy, int16_t &gz,
             int16_t &tcnt) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)14);
  ax   = (Wire.read() << 8) | Wire.read();
  ay   = (Wire.read() << 8) | Wire.read();
  az   = (Wire.read() << 8) | Wire.read();
  tcnt = (Wire.read() << 8) | Wire.read();
  gx   = (Wire.read() << 8) | Wire.read();
  gy   = (Wire.read() << 8) | Wire.read();
  gz   = (Wire.read() << 8) | Wire.read();
}

// --- magnetometer (HMC vs QMC) --------------------------------------------

bool hmcInit() {
  if (!ping(HMC_ADDR)) return false;
  writeReg(HMC_ADDR, 0x00, 0x70);  // 8-sample average, 15 Hz, normal
  writeReg(HMC_ADDR, 0x01, 0x20);  // gain +/-1.3 Ga
  writeReg(HMC_ADDR, 0x02, 0x00);  // continuous mode
  return true;
}

bool qmcInit() {
  if (!ping(QMC_ADDR)) return false;
  writeReg(QMC_ADDR, 0x0B, 0x01);  // set/reset period
  writeReg(QMC_ADDR, 0x09, 0x1D);  // continuous, 200 Hz, +/-8 G, OSR 512
  return true;
}

// QMC5883P: newer QST part at 0x2C, different register map from QMC5883L.
//   0x00  = chip ID (expect 0x80)
//   0x01..0x06 = data XL XH YL YH ZL ZH (little-endian i16, same byte order as QMC5883L)
//   0x09  = status  (bit0 DRDY)
//   0x0A  = CTRL1   [OSR2 7:6] [OSR1 5:4] [ODR 3:2] [MODE 1:0]
//   0x0B  = CTRL2   [SOFT_RST 7] [SELF_TEST 6] [RNG 3:2] [SET_RST 1:0]
bool qmcpInit() {
  if (!ping(QMCP_ADDR)) return false;
  writeReg(QMCP_ADDR, 0x0B, 0x80);   // soft reset
  delay(10);
  writeReg(QMCP_ADDR, 0x0B, 0x0C);   // RNG = +/-8G,  SET/RESET on
  writeReg(QMCP_ADDR, 0x0A, 0xC3);   // OSR2=1, OSR1=1, ODR=200Hz, MODE=continuous
  delay(10);
  return true;
}

void qmcpDumpRegs() {
  Serial.printf("QMC5883P register dump @0x%02X (hex, regs 0x00..0x0F):\n  ", QMCP_ADDR);
  for (uint8_t r = 0x00; r <= 0x0F; r++) Serial.printf("%02X ", readReg(QMCP_ADDR, r));
  Serial.println();
}

void magRead(int16_t &x, int16_t &y, int16_t &z) {
  uint8_t dataReg = (magKind == MAG_HMC)  ? 0x03
                  : (magKind == MAG_QMCL) ? 0x00
                                          : 0x01;   // QMC5883P
  Wire.beginTransmission(magAddr);
  Wire.write(dataReg);
  Wire.endTransmission(false);
  Wire.requestFrom(magAddr, (uint8_t)6);
  if (magKind == MAG_HMC) {           // HMC: XH XL ZH ZL YH YL (note Z before Y)
    x = (Wire.read() << 8) | Wire.read();
    z = (Wire.read() << 8) | Wire.read();
    y = (Wire.read() << 8) | Wire.read();
  } else {                            // QMC5883L & QMC5883P: XL XH YL YH ZL ZH
    x = Wire.read() | (Wire.read() << 8);
    y = Wire.read() | (Wire.read() << 8);
    z = Wire.read() | (Wire.read() << 8);
  }
}

// --- setup / loop ---------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(400);
  Serial.println();
  Serial.println("=== ESP32 + GY-87 10DOF bring-up ===");

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(I2C_HZ);

  // 1. MPU-6050 first (it owns the bus until bypass is enabled).
  if (!ping(MPU_ADDR)) {
    Serial.println("MPU-6050 not found at 0x68 - check wiring, halt.");
    while (1) delay(10);
  }
  uint8_t who = readReg(MPU_ADDR, MPU_WHO_AM_I);
  Serial.printf("MPU-6050 WHO_AM_I = 0x%02X  (expect 0x68 or 0x72 on clones)\n", who);
  mpuInit();
  delay(10);

  // 2. Scan AFTER bypass is on - this is when the AUX devices appear.
  i2cScan();

  // 3. Magnetometer: try HMC first, then QMC5883L (0x0D), then QMC5883P (0x2C).
  if (hmcInit())  { magAddr = HMC_ADDR;  magKind = MAG_HMC;  Serial.println("HMC5883L ok at 0x1E"); }
  else if (qmcInit())  { magAddr = QMC_ADDR;  magKind = MAG_QMCL; Serial.println("QMC5883L ok at 0x0D"); }
  else if (qmcpInit()) {
    magAddr = QMCP_ADDR; magKind = MAG_QMCP;
    uint8_t cid = readReg(QMCP_ADDR, 0x00);
    Serial.printf("QMC5883P ok at 0x2C  (chip ID reg 0x00 = 0x%02X, expect 0x80)\n", cid);
    qmcpDumpRegs();
  }
  else Serial.println("No magnetometer found - skipping.");

  // 4. BMP180 (Adafruit lib handles the calibration maths).
  bmpOk = bmp.begin();
  Serial.println(bmpOk ? "BMP180 ok at 0x77" : "BMP180 not found - skipping.");

  Serial.println();
  Serial.println("Streaming (a [g] | g [dps] | m [counts] | P [hPa] alt [m] | T [C]):");
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz, tcnt;
  mpuRead(ax, ay, az, gx, gy, gz, tcnt);

  int16_t mx = 0, my = 0, mz = 0;
  if (magAddr) magRead(mx, my, mz);

  const float AG = 1.0f / 4096.0f;   // +/-8 g       LSB sensitivity
  const float GD = 1.0f / 32.8f;     // +/-1000 dps  LSB sensitivity
  float tC   = tcnt / 340.0f + 36.53f;
  float pHpa = bmpOk ? bmp.readPressure() / 100.0f : 0;
  float alt  = bmpOk ? bmp.readAltitude()           : 0;

  Serial.printf("a: %+6.2f %+6.2f %+6.2f | g: %+7.1f %+7.1f %+7.1f | m: %+6d %+6d %+6d | %7.2f hPa  %+6.1f m | %4.1f C\n",
                ax * AG, ay * AG, az * AG,
                gx * GD, gy * GD, gz * GD,
                mx, my, mz,
                pHpa, alt, tC);

  delay(50);   // ~20 Hz, human-readable rate
}
