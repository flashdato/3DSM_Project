// esp32_visualize_gy87.ino
// ESP32 + GY-87 10DOF -> serial stream of segment orientation for the
// Python 3D visualizer (tools/visualize_arm_gy87.py). Full 3-axis
// orientation using accel + magnetometer (no gyro drift) with a 5-second
// N-pose calibration so the on-screen arm starts at rest and mirrors the
// physical arm from there.
//
// Boot sequence:
//   1. 2 s warmup (board stabilizes, Serial drains).
//   2. 5 s N-pose capture: stand upright, right arm hanging straight down,
//      palm facing your thigh. Hold still. The sketch averages accel/gyro/mag
//      to get gyro bias + reference frame.
//   3. Stream at 50 Hz:
//        qw,qx,qy,qz,ax,ay,az,gx,gy,gz,mx,my,mz
//      The quaternion represents the WORLD rotation that moves the rest-pose
//      segment to its current pose, i.e. v_now_world = q * v_rest_world.
//      At N-pose the quaternion is identity.
//
// Wiring (5V side of the DevKit, same as esp32_gy87_test):
//   VCC_IN -> VIN / 5V
//   GND    -> GND
//   SDA    -> GPIO 26
//   SCL    -> GPIO 25

#include <Wire.h>
#include <math.h>

// ------------- Config -------------
static const int PIN_SDA = 26;
static const int PIN_SCL = 25;

static const uint8_t MPU_ADDR  = 0x68;
static const uint8_t QMCP_ADDR = 0x2C;   // QMC5883P (newer QST mag)

static const uint8_t ACC_FS_SEL = 2;     // +/-8 g
static const uint8_t GYR_FS_SEL = 2;     // +/-1000 dps
static const uint8_t SMPLRT_DIV = 9;     // 100 Hz

static const uint32_t WARMUP_MS = 2000;
static const uint32_t CALIB_MS  = 5000;
static const uint32_t PRINT_MS  = 20;    // 50 Hz output

// ------------- Registers -------------
#define REG_SMPLRT_DIV   0x19
#define REG_CONFIG       0x1A
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_INT_PIN_CFG  0x37
#define REG_ACCEL_XOUT_H 0x3B
#define REG_USER_CTRL    0x6A
#define REG_PWR_MGMT_1   0x6B

static const float ACC_LSB[4] = { 16384.0f, 8192.0f, 4096.0f, 2048.0f };
static const float GYR_LSB[4] = { 131.0f, 65.5f, 32.8f, 16.4f };

// ------------- State -------------
static float biasGx = 0, biasGy = 0, biasGz = 0;
static float R_cal[9];        // world->body rotation at N-pose, row-major
static uint32_t lastPrint = 0;

// ------------- I2C helpers -------------
static void writeReg(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr); Wire.write(reg); Wire.write(val); Wire.endTransmission();
}

static bool readMPU(float a[3], float g[3]) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(REG_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)14) != 14) return false;
  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();                                  // temp (unused)
  int16_t gx = (Wire.read() << 8) | Wire.read();
  int16_t gy = (Wire.read() << 8) | Wire.read();
  int16_t gz = (Wire.read() << 8) | Wire.read();
  const float aL = ACC_LSB[ACC_FS_SEL];
  const float gL = GYR_LSB[GYR_FS_SEL];
  a[0] = ax / aL; a[1] = ay / aL; a[2] = az / aL;
  g[0] = gx / gL; g[1] = gy / gL; g[2] = gz / gL;
  return true;
}

static bool readMag(float m[3]) {
  Wire.beginTransmission(QMCP_ADDR); Wire.write(0x01);        // data regs 0x01..0x06
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(QMCP_ADDR, (uint8_t)6) != 6) return false;
  int16_t mx = (int16_t)(Wire.read() | (Wire.read() << 8));
  int16_t my = (int16_t)(Wire.read() | (Wire.read() << 8));
  int16_t mz = (int16_t)(Wire.read() | (Wire.read() << 8));
  m[0] = mx; m[1] = my; m[2] = mz;
  return true;
}

// ------------- Linear-algebra -------------
static void normalize3(float v[3]) {
  float n = sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
  if (n < 1e-6f) { v[0]=0; v[1]=0; v[2]=1; return; }
  v[0]/=n; v[1]/=n; v[2]/=n;
}

static void cross3(const float a[3], const float b[3], float out[3]) {
  out[0] = a[1]*b[2] - a[2]*b[1];
  out[1] = a[2]*b[0] - a[0]*b[2];
  out[2] = a[0]*b[1] - a[1]*b[0];
}

static void transpose3(const float A[9], float T[9]) {
  T[0]=A[0]; T[1]=A[3]; T[2]=A[6];
  T[3]=A[1]; T[4]=A[4]; T[5]=A[7];
  T[6]=A[2]; T[7]=A[5]; T[8]=A[8];
}

static void matmul3(const float A[9], const float B[9], float C[9]) {
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      C[i*3+j] = A[i*3+0]*B[0*3+j] + A[i*3+1]*B[1*3+j] + A[i*3+2]*B[2*3+j];
}

// Build rotation matrix R (row-major) such that v_body = R @ v_world.
// Rows of R are the world basis (N, E, U) expressed in the body frame.
static void buildR(const float a[3], const float m[3], float R[9]) {
  float up[3] = { a[0], a[1], a[2] };
  normalize3(up);
  float d = m[0]*up[0] + m[1]*up[1] + m[2]*up[2];
  float north[3] = { m[0] - d*up[0], m[1] - d*up[1], m[2] - d*up[2] };
  normalize3(north);
  float east[3];
  cross3(up, north, east);          // right-handed NEU
  R[0]=north[0]; R[1]=north[1]; R[2]=north[2];
  R[3]=east[0];  R[4]=east[1];  R[5]=east[2];
  R[6]=up[0];    R[7]=up[1];    R[8]=up[2];
}

// Rotation matrix (row-major) -> unit quaternion [w, x, y, z].
static void matToQuat(const float R[9], float q[4]) {
  float tr = R[0] + R[4] + R[8];
  if (tr > 0) {
    float s = sqrtf(tr + 1.0f) * 2.0f;
    q[0] = 0.25f * s;
    q[1] = (R[7] - R[5]) / s;
    q[2] = (R[2] - R[6]) / s;
    q[3] = (R[3] - R[1]) / s;
  } else if (R[0] > R[4] && R[0] > R[8]) {
    float s = sqrtf(1.0f + R[0] - R[4] - R[8]) * 2.0f;
    q[0] = (R[7] - R[5]) / s;
    q[1] = 0.25f * s;
    q[2] = (R[1] + R[3]) / s;
    q[3] = (R[2] + R[6]) / s;
  } else if (R[4] > R[8]) {
    float s = sqrtf(1.0f + R[4] - R[0] - R[8]) * 2.0f;
    q[0] = (R[2] - R[6]) / s;
    q[1] = (R[1] + R[3]) / s;
    q[2] = 0.25f * s;
    q[3] = (R[5] + R[7]) / s;
  } else {
    float s = sqrtf(1.0f + R[8] - R[0] - R[4]) * 2.0f;
    q[0] = (R[3] - R[1]) / s;
    q[1] = (R[2] + R[6]) / s;
    q[2] = (R[5] + R[7]) / s;
    q[3] = 0.25f * s;
  }
}

static void qmcpInit() {
  writeReg(QMCP_ADDR, 0x0B, 0x80); delay(10);      // soft reset
  writeReg(QMCP_ADDR, 0x0B, 0x0C);                 // RNG = +/-8G, SET/RESET on
  writeReg(QMCP_ADDR, 0x0A, 0xC3);                 // continuous, 200 Hz, OSR config
  delay(10);
}

// ------------- Setup -------------
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("# esp32_visualize_gy87");

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);

  // MPU-6050 setup + enable AUX bypass so the QMC5883P is visible.
  writeReg(MPU_ADDR, REG_PWR_MGMT_1, 0x01);       // wake, PLL on gyro X
  delay(50);
  writeReg(MPU_ADDR, REG_CONFIG,       0x03);     // DLPF ~44 Hz
  writeReg(MPU_ADDR, REG_SMPLRT_DIV,   SMPLRT_DIV);
  writeReg(MPU_ADDR, REG_GYRO_CONFIG,  (GYR_FS_SEL & 3) << 3);
  writeReg(MPU_ADDR, REG_ACCEL_CONFIG, (ACC_FS_SEL & 3) << 3);
  writeReg(MPU_ADDR, REG_USER_CTRL,    0x00);
  writeReg(MPU_ADDR, REG_INT_PIN_CFG,  0x02);
  delay(10);

  qmcpInit();

  // ---- 1. Warmup ----
  Serial.printf("# warmup %lu ms...\n", (unsigned long)WARMUP_MS);
  delay(WARMUP_MS);

  // ---- 2. N-pose calibration ----
  Serial.printf("# CALIBRATING: hold arm hanging down, palm toward thigh, %lu s...\n",
                (unsigned long)(CALIB_MS / 1000));
  double sa[3] = {0,0,0}, sg[3] = {0,0,0}, sm[3] = {0,0,0};
  uint32_t na = 0, nm = 0;
  uint32_t t0 = millis();
  uint32_t tickT = t0;
  while (millis() - t0 < CALIB_MS) {
    float a[3], g[3], m[3];
    if (readMPU(a, g)) {
      for (int i = 0; i < 3; i++) { sa[i] += a[i]; sg[i] += g[i]; }
      na++;
    }
    if (readMag(m)) {
      for (int i = 0; i < 3; i++) sm[i] += m[i];
      nm++;
    }
    if (millis() - tickT >= 1000) {
      tickT = millis();
      uint32_t left = (CALIB_MS - (millis() - t0)) / 1000;
      Serial.printf("#   %lus left\n", (unsigned long)left + 1);
    }
    delay(5);
  }
  if (na == 0 || nm == 0) {
    Serial.println("# ERROR: no sensor samples captured during calibration - halt.");
    while (1) delay(10);
  }
  float a_cal[3] = { (float)(sa[0]/na), (float)(sa[1]/na), (float)(sa[2]/na) };
  float m_cal[3] = { (float)(sm[0]/nm), (float)(sm[1]/nm), (float)(sm[2]/nm) };
  biasGx = sg[0] / na; biasGy = sg[1] / na; biasGz = sg[2] / na;
  buildR(a_cal, m_cal, R_cal);

  Serial.printf("# N-pose captured: a=(%.3f,%.3f,%.3f) m=(%.0f,%.0f,%.0f)\n",
                a_cal[0], a_cal[1], a_cal[2], m_cal[0], m_cal[1], m_cal[2]);
  Serial.printf("# gyro bias dps: (%.2f, %.2f, %.2f) from %lu samples\n",
                biasGx, biasGy, biasGz, (unsigned long)na);
  Serial.println("# format: qw,qx,qy,qz,ax,ay,az,gx,gy,gz,mx,my,mz");

  lastPrint = millis();
}

// ------------- Loop -------------
void loop() {
  float a[3], g[3], m[3];
  if (!readMPU(a, g)) return;
  readMag(m);                                   // best-effort; keeps prior values on failure

  uint32_t nowMs = millis();
  if (nowMs - lastPrint < PRINT_MS) return;
  lastPrint = nowMs;

  g[0] -= biasGx; g[1] -= biasGy; g[2] -= biasGz;

  // Current world->body rotation.
  float R_now[9];
  buildR(a, m, R_now);

  // World rotation that moves a rest-pose point P to its current location:
  //   P_now_world = R_now @ P_body
  //   P_rest_world = R_cal @ P_body   -> P_body = R_cal^T @ P_rest_world
  //   P_now_world = (R_now @ R_cal^T) @ P_rest_world
  // So W = R_now @ R_cal^T.  At N-pose W = I, so the model starts at rest.
  float R_calT[9], W[9];
  transpose3(R_cal, R_calT);
  matmul3(R_now, R_calT, W);

  float q[4];
  matToQuat(W, q);

  Serial.printf("%.4f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.0f,%.0f,%.0f\n",
                q[0], q[1], q[2], q[3],
                a[0], a[1], a[2],
                g[0], g[1], g[2],
                m[0], m[1], m[2]);
}
