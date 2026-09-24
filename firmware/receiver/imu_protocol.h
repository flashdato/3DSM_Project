// 3DSM wireless IMU protocol, version 1.
//
// Shared by firmware/sensor_node and firmware/receiver.
// KEEP BOTH COPIES IDENTICAL (Arduino can't include a header from outside the
// sketch folder, so each sketch carries its own copy).
//
// Transport: ESP-NOW on a fixed Wi-Fi channel, max 250-byte payload.
//   receiver -> broadcast : beacon_pkt_t  (time sync + lets nodes learn the receiver MAC)
//   node     -> receiver  : data_pkt_t    (1..MAX_SAMPLES_PER_PKT raw IMU samples)
//
// All multi-byte fields are little-endian (native on ESP32).

#pragma once
#include <stdint.h>

#define IMU_PROTO_MAGIC    0x3D5Du
#define IMU_PROTO_VERSION  1

// Every node and the receiver must use the same channel.
#define ESPNOW_CHANNEL     1

enum : uint8_t {
  PKT_DATA   = 1,
  PKT_BEACON = 2,
};

// Body-segment IDs (matches the 10-node plan in HARDWARE.md).
enum : uint8_t {
  SEG_PELVIS      = 0,
  SEG_CHEST       = 1,
  SEG_R_UPPER_ARM = 2,
  SEG_R_FOREARM   = 3,
  SEG_L_UPPER_ARM = 4,
  SEG_L_FOREARM   = 5,
  SEG_R_THIGH     = 6,
  SEG_R_SHIN      = 7,
  SEG_L_THIGH     = 8,
  SEG_L_SHIN      = 9,
  SEG_HEAD        = 10,
};

// data_pkt_t.flags
#define FLAG_SYNCED     0x01  // sample timestamps are in receiver time (a beacon arrived recently)
#define FLAG_MAG_VALID  0x02  // mag[] holds real data
#define FLAG_OVERRUN    0x04  // node missed >= 1 data-ready event since the previous packet
#define FLAG_LOW_BATT   0x08

typedef struct __attribute__((packed)) {
  uint16_t magic;        // IMU_PROTO_MAGIC
  uint8_t  version;      // IMU_PROTO_VERSION
  uint8_t  type;         // PKT_BEACON
  uint16_t beacon_seq;
  uint64_t rx_time_us;   // receiver esp_timer_get_time() just before sending
} beacon_pkt_t;          // 14 bytes

typedef struct __attribute__((packed)) {
  uint32_t t_us;         // sample time, low 32 bits (receiver clock if FLAG_SYNCED, else node clock)
  uint16_t seq;          // per-node sample counter (wraps at 65536)
  int16_t  acc[3];       // raw counts, scale = 32768 / acc_fs_g  counts per g
  int16_t  gyr[3];       // raw counts, scale = 32768 / gyr_fs_dps counts per deg/s
  int16_t  mag[3];       // raw counts, only if FLAG_MAG_VALID
  int16_t  temp;         // raw MPU-6050: degC = temp / 340 + 36.53
} imu_sample_t;          // 26 bytes

#define MAX_SAMPLES_PER_PKT 8

typedef struct __attribute__((packed)) {
  uint16_t magic;        // IMU_PROTO_MAGIC
  uint8_t  version;      // IMU_PROTO_VERSION
  uint8_t  type;         // PKT_DATA
  uint8_t  node_id;
  uint8_t  segment_id;
  uint8_t  n;            // samples in this packet (1..MAX_SAMPLES_PER_PKT)
  uint8_t  flags;
  uint16_t pkt_seq;      // per-node packet counter
  uint16_t batt_mv;      // 0 = not measured
  uint16_t sample_hz;
  uint8_t  acc_fs_g;     // 2 / 4 / 8 / 16
  uint16_t gyr_fs_dps;   // 250 / 500 / 1000 / 2000
  imu_sample_t s[MAX_SAMPLES_PER_PKT];
} data_pkt_t;            // 17-byte header + 26 bytes per sample (max 225 bytes)

#define DATA_PKT_HEADER_BYTES (sizeof(data_pkt_t) - sizeof(imu_sample_t) * MAX_SAMPLES_PER_PKT)

static_assert(sizeof(beacon_pkt_t) == 14, "beacon size");
static_assert(sizeof(imu_sample_t) == 26, "sample size");
static_assert(DATA_PKT_HEADER_BYTES == 17, "data header size");
static_assert(sizeof(data_pkt_t) <= 250, "ESP-NOW v1 payload limit is 250 bytes");
