// 3DSM receiver: one ESP32 plugged into the laptop over USB.
//
// - Broadcasts a time beacon every BEACON_PERIOD_MS. Nodes sync their clocks to it
//   and learn this board's MAC from it.
// - Receives data packets from all nodes over ESP-NOW.
// - Prints every sample as one CSV line on USB serial (921600 baud), plus a
//   stats line per node every STATS_PERIOD_MS.
//
// Serial line formats (see HARDWARE.md, "Serial output"):
//   #...                                  comments / metadata
//   N,node,segment,sample_hz,acc_fs_g,gyr_fs_dps,mac        first time a node is seen
//   D,node,segment,seq,t_us,ax,ay,az,gx,gy,gz,mx,my,mz,temp,flags,batt_mv,rssi,rx_us
//   S,node,pkts_per_s,samples_per_s,lost_total,rssi,latency_ms,synced,batt_mv

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_timer.h>
#include "imu_protocol.h"

#define SERIAL_BAUD       921600
#define BEACON_PERIOD_MS  1000
#define STATS_PERIOD_MS   2000
#define MAX_NODES         16
#define RX_QUEUE_LEN      64

static const uint8_t BROADCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

typedef struct {
  uint8_t  mac[6];
  int8_t   rssi;
  int16_t  len;
  int64_t  rx_us;
  uint8_t  data[250];
} rx_item_t;

static QueueHandle_t g_rxQueue;
static volatile uint32_t g_queueDrops = 0;

static void enqueue(const uint8_t *mac, int8_t rssi, const uint8_t *data, int len) {
  int64_t t = esp_timer_get_time();
  if (len <= 0 || len > 250) return;
  rx_item_t item;
  memcpy(item.mac, mac, 6);
  item.rssi = rssi;
  item.len = (int16_t)len;
  item.rx_us = t;
  memcpy(item.data, data, len);
  if (xQueueSend(g_rxQueue, &item, 0) != pdTRUE) g_queueDrops = g_queueDrops + 1;
}

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
static void onEspNowRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  int8_t rssi = info->rx_ctrl ? (int8_t)info->rx_ctrl->rssi : 0;
  enqueue(info->src_addr, rssi, data, len);
}
#else
static void onEspNowRecv(const uint8_t *mac, const uint8_t *data, int len) {
  enqueue(mac, 0, data, len);   // RSSI not exposed by the 2.x callback
}
#endif

// ---------------- per-node bookkeeping ----------------
typedef struct {
  bool     used;
  uint8_t  node_id;
  uint8_t  segment_id;
  bool     announced;
  bool     haveSeq;
  uint16_t lastSampleSeq;
  uint32_t lostTotal;
  uint32_t pkts, samples;       // since last stats line
  int8_t   rssi;
  int64_t  latencySumUs;
  uint32_t latencyN;
  bool     synced;
  uint16_t batt_mv;
} node_state_t;

static node_state_t g_nodes[MAX_NODES];

static node_state_t *nodeFor(uint8_t id) {
  for (auto &n : g_nodes) if (n.used && n.node_id == id) return &n;
  for (auto &n : g_nodes) if (!n.used) { memset(&n, 0, sizeof(n)); n.used = true; n.node_id = id; return &n; }
  return nullptr;
}

// Nodes send the low 32 bits of a receiver-clock timestamp; rebuild the full
// 64-bit value using our own clock at arrival (samples are only ms old).
static int64_t unwrap32(uint32_t t32, int64_t ref) {
  int64_t t = (ref & ~0xFFFFFFFFLL) | (int64_t)t32;
  if (t > ref + 0x80000000LL) t -= 0x100000000LL;
  else if (t < ref - 0x80000000LL) t += 0x100000000LL;
  return t;
}

static void handleData(const rx_item_t &it) {
  if (it.len < (int)DATA_PKT_HEADER_BYTES) return;
  data_pkt_t p;
  memset(&p, 0, sizeof(p));
  memcpy(&p, it.data, it.len);
  if (p.magic != IMU_PROTO_MAGIC || p.version != IMU_PROTO_VERSION || p.type != PKT_DATA) return;
  if (p.n == 0 || p.n > MAX_SAMPLES_PER_PKT) return;
  if (it.len < (int)(DATA_PKT_HEADER_BYTES + p.n * sizeof(imu_sample_t))) return;

  node_state_t *ns = nodeFor(p.node_id);
  if (!ns) return;
  ns->segment_id = p.segment_id;
  if (!ns->announced) {
    Serial.printf("N,%u,%u,%u,%u,%u,%02X:%02X:%02X:%02X:%02X:%02X\n",
                  p.node_id, p.segment_id, p.sample_hz, p.acc_fs_g, p.gyr_fs_dps,
                  it.mac[0], it.mac[1], it.mac[2], it.mac[3], it.mac[4], it.mac[5]);
    ns->announced = true;
  }

  bool synced = p.flags & FLAG_SYNCED;
  ns->pkts++;
  ns->rssi = it.rssi;
  ns->synced = synced;
  ns->batt_mv = p.batt_mv;

  for (uint8_t i = 0; i < p.n; i++) {
    const imu_sample_t &s = p.s[i];
    if (ns->haveSeq) {
      uint16_t gap = (uint16_t)(s.seq - ns->lastSampleSeq);
      if (gap > 1 && gap < 30000) ns->lostTotal += gap - 1;
    }
    ns->haveSeq = true;
    ns->lastSampleSeq = s.seq;
    ns->samples++;

    int64_t t = synced ? unwrap32(s.t_us, it.rx_us) : (int64_t)s.t_us;
    if (synced && i == p.n - 1) {             // newest sample: sensor-to-receiver latency
      ns->latencySumUs += it.rx_us - t;
      ns->latencyN++;
    }
    Serial.printf("D,%u,%u,%u,%lld,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%u,%u,%d,%lld\n",
                  p.node_id, p.segment_id, s.seq, (long long)t,
                  s.acc[0], s.acc[1], s.acc[2], s.gyr[0], s.gyr[1], s.gyr[2],
                  s.mag[0], s.mag[1], s.mag[2], s.temp,
                  p.flags, p.batt_mv, it.rssi, (long long)it.rx_us);
  }
}

static void sendBeacon() {
  static uint16_t seq = 0;
  beacon_pkt_t b;
  b.magic = IMU_PROTO_MAGIC;
  b.version = IMU_PROTO_VERSION;
  b.type = PKT_BEACON;
  b.beacon_seq = seq++;
  b.rx_time_us = (uint64_t)esp_timer_get_time();   // stamped as late as possible before sending
  esp_now_send(BROADCAST, (const uint8_t *)&b, sizeof(b));
}

static void printStats(float periodS) {
  for (auto &n : g_nodes) {
    if (!n.used) continue;
    float lat = n.latencyN ? (float)n.latencySumUs / n.latencyN / 1000.0f : -1.0f;
    Serial.printf("S,%u,%.1f,%.1f,%lu,%d,%.2f,%d,%u\n",
                  n.node_id, n.pkts / periodS, n.samples / periodS, (unsigned long)n.lostTotal,
                  n.rssi, lat, n.synced ? 1 : 0, n.batt_mv);
    n.pkts = n.samples = 0;
    n.latencySumUs = 0;
    n.latencyN = 0;
  }
  if (g_queueDrops) Serial.printf("# receiver queue drops: %lu\n", (unsigned long)g_queueDrops);
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(300);
  g_rxQueue = xQueueCreate(RX_QUEUE_LEN, sizeof(rx_item_t));

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) {
    Serial.println("# esp_now_init failed");
    while (true) delay(1000);
  }
  esp_now_register_recv_cb(onEspNowRecv);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BROADCAST, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.encrypt = false;
  peer.ifidx = WIFI_IF_STA;
  esp_now_add_peer(&peer);

  Serial.printf("# 3DSM receiver, protocol v%d, channel %d, MAC %s\n",
                IMU_PROTO_VERSION, ESPNOW_CHANNEL, WiFi.macAddress().c_str());
  Serial.println("# N,node,segment,sample_hz,acc_fs_g,gyr_fs_dps,mac");
  Serial.println("# D,node,segment,seq,t_us,ax,ay,az,gx,gy,gz,mx,my,mz,temp,flags,batt_mv,rssi,rx_us");
  Serial.println("# S,node,pkts_per_s,samples_per_s,lost_total,rssi,latency_ms,synced,batt_mv");
}

void loop() {
  uint32_t nowMs = millis();

  static uint32_t lastBeaconMs = 0;
  if (nowMs - lastBeaconMs >= BEACON_PERIOD_MS) {
    lastBeaconMs = nowMs;
    sendBeacon();
  }

  rx_item_t it;
  while (xQueueReceive(g_rxQueue, &it, 0) == pdTRUE) {
    if (it.len >= 4 && it.data[3] == PKT_DATA) handleData(it);
  }

  static uint32_t lastStatsMs = 0;
  if (nowMs - lastStatsMs >= STATS_PERIOD_MS) {
    printStats((nowMs - lastStatsMs) / 1000.0f);
    lastStatsMs = nowMs;
  }

  delay(1);
}
