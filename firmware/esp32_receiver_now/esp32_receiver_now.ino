// esp32_receiver_now.ino
// ESP-NOW receiver that forwards sensor samples to the PC over USB serial.
// Pair with esp32_sensor_now.ino on each wireless sensor node.
//
// --- Serial output format (matches the Python box visualizer) ---
//   One CSV data line per IMU sample:
//     qw,qx,qy,qz,ax,ay,az,gx,gy,gz,mx,my,mz
//   Quaternion is set to identity and mag to 0 — the Python side runs its own
//   accel+gyro complementary filter and does not use either channel. Node ID
//   appears in "# node X ..." comment lines interleaved with the stream so a
//   future multi-node version can be added without breaking the CSV parser.
//
// --- Baud ---
//   1,000,000 — set --baud 1000000 on the Python side.
//
// --- Reconnection ---
//   Sensors broadcast with no pairing. When a sensor is power-cycled the
//   receiver notices the >500 ms silence and emits "# node X online (seq N)"
//   on the first new packet so you can see reconnect events in the stream.

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

static const uint32_t BAUD                  = 1000000;
static const uint8_t  WIFI_CHANNEL          = 1;
static const uint32_t RECONNECT_SILENCE_MS  = 500;
static const uint32_t STATS_PERIOD_MS       = 2000;
static const uint8_t  MAX_NODE_ID           = 8;

#pragma pack(push, 1)
struct SampleRaw {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
};

struct PacketHeader {
  uint16_t magic;        // 0x3D5D
  uint8_t  version;      // 2
  uint8_t  node_id;
  uint32_t pkt_seq;
  uint32_t t_us;
  uint16_t sample_hz;
  uint8_t  n_samples;
  uint8_t  acc_fs_sel;
  uint8_t  gyr_fs_sel;
  uint8_t  _pad[3];
};
#pragma pack(pop)

static const float ACC_LSB[4] = { 16384.0f, 8192.0f, 4096.0f, 2048.0f };
static const float GYR_LSB[4] = { 131.0f, 65.5f, 32.8f, 16.4f };

static uint32_t last_rx_ms_per_node [MAX_NODE_ID] = {0};
static uint32_t last_seq_per_node    [MAX_NODE_ID] = {0};
static uint32_t total_pkts    = 0;
static uint32_t total_samples = 0;
static uint32_t total_dropped = 0;

// New-style callback (IDF >= 5). The core v3 ESP32 Arduino uses this signature.
#if ESP_IDF_VERSION_MAJOR >= 5
static void onReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
#else
static void onReceive(const uint8_t *mac, const uint8_t *data, int len) {
#endif
  if (len < (int)sizeof(PacketHeader)) return;
  const PacketHeader *p = (const PacketHeader*)data;
  if (p->magic != 0x3D5D) return;
  if (p->version != 2)    return;
  if (p->n_samples == 0 || p->n_samples > 32) return;
  const int expected = sizeof(PacketHeader) + p->n_samples * (int)sizeof(SampleRaw);
  if (len < expected) return;

  if (p->node_id < MAX_NODE_ID) {
    const uint32_t now_ms = millis();
    if (now_ms - last_rx_ms_per_node[p->node_id] > RECONNECT_SILENCE_MS) {
      Serial.printf("# node %u online (seq %u)\n", p->node_id, p->pkt_seq);
      last_seq_per_node[p->node_id] = p->pkt_seq;
    } else {
      const uint32_t expect = last_seq_per_node[p->node_id] + 1;
      if (p->pkt_seq >= expect) {
        if (p->pkt_seq > expect) total_dropped += p->pkt_seq - expect;
        last_seq_per_node[p->node_id] = p->pkt_seq;
      }
    }
    last_rx_ms_per_node[p->node_id] = now_ms;
  }

  const float a_lsb = ACC_LSB[p->acc_fs_sel & 3];
  const float g_lsb = GYR_LSB[p->gyr_fs_sel & 3];
  const SampleRaw *samples = (const SampleRaw*)(data + sizeof(PacketHeader));
  for (int i = 0; i < p->n_samples; i++) {
    const SampleRaw &s = samples[i];
    Serial.printf("1,0,0,0,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,0,0,0\n",
                  s.ax / a_lsb, s.ay / a_lsb, s.az / a_lsb,
                  s.gx / g_lsb, s.gy / g_lsb, s.gz / g_lsb);
  }
  total_pkts++;
  total_samples += p->n_samples;
}

void setup() {
  Serial.begin(BAUD);
  Serial.setTxBufferSize(4096);
  delay(200);
  Serial.println("\n# esp32_receiver_now");

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_max_tx_power(84);

  if (esp_now_init() != ESP_OK) {
    Serial.println("# ESP-NOW init failed, halt");
    while (1) delay(1000);
  }
  esp_now_register_recv_cb(onReceive);

  Serial.printf("# receiver ready  ch=%u  baud=%u\n", WIFI_CHANNEL, BAUD);
  Serial.printf("# own MAC: %s\n", WiFi.macAddress().c_str());
  Serial.println("# format: qw,qx,qy,qz,ax,ay,az,gx,gy,gz,mx,my,mz "
                 "(quat=identity, mag=0; see '# node X online' lines)");
}

void loop() {
  static uint32_t last_stats_ms = 0;
  const uint32_t now_ms = millis();
  if (now_ms - last_stats_ms >= STATS_PERIOD_MS) {
    last_stats_ms = now_ms;
    Serial.printf("# stats: pkts=%u samples=%u dropped=%u\n",
                  total_pkts, total_samples, total_dropped);
  }
  delay(10);
}
