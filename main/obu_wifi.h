#pragma once

#include <atomic>
#include <cstdint>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_wifi.h"

#include "obu_comm.h"

typedef struct {
  uint8_t *payload;
  uint16_t length;
} wifi_packet_t;

typedef struct {
  uint32_t rx;
  uint32_t rxErr;
  uint32_t tx;
  uint32_t txErr;
} wifi_stats_t;

/**
 * @brief WLAN Sniffer Filter
 *
 */
typedef enum {
  WIFI_FILTER_MGMT = 0, /*!< MGMT */
  WIFI_FILTER_CTRL,     /*!< CTRL */
  WIFI_FILTER_DATA,     /*!< DATA */
  WIFI_FILTER_MISC,     /*!< MISC */
  WIFI_FILTER_MPDU,     /*!< MPDU */
  WIFI_FILTER_AMPDU,    /*!< AMPDU */
  WIFI_FILTER_FCSFAIL,  /*!< When this bit is set, the hardware will receive packets for which frame check sequence failed */
  WIFI_FILTER_MAX
} wifi_filter_t;

typedef struct {
  float latest_percent;  // most recent accepted (hardware-valid) 1s window's CBR, 0..100
  float avg_percent;     // outlier-filtered rolling average over the last few accepted windows
  uint32_t valid_samples;    // lifetime count of accepted windows
  uint32_t rejected_samples; // lifetime count of windows discarded (start-pulse miss-fire etc)
  bool has_data;          // false until the first window is accepted
} wifi_cca_stats_t;

/* Owns the 802.11p radio: promiscuous sniffer, raw TX, BLE coex time-slicing,
 * background CBR measurement and its wire-protocol push. Single instance —
 * the radio is a singleton.
 */
/* esp_wifi_80211_tx* raw-frame length limit (esp_wifi.h), bytes incl. 802.11 header. */
#define WIFI_TX_MAX_FRAME_LEN 1500

class WifiController
{
public:
  static WifiController &instance() { return s_instance; }

  WifiController(const WifiController &) = delete;
  WifiController &operator=(const WifiController &) = delete;

  void init();
  esp_err_t start();
  void autostart();
  esp_err_t stop();

  void txFrame(uint8_t *payload, uint16_t length);
  /* Bare 802.11 frame (no CommFrameHeader) at the current setDatarate() rate;
   * counts into stats(). length 24..WIFI_TX_MAX_FRAME_LEN. */
  bool txFrameRaw(uint8_t *payload, uint16_t length);

  wifi_stats_t stats() const { return stats_; }

  void setChannel(uint16_t channel);
  void setFilter(uint32_t filter) { filter_ = filter; }
  void setTxPower(int powerDbm);
  void setDatarate(uint16_t kbps) { txDatarate_ = kbps; }
  void setLoopback(bool on) { txLoopback_ = on; }

  uint16_t channel() const { return channel_; }
  uint32_t filter() const { return filter_; }
  int8_t txPower();

  /* Time-slice the radio with BLE: pause releases the RF (promisc off, 802.11p
   * PHY off, esp_wifi_stop), resume re-applies the sniffer's state. The sniffer
   * task keeps running between calls — it just sees no RX callbacks while paused.
   */
  void pause();
  void resume();
  /* pause() ignoring coex holds. Call after coexHold() so the BLE coex
   * scheduler can't resume the radio in between. */
  void pauseForce();
  /* resume() ignoring coex holds. Call after coexHold() so the radio is frozen
   * active (e.g. TX loop) even if the hold landed in a BLE window. */
  void resumeForce();

  /* Freeze the BLE/sniffer coex scheduler in its current radio state. Counted:
   * every hold needs one release; scheduler runs again when all are released. */
  void coexHold() { coexHolds_.fetch_add(1); }
  void coexRelease();

  /* Starts the background CBR measurement task (see docs/esp_dcc_measurements.md
   * for the hardware validation this is based on). Safe to call once; repeat
   * calls are no-ops. */
  void ccaStart();
  wifi_cca_stats_t ccaStats();

  /* Periodic COMM_FRAME_CBR_IND push over USB/BLE (whichever is connected),
   * driven by COMM_FRAME_CMD_CBR_START/STOP — see obu_comm.cpp. intervalMs must be
   * 1..CBR_IND_MAX_INTERVAL_MS; obu_comm.cpp validates before calling. */
  void cbrIndStart(uint32_t intervalMs);
  void cbrIndStop() { cbrIndIntervalMs_ = -1; }

private:
  static constexpr int CCA_HISTORY_LEN = 5; // rolling window for the filtered average

  constexpr WifiController() = default;

  static WifiController s_instance;

  static void rxCallback(void *recvBuf, wifi_promiscuous_pkt_type_t type);
  static void sniffTaskEntry(void *arg);
  static void ccaTaskEntry(void *arg);
  static void cbrIndTaskEntry(void *arg);

  void sniffTask();
  void ccaTask();
  void cbrIndTask();

  esp_err_t tuneRadio();
  void queuePacket(const void *data, uint16_t len, const CommFrameHeader *info);
  void loopback(const uint8_t *payload, uint16_t len, uint16_t chan, uint16_t rate);
  float ccaFilteredAverage() const;
  void cbrIndPublish();

  bool running_ = false;
  bool paused_ = false;
  /* Outstanding coexHold()s (USB rx, tx task, FW update). While >0,
   * pause()/resume() no-op, so the coex scheduler can't move the radio. */
  std::atomic<int> coexHolds_{0};

  uint16_t channel_ = 0;        // MHz, e.g. 5900 = CCH
  uint32_t filter_ = 0;         // WIFI_PROMIS_FILTER_MASK_* bits
  uint16_t txDatarate_ = 6000;  // kbps, used when a TX header leaves rate 0
  int8_t txPower_ = -1;         // dBm; -1 = unknown, re-read from driver
  bool txLoopback_ = false;

  wifi_tx_rate_config_t txRateConfig_ = {
    .phymode = WIFI_PHY_MODE_11A,
    .rate = WIFI_PHY_RATE_6M,
    .ersu = false,
    .dcm = false,
  };

  wifi_stats_t stats_ = {};

  TaskHandle_t task_ = nullptr;
  QueueHandle_t workQueue_ = nullptr;
  SemaphoreHandle_t semTaskOver_ = nullptr;

  SemaphoreHandle_t ccaMutex_ = nullptr; // guards ccaHistory*/ccaStats_
  TaskHandle_t ccaTask_ = nullptr;
  float ccaHistory_[CCA_HISTORY_LEN] = {};
  uint8_t ccaHistoryCount_ = 0;
  uint8_t ccaHistoryNext_ = 0;
  wifi_cca_stats_t ccaStats_ = {};

  int cbrIndIntervalMs_ = -1; // <=0: task idles, doesn't publish
  TaskHandle_t cbrIndTask_ = nullptr;
};

/* Console cmd probing phy_get_cca_cnt/phy_set_cca_cnt, undocumented libphy.a
 * symbols reverse-engineered as a possible CCA busy/total counter pair for
 * ETSI DCC channel-busy-ratio. See `ccatest` command help for usage/caveats.
 */
void register_cca_test_cmd(void);
void register_cca_stats_cmd(void); // `ccastats` console command, read-only
void register_dccmon_cmd(void);    // `dccmon` console command, periodic background printer
