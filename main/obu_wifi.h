#pragma once

#include "obu_comm.h"

#ifdef __cplusplus
extern "C" {
#endif

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

void wifi_init(void);
int wifi_start(void);
void wifi_autostart(void);
int wifi_stop(void);

void wifi_tx_frame(uint8_t *payload, uint16_t length);

void wifi_get_stats(wifi_stats_t *stats);

void wifi_set_channel(uint16_t channel);
void wifi_set_filter(uint32_t filter);
void wifi_set_txpower(int powerDbm);
void wifi_set_datarate(uint16_t datarate);
void wifi_set_loop(uint8_t onOff);

uint32_t wifi_get_channel(void);
uint32_t wifi_get_filter(void);
int8_t wifi_get_txpower(void);

/* Time-slice the radio with BLE: pause releases the RF (promisc off, 802.11p
 * PHY off, esp_wifi_stop), resume re-applies the sniffer's state. The sniffer
 * task keeps running between calls — it just sees no RX callbacks while paused.
 */
void wifi_pause(void);
void wifi_resume(void);
/* wifi_pause() without the coex_enabled guard. Call after wifi_coex_disable()
 * so coex_window_task can't resume the radio in between. */
void wifi_pause_force(void);

void wifi_coex_enable(void);
void wifi_coex_disable(void);

/* Console cmd probing phy_get_cca_cnt/phy_set_cca_cnt, undocumented libphy.a
 * symbols reverse-engineered as a possible CCA busy/total counter pair for
 * ETSI DCC channel-busy-ratio. See `ccatest` command help for usage/caveats.
 */
void register_cca_test_cmd(void);

typedef struct {
  float latest_percent;  // most recent accepted (hardware-valid) 1s window's CBR, 0..100
  float avg_percent;     // outlier-filtered rolling average over the last few accepted windows
  uint32_t valid_samples;    // lifetime count of accepted windows
  uint32_t rejected_samples; // lifetime count of windows discarded (start-pulse miss-fire etc)
  bool has_data;          // false until the first window is accepted
} wifi_cca_stats_t;

/* Starts the background CBR measurement task (see docs/esp_dcc_measurements.md
 * for the hardware validation this is based on). Safe to call once; repeat
 * calls are no-ops. */
void wifi_cca_start(void);
void wifi_get_cca_stats(wifi_cca_stats_t *stats);
void register_cca_stats_cmd(void); // `ccastats` console command, read-only
void register_dccmon_cmd(void);    // `dccmon` console command, periodic background printer

#ifdef __cplusplus
}
#endif
