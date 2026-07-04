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

void wifi_set_channel(uint32_t channel);
void wifi_set_filter(uint32_t filter);
void wifi_set_txpower(int powerDbm);
void wifi_set_datarate(int datarate);

uint32_t wifi_get_channel(void);
uint32_t wifi_get_filter(void);
int8_t wifi_get_txpower(void);

/* Time-slice the radio with BLE: pause releases the RF (promisc off, 802.11p
 * PHY off, esp_wifi_stop), resume re-applies the sniffer's state. The sniffer
 * task keeps running between calls — it just sees no RX callbacks while paused.
 */
void wifi_pause(void);
void wifi_resume(void);

void wifi_coex_enable(void);
void wifi_coex_disable(void);

#ifdef __cplusplus
}
#endif
