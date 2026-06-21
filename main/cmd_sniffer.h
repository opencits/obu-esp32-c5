#pragma once

#include "obu_comm.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t *payload;
    uint16_t length;
} sniffer_packet_t;

/**
 * @brief Supported Sniffer Interface
 *
 */
typedef enum {
    SNIFFER_INTF_UNKNOWN = 0,
    SNIFFER_INTF_WLAN, /*!< WLAN interface */
} sniffer_intf_t;

/**
 * @brief WLAN Sniffer Filter
 *
 */
typedef enum {
    SNIFFER_WLAN_FILTER_MGMT = 0, /*!< MGMT */
    SNIFFER_WLAN_FILTER_CTRL,     /*!< CTRL */
    SNIFFER_WLAN_FILTER_DATA,     /*!< DATA */
    SNIFFER_WLAN_FILTER_MISC,     /*!< MISC */
    SNIFFER_WLAN_FILTER_MPDU,     /*!< MPDU */
    SNIFFER_WLAN_FILTER_AMPDU,    /*!< AMPDU */
    SNIFFER_WLAN_FILTER_FCSFAIL,  /*!< When this bit is set, the hardware will receive packets for which frame check sequence failed */
    SNIFFER_WLAN_FILTER_MAX
} sniffer_wlan_filter_t;

void sniffer_init(void);
void register_sniffer_cmd(void);
void sniffer_autostart(void);

/* Time-slice the radio with BLE: pause releases the RF (promisc off, 802.11p
 * PHY off, esp_wifi_stop), resume re-applies the sniffer's state. The sniffer
 * task keeps running between calls — it just sees no RX callbacks while paused.
 */
void sniffer_pause(void);
void sniffer_resume(void);


#ifdef __cplusplus
}
#endif
