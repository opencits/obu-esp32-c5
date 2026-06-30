#pragma once

#include "esp_event.h"

#ifdef __cplusplus
extern "C" {
#endif

ESP_EVENT_DECLARE_BASE(CONFIG_EVENT_BASE);

// CONFIG_EVENT_BASE has config_index_t as events

ESP_EVENT_DECLARE_BASE(WIFI_EVENT_BASE);

enum WifiEvent
{
    WIFI_STARTED,
    WIFI_STOPPED,
    WIFI_RECEIVED_PACKET,
    WIFI_TRANSMIT_PACKET
};

ESP_EVENT_DECLARE_BASE(COMM_EVENT_BASE);

enum CommEvent
{
    COMM_CONNECTED,
    COMM_DISCONNECTED
};

#ifdef __cplusplus
}
#endif
