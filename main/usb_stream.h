#pragma once

#include <stdint.h>
#include "obu_wifi.h"

void usb_stream_init(void);
void usb_stream_publish_packet(const wifi_packet_t *pkt);
void usb_stream_send_ping(void);
void usb_stream_publish_frame(uint16_t type, const uint8_t *payload, uint16_t len);

/**
 * Start the USB RX config channel task.
 * Listens for "CFG:key=value\n" commands from the setup wizard and writes
 * them to NVS. Acknowledges with "CFG_OK:key\n" or "CFG_ERR:msg\n".
 */
void usb_config_rx_start(void);
