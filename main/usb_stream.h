#pragma once

#include <stdint.h>
#include "cmd_sniffer.h"

void usb_stream_init(void);
void usb_stream_publish_packet(const sniffer_packet_t *pkt);
void usb_stream_send_ping(void);

/**
 * Start the USB RX config channel task.
 * Listens for "CFG:key=value\n" commands from the setup wizard and writes
 * them to NVS. Acknowledges with "CFG_OK:key\n" or "CFG_ERR:msg\n".
 */
void usb_config_rx_start(void);
