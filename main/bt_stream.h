#pragma once

#include <stdint.h>
#include "cmd_sniffer.h"

void bt_stream_init(void);
void bt_stream_publish_packet(const sniffer_packet_t *pkt);

/** Stop BLE advertising (frees the 2.4 GHz radio for WiFi scanning). */
void bt_coex_pause(void);
/** Restart BLE advertising after bt_stream_pause(). */
void bt_coex_resume(void);
