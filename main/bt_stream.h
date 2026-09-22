#pragma once

#include <stdint.h>
#include "obu_wifi.h"

void bt_stream_init(void);
void bt_stream_publish_packet(const wifi_packet_t *pkt);
void bt_stream_publish_frame(uint16_t type, const uint8_t *payload, uint16_t len);
