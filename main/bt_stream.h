#pragma once

#include <stdint.h>
#include "obu_wifi.h"

void bt_stream_init(void);
void bt_stream_publish_packet(const wifi_packet_t *pkt);
