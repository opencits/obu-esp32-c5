#pragma once

#include <stdint.h>

#include "obu_comm.h"

/* Wire-protocol firmware push (FW_BEGIN/DATA/END/ABORT/CONFIRM), into the
 * non-running OTA partition via esp_ota_*. Dispatched from obu_comm.c,
 * shared by BLE and USB transports (each has its own obu_comm_reply_fn). */

void fw_update_init(void);

void fw_update_begin(const uint8_t *payload, uint16_t length, obu_comm_reply_fn reply, void *ctx);
void fw_update_data(const uint8_t *payload, uint16_t length, obu_comm_reply_fn reply, void *ctx);
void fw_update_end(obu_comm_reply_fn reply, void *ctx);
void fw_update_abort(obu_comm_reply_fn reply, void *ctx);
void fw_update_confirm(obu_comm_reply_fn reply, void *ctx);
