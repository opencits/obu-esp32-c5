#pragma once

#include "esp_err.h"

void register_ota_cmd(void);
void ota_init(void);

/* Marks the running app partition valid and cancels the anti-brick rollback
 * timer armed by ota_init(). Shared by `ota -C` and the wire-protocol
 * FW_CONFIRM handler (fw_update.c). */
esp_err_t ota_confirm_running_partition(void);
