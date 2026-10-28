#pragma once

#include "obu_comm.h"

#ifdef __cplusplus
extern "C" {
#endif

void register_tx_cmd(void);

void start_tx_loop(uint8_t *frame, uint16_t frame_len, obu_comm_reply_fn reply, void *ctx);
void stop_tx_loop(obu_comm_reply_fn reply, void *ctx);
void status_tx_loop(obu_comm_reply_fn reply, void *ctx);

#ifdef __cplusplus
}
#endif
