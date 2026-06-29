#pragma once

#include "obu_comm.h"

#ifdef __cplusplus
extern "C" {
#endif

void register_tx_cmd(void);

bool set_tx_power(int8_t txpower);

#ifdef __cplusplus
}
#endif
