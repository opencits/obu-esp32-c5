#pragma once

#include <stddef.h>
#include <stdint.h>

#define COMM_VERSION "0.4"

#define COMM_MAGIC 0x7384

#define COMM_FRAME_PING 1
#define COMM_FRAME_PONG 2

#define COMM_FRAME_CMD_REBOOT 20
#define COMM_FRAME_CMD_CONFIG 21
#define COMM_FRAME_CMD_START  22
#define COMM_FRAME_CMD_STOP   23

#define COMM_FRAME_CMD_GOOD 28
#define COMM_FRAME_CMD_FAIL 29

#define COMM_FRAME_RX    30
#define COMM_FRAME_TX    31

#define COMM_MAX_BUF_SIZE (64*1024)

typedef struct
{
  uint16_t type;
  uint16_t length;
} __packed TypeLengthHeader;

typedef struct
{
  uint16_t magic;
  uint16_t seqNo;
  TypeLengthHeader xferHeader;
} __packed CommHeader;

typedef struct {
  uint8_t channel;
  uint8_t rate;
  int8_t rssi_power;
  int8_t noise;
  uint32_t seconds;
  uint32_t microseconds;
  uint16_t frameLen;
} __packed CommFrameHeader;

/* Per-channel state for obu_comm_feed(); USB and BLE each keep their own
 * instance over their own buffer since the two links reassemble independently. */
typedef struct {
    uint8_t *buf;
    size_t   buf_size;
    int      end;
    int      payloadType;
    int      payloadLen;
    int      seqNo;
} obu_comm_reassembler_t;

void obu_comm_reassembler_init(obu_comm_reassembler_t *r, uint8_t *buf, size_t buf_size);

/* Called once per fully reassembled frame; payload points into the reassembler's buffer. */
typedef void (*obu_comm_frame_fn)(uint16_t type, uint16_t length, uint8_t *payload, void *ctx);

/* Feeds newly received bytes into the reassembler, invoking on_frame for each
 * complete CommHeader-framed message found (possibly more than one per call). */
void obu_comm_feed(obu_comm_reassembler_t *r, const uint8_t *data, int len,
                    obu_comm_frame_fn on_frame, void *ctx);

/* Same as obu_comm_feed, but for callers that already wrote `len` new bytes
 * directly at r->buf + r->end (e.g. a UART read straight into the reassembly
 * buffer) — advances and parses without copying. */
void obu_comm_feed_inplace(obu_comm_reassembler_t *r, int len,
                            obu_comm_frame_fn on_frame, void *ctx);

/* Sends a CommHeader-framed reply back to the originating transport. */
typedef void (*obu_comm_reply_fn)(uint16_t type, const uint8_t *payload, uint16_t len, void *ctx);

/* Dispatches PING/CMD_REBOOT/CMD_CONFIG, replying via `reply`. Transport-agnostic;
 * call from a dedicated task, never from a stack callback (CMD_REBOOT blocks+restarts). */
void obu_comm_process_frame(uint16_t type, uint16_t length, uint8_t *payload,
                             obu_comm_reply_fn reply, void *ctx);

bool obu_comm_ble_connected(void);
bool obu_comm_usb_connected(void);