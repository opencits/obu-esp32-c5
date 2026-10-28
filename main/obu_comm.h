#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define COMM_VERSION "0.8"

#define COMM_MAGIC 0x7384

#define COMM_FRAME_PING 1
#define COMM_FRAME_PONG 2

#define COMM_FRAME_CMD_REBOOT   20
#define COMM_FRAME_CMD_CONFIG   21
#define COMM_FRAME_CMD_START    22
#define COMM_FRAME_CMD_STOP     23

#define COMM_FRAME_CMD_GOOD 28
#define COMM_FRAME_CMD_FAIL 29

#define COMM_FRAME_RX    30
#define COMM_FRAME_TX    31

/* Firmware push over the existing wire protocol (BLE/USB), into esp_ota_*
 * directly — bypasses esp_https_ota, whose WiFi STA never actually
 * associates in this firmware. See fw_update.cpp. */
#define COMM_FRAME_FW_BEGIN    40
#define COMM_FRAME_FW_DATA     41
#define COMM_FRAME_FW_END      42
#define COMM_FRAME_FW_ABORT    43
#define COMM_FRAME_FW_CONFIRM  44
#define COMM_FRAME_FW_ACK      45
#define COMM_FRAME_FW_NAK      46

/* ETSI DCC channel-busy-ratio (CBR), see docs/esp_dcc_measurements.md. READ
 * gets one CBR_IND immediately; START/STOP gate a periodic CBR_IND push
 * (USB/BLE, whichever is connected) driven by WifiController::cbrIndStart/cbrIndStop. */
#define COMM_FRAME_CMD_CBR_READ   50
#define COMM_FRAME_CMD_CBR_START  51
#define COMM_FRAME_CMD_CBR_STOP   52
#define COMM_FRAME_CBR_IND        53

#define COMM_FRAME_TX_LOOP_START  60
#define COMM_FRAME_TX_LOOP_STOP   61
#define COMM_FRAME_TX_LOOP_STATUS 62  /* request (empty) and reply (JSON fps/bps/err/interval/len) */

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
  uint16_t channel;  // in MHz, zero is valid (default used)
  uint16_t rate;     // in kbps, zero is valid (default used)
  int8_t rssi_power;
  int8_t noise;
  uint32_t seconds;
  uint32_t microseconds;
  uint16_t frameLen;
} __packed CommFrameHeader;

/* FW_BEGIN payload. Network byte order (htonl/ntohl), same convention as
 * every other multi-byte field in this protocol (CommHeader, CommFrameHeader) —
 * fields below are wire-format and must be swapped explicitly on this
 * little-endian chip, never memcpy'd straight into use. */
typedef struct {
  uint32_t totalSize;
} __packed FwBeginPayload;

/* FW_DATA payload prefix; raw image bytes follow. */
typedef struct {
  uint32_t offset;
} __packed FwDataHeader;

/* FW_ACK/FW_NAK payload: bytes written so far (ACK) or expected offset to
 * resume from (NAK); 0 on a NAK unrelated to offset (e.g. not active). */
typedef struct {
  uint32_t value;
} __packed FwAckPayload;

/* TX_LOOP_START payload, network byte order; replies CMD_GOOD, or CMD_FAIL if
 * out of range. Frame = fixed 34-byte ITS header + random payload (cmd_tx.cpp). */
typedef struct {
  uint32_t intervalMs;   // 1..3600000 ms, 0 = 1000; tick-quantized (10 ms at HZ=100)
  uint32_t burstCount;   // frames per interval, 1..32; 0 = keep current (1 at boot)
  uint32_t frameLen;     // random payload bytes after the header, 0..1466; 0 = keep current (128 at boot)
  int32_t  txPower;      // 2..20 dBm, 0 = keep current
  uint32_t datarate;     // kbps (3000..27000, 802.11p 10 MHz), 0 = keep current
} __packed TxStartPayload;

/* Upper bound keeps pdMS_TO_TICKS' 32-bit ms*HZ product from overflowing. */
#define CBR_IND_MAX_INTERVAL_MS 3600000u

/* CBR_START payload: periodic CBR_IND push interval, 1..CBR_IND_MAX_INTERVAL_MS. */
typedef struct {
  uint32_t intervalMs;
} __packed CbrStartPayload;

/* CBR_IND payload: percent fields are milli-percent (percent*1000, so 0..100000)
 * to keep floats off the wire, same convention as every other multi-byte field
 * here. validSamples==0 means no CBR window has been measured yet. */
typedef struct {
  uint32_t avgPercentMilli;
  uint32_t latestPercentMilli;
  uint32_t validSamples;
  uint32_t rejectedSamples;
} __packed CbrIndPayload;

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

#ifdef __cplusplus
}
#endif
