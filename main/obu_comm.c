#include "obu_comm.h"

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "config.h"
#include "obu_wifi.h"

static const char TAG[] = "OBU_COMM";

static bool BLE_connected = false;
static bool USB_connected = false;

void obu_comm_reassembler_init(obu_comm_reassembler_t *r, uint8_t *buf, size_t buf_size)
{
    r->buf = buf;
    r->buf_size = buf_size;
    r->end = 0;
    r->payloadType = -1;
    r->payloadLen = 0;
    r->seqNo = 0;
}

static void parse(obu_comm_reassembler_t *r, obu_comm_frame_fn on_frame, void *ctx)
{
    while (true)
    {
        int available = r->end;
        if (r->payloadType < 0)
        {
            // no valid header yet detected
            if (available < (int)sizeof(CommHeader))
            {
                break;
            }

            const CommHeader* hdr = (const CommHeader*)r->buf;
            if (ntohs(hdr->magic) != COMM_MAGIC)
            {
                ESP_LOGW(TAG, "Invalid Magic %x [%d]", ntohs(hdr->magic), r->seqNo);
                r->payloadType = -1;
                r->payloadLen = 0;
                r->end = 0;
                break;
            }

            uint16_t seqNo = ntohs(hdr->seqNo);
            r->payloadType = ntohs(hdr->xferHeader.type);
            r->payloadLen  = ntohs(hdr->xferHeader.length);
            if (r->payloadLen > (int)r->buf_size)
            {
                ESP_LOGW(TAG, "Length exceeded [%d:%d]", seqNo, r->seqNo);
                r->payloadType = -1;
                r->payloadLen = 0;
                r->end = 0;
                break;
            }

            // checking SeqNo
            if (r->seqNo != 0 && (r->seqNo + 1) != seqNo)
            {
                ESP_LOGI(TAG, "SeqNo check failed [%d:%d]", seqNo, r->seqNo);
            }
            r->seqNo = seqNo;

            int thisFrameLen = r->payloadLen + (int)sizeof(CommHeader);
            if (available > thisFrameLen)
            {
                // more than one frame available
                on_frame((uint16_t)r->payloadType, (uint16_t)r->payloadLen,
                         r->buf + sizeof(CommHeader), ctx);
                r->payloadType = -1;
                r->payloadLen = 0;
                r->end = 0;

                // move remainder to buffer head
                int remaining = available - thisFrameLen;
                memmove(r->buf, r->buf + thisFrameLen, remaining);
                r->end = remaining;
            }
            else if (available == thisFrameLen)
            {
                // received full frame
                on_frame((uint16_t)r->payloadType, (uint16_t)r->payloadLen,
                         r->buf + sizeof(CommHeader), ctx);
                r->payloadType = -1;
                r->payloadLen = 0;
                r->end = 0;
            }
        }
        else
        {
            // at least header already read
            // check if read fetched more than one frame
            int thisFrameLen = r->payloadLen + (int)sizeof(CommHeader);
            if (available > thisFrameLen)
            {
                on_frame((uint16_t)r->payloadType, (uint16_t)r->payloadLen,
                         r->buf + sizeof(CommHeader), ctx);
                r->payloadType = -1;
                r->payloadLen = 0;

                // move remainder to buffer head
                int remaining = available - thisFrameLen;
                memmove(r->buf, r->buf + thisFrameLen, remaining);
                r->end = remaining;
            }
            // check for full frame
            else if (available == thisFrameLen)
            {
                on_frame((uint16_t)r->payloadType, (uint16_t)r->payloadLen,
                         r->buf + sizeof(CommHeader), ctx);
                r->payloadType = -1;
                r->payloadLen = 0;
                r->end = 0;
                break;
            }
            else
            {
                break;
            }
        }
    }
}

void obu_comm_feed(obu_comm_reassembler_t *r, const uint8_t *data, int len,
                    obu_comm_frame_fn on_frame, void *ctx)
{
    if (len <= 0) return;
    if (r->end + len > (int)r->buf_size) {
        ESP_LOGW(TAG, "feed overflow, resetting reassembler");
        r->end = 0;
        r->payloadType = -1;
        r->payloadLen = 0;
        return;
    }
    memcpy(r->buf + r->end, data, len);
    r->end += len;
    parse(r, on_frame, ctx);
}

void obu_comm_feed_inplace(obu_comm_reassembler_t *r, int len,
                            obu_comm_frame_fn on_frame, void *ctx)
{
    if (len <= 0) return;
    if (r->end + len > (int)r->buf_size) {
        ESP_LOGW(TAG, "feed_inplace overflow, resetting reassembler");
        r->end = 0;
        r->payloadType = -1;
        r->payloadLen = 0;
        return;
    }
    r->end += len;
    parse(r, on_frame, ctx);
}

/* ── Config command sub-protocol ────────────────────────────────────────
 * Protocol: wizard sends  "CFG:key=value\n"
 *           device replies "CFG_OK:key\n" or "CFG_ERR:message\n"
 * Keys map directly to NVS keys used by config.c (namespace "its").
 * ──────────────────────────────────────────────────────────────────────*/

/* Map key name → config_index_t */
static int cfg_key_to_index(const char *key)
{
    /* string keys */
    if (!strcmp(key, "nodeid"))    return CONFIG_INDEX_NODEID;
    if (!strcmp(key, "wifi1ssid")) return CONFIG_INDEX_WIFI1_SSID;
    if (!strcmp(key, "wifi1pass")) return CONFIG_INDEX_WIFI1_PASS;
    if (!strcmp(key, "wifi2ssid")) return CONFIG_INDEX_WIFI2_SSID;
    if (!strcmp(key, "wifi2pass")) return CONFIG_INDEX_WIFI2_PASS;
    if (!strcmp(key, "wifiip"))    return CONFIG_INDEX_WIFI_IP;
    if (!strcmp(key, "wifinm"))    return CONFIG_INDEX_WIFI_NM;
    if (!strcmp(key, "wifigw"))    return CONFIG_INDEX_WIFI_GW;
    if (!strcmp(key, "wifidns"))   return CONFIG_INDEX_WIFI_DNS;
    /* u8 keys */
    if (!strcmp(key, "wifiopen"))  return CONFIG_INDEX_WIFI_OPEN;
    if (!strcmp(key, "connmode"))  return CONFIG_INDEX_CONN_MODE;
    if (!strcmp(key, "sniffmode")) return CONFIG_INDEX_SNIFF_MODE;
    if (!strcmp(key, "led"))       return CONFIG_INDEX_LED_BRIGHTNESS;
    if (!strcmp(key, "channel"))   return CONFIG_INDEX_AUTOSTART_CHAN;
    /* u32 keys */
    if (!strcmp(key, "sniffms"))   return CONFIG_INDEX_SNIFF_MS;
    if (!strcmp(key, "wifims"))    return CONFIG_INDEX_WIFI_MS;
    if (!strcmp(key, "txpower"))   return CONFIG_INDEX_WIFI_TX_POWER;
    if (!strcmp(key, "loop"))      return CONFIG_INDEX_WIFI_LOOP;

    return -1;
}

static bool is_u8_key(const char *key)
{
    return !strcmp(key,"wifiopen") || !strcmp(key,"connmode") || !strcmp(key,"sniffmode") ||
           !strcmp(key,"led") || !strcmp(key,"loop");
}
static bool is_u32_key(const char *key)
{
    return !strcmp(key,"sniffms") || !strcmp(key,"wifims") || !strcmp(key,"channel");
}
static bool is_i8_key(const char *key)
{
    return !strcmp(key,"txpower");
}

static void on_ping(obu_comm_reply_fn reply, void *ctx)
{
    char buffer[1024];

    wifi_stats_t stats;
    wifi_get_stats(&stats);

    int len = snprintf(buffer, 1024, "{\"rx\":%lu", stats.rx);
    len += snprintf(buffer + len, 1024 - len, ",\"rxErr\":%lu", stats.rxErr);
    len += snprintf(buffer + len, 1024 - len, ",\"tx\":%lu", stats.tx);
    len += snprintf(buffer + len, 1024 - len, ",\"txErr\":%lu", stats.txErr);
    len += snprintf(buffer + len, 1024 - len, ",\"version\":\"%s\"", COMM_VERSION);
    len += snprintf(buffer + len, 1024 - len, "}");

    reply(COMM_FRAME_PONG, (const uint8_t*)buffer, len, ctx);
}

static void on_config(uint8_t *frame, uint16_t frame_len, obu_comm_reply_fn reply, void *ctx)
{
    char line[1024];
    size_t n = (frame_len < 1023) ? frame_len : 1023;
    memcpy(line, frame, n);
    line[n] = '\0';

    esp_err_t err;
    /* Expect "CFG:key=value" */
    if (strncmp(line, "CFG:", 4) != 0) {
        ESP_LOGW(TAG, "CFG: no header '%s'", line);
        reply(COMM_FRAME_CMD_FAIL, NULL, 0, ctx);
        return;
    }

    char *kv = line + 4;
    char *eq = strchr(kv, '=');
    if (eq)
    {
        // WRITE - command
        *eq = '\0';
        char *key = kv, *val = eq + 1;

        int idx = cfg_key_to_index(key);
        if (idx < 0) {
            ESP_LOGW(TAG, "CFG: unknown key '%s'", key);
            reply(COMM_FRAME_CMD_FAIL, NULL, 0, ctx);
            return;
        }

        if (is_u8_key(key)) {
            uint8_t v = (uint8_t)atoi(val);
            err = config_set_u8((config_index_t)idx, v);
        } else if (is_u32_key(key)) {
            uint32_t v = (uint32_t)atol(val);
            err = config_set_u32((config_index_t)idx, v);
        } else if (is_i8_key(key)) {
            int8_t v = (int8_t)atoi(val);
            err = config_set_i8((config_index_t)idx, v);
        } else {
            err = config_set_str((config_index_t)idx, val);
        }
        if (err != ESP_OK) {
            reply(COMM_FRAME_CMD_FAIL, NULL, 0, ctx);
            return;
        }
    }
    // READ - command
    int idx = cfg_key_to_index(kv);
    if (idx < 0) {
        ESP_LOGW(TAG, "CFG: unknown key '%s'", kv);
        reply(COMM_FRAME_CMD_FAIL, NULL, 0, ctx);
        return;
    }
    size_t len = 0;
    if (is_u8_key(kv)) {
        uint8_t v;
        err = config_get_u8((config_index_t)idx, &v);
        len = snprintf(line, 1023, "%u", v);
    } else if (is_i8_key(kv)) {
        int8_t v;
        err = config_get_i8((config_index_t)idx, &v);
        len = snprintf(line, 1023, "%d", v);
    } else if (is_u32_key(kv)) {
        uint32_t v;
        err = config_get_u32((config_index_t)idx, &v);
        len = snprintf(line, 1023, "%lu", v);
    } else {
        len = 1023;
        err = config_get_str((config_index_t)idx, line, &len);
    }
    if (err == ESP_OK) {
        reply(COMM_FRAME_CMD_CONFIG, (const uint8_t*)line, len, ctx);
    } else {
        reply(COMM_FRAME_CMD_CONFIG, NULL, 0, ctx);
    }
}

static void on_start(uint8_t *frame, uint16_t frame_len, obu_comm_reply_fn reply, void *ctx)
{
    if (frame_len != 3) {
        ESP_LOGW(TAG, "Invalid START frame length: %d", frame_len);
        reply(COMM_FRAME_CMD_FAIL, NULL, 0, ctx);
        return;
    }
    if (frame[0] == 'U' && frame[1] == 'S' && frame[2] == 'B') {
        USB_connected = true;
        reply(COMM_FRAME_CMD_GOOD, NULL, 0, ctx);
        return;
    }
    else if (frame[0] == 'B' && frame[1] == 'L' && frame[2] == 'E') {
        BLE_connected = true;
        reply(COMM_FRAME_CMD_GOOD, NULL, 0, ctx);
        return;
    }
    ESP_LOGW(TAG, "Unrecognized START: %02x-%02x-%02x", frame[0], frame[1], frame[2]);
    reply(COMM_FRAME_CMD_FAIL, NULL, 0, ctx);
}

static void on_stop(uint8_t *frame, uint16_t frame_len, obu_comm_reply_fn reply, void *ctx)
{
    if (frame_len != 3) {
        ESP_LOGW(TAG, "Invalid STOP frame length: %d", frame_len);
        reply(COMM_FRAME_CMD_FAIL, NULL, 0, ctx);
        return;
    }
    if (frame[0] == 'U' && frame[1] == 'S' && frame[2] == 'B') {
        USB_connected = false;
        reply(COMM_FRAME_CMD_GOOD, NULL, 0, ctx);
        return;
    }
    else if (frame[0] == 'B' && frame[1] == 'L' && frame[2] == 'E') {
        BLE_connected = false;
        reply(COMM_FRAME_CMD_GOOD, NULL, 0, ctx);
        return;
    }
    ESP_LOGW(TAG, "Unrecognized STOP: %02x-%02x-%02x", frame[0], frame[1], frame[2]);
    reply(COMM_FRAME_CMD_FAIL, NULL, 0, ctx);
}

void obu_comm_process_frame(uint16_t type, uint16_t length, uint8_t *payload,
                             obu_comm_reply_fn reply, void *ctx)
{
    switch (type) {
        case COMM_FRAME_PING:
            on_ping(reply, ctx);
            break;
        case COMM_FRAME_PONG:
            ESP_LOGI(TAG, "Received PONG from host");
            break;
        case COMM_FRAME_CMD_REBOOT:
            reply(COMM_FRAME_CMD_GOOD, NULL, 0, ctx);
            vTaskDelay(pdMS_TO_TICKS(150));
            esp_restart();
            break;
        
        case COMM_FRAME_CMD_START:
            on_start(payload, length, reply, ctx);
            break;
        case COMM_FRAME_CMD_STOP:
            on_stop(payload, length, reply, ctx);
            break;
        
        case COMM_FRAME_CMD_CONFIG:
            on_config(payload, length, reply, ctx);
            break;
        case COMM_FRAME_TX:
            wifi_tx_frame(payload, length);
            break;

        default:
            ESP_LOGW(TAG, "Received unknown frame type: %d", type);
            break;
    }
}

bool obu_comm_ble_connected(void)
{
    return BLE_connected;
}

bool obu_comm_usb_connected(void)
{
    return USB_connected;
}
