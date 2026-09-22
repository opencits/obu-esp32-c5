#include "usb_stream.h"

#include <stdint.h>
#include <stdio.h>

#include "driver/usb_serial_jtag.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "nvs.h"

#include "bt_stream.h"
#include "cmd_sniffer.h"
#include "esp_coexist.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "obu_wifi.h"
#include "obu_comm.h"
#include "led.h"

static const char TAG[] = "USB_STREAM";
static bool s_ready;

static uint16_t s_comm_seqno;
/* wifi_task, the rx task and tCbrInd all write frames; hold this across
 * seqno + header + payload so frames never interleave on the wire. */
static SemaphoreHandle_t s_tx_lock;
static uint8_t s_commBuffer[COMM_MAX_BUF_SIZE];

static uint16_t usb_inc_seqno()
{
    uint16_t seqNo = s_comm_seqno++;
    if (s_comm_seqno == 0) s_comm_seqno = 1;
    return seqNo;
}

static void usb_write_frame(uint16_t type, const void *payload, uint16_t len)
{
    if (!s_ready) return;

    CommHeader hdr;
    hdr.magic = htons(COMM_MAGIC);
    hdr.xferHeader.type = htons(type);
    hdr.xferHeader.length = htons(len);

    xSemaphoreTake(s_tx_lock, portMAX_DELAY);
    hdr.seqNo = htons(usb_inc_seqno());
    usb_serial_jtag_write_bytes(&hdr, sizeof(hdr), pdMS_TO_TICKS(50));
    if (len > 0) {
        usb_serial_jtag_write_bytes(payload, len, pdMS_TO_TICKS(50));
    }
    xSemaphoreGive(s_tx_lock);
}

void usb_stream_init(void)
{
    if (s_ready) return;

    if (!s_tx_lock) s_tx_lock = xSemaphoreCreateMutex();
    if (!s_tx_lock) {
        ESP_LOGE(TAG, "create tx lock failed");
        return;
    }

    usb_serial_jtag_driver_config_t cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    cfg.tx_buffer_size = 4096;
    cfg.rx_buffer_size = 4096;  /* was 256 — FW_DATA update chunks can arrive faster than
                                 * usb_config_rx_task's 50ms poll loop can drain a small ring */

    esp_err_t err = usb_serial_jtag_driver_install(&cfg);
    if (err == ESP_OK || err == ESP_ERR_INVALID_STATE) {
        s_ready = true;
    } else {
        ESP_LOGE(TAG, "usb_serial_jtag_driver_install failed: %s", esp_err_to_name(err));
    }
    s_comm_seqno = 1;
}

void usb_stream_publish_packet(const wifi_packet_t *pkt)
{
    if (!s_ready) usb_stream_init();
    if (!s_ready) return;

    usb_write_frame(COMM_FRAME_RX, pkt->payload, pkt->length);

    led_pulse_usb();
}

void usb_stream_send_ping(void)
{
    usb_write_frame(COMM_FRAME_PING, NULL, 0);
}

/* ── USB RX config channel ──────────────────────────────────────────────
 * Reassembly and command dispatch (PING/CMD_REBOOT/CMD_CONFIG) live in
 * obu_comm.c, shared with the BLE write path in bt_stream.c. This file
 * just feeds bytes in and writes replies back out over USB-CDC.
 * ──────────────────────────────────────────────────────────────────────*/
static void usb_reply(uint16_t type, const uint8_t *payload, uint16_t len, void *ctx)
{
    (void)ctx;
    usb_write_frame(type, payload, len);
}

void usb_stream_publish_frame(uint16_t type, const uint8_t *payload, uint16_t len)
{
    usb_write_frame(type, payload, len);
}

static void on_usb_frame(uint16_t type, uint16_t length, uint8_t *payload, void *ctx)
{
    (void)ctx;
    obu_comm_process_frame(type, length, payload, usb_reply, NULL);
}

static void usb_config_rx_task(void *arg)
{
    obu_comm_reassembler_t reassembler;
    obu_comm_reassembler_init(&reassembler, s_commBuffer, COMM_MAX_BUF_SIZE);
    int coex_resume_count = 0;

    for (;;)
    {
        // append to any existing data
        //  | x x x x x x x x | . . . . . . . . . . . . . |
        //  ^ Buffer          ^ End                      ^ MAX
        uint8_t* recvBufferStart = s_commBuffer + reassembler.end;
        int numRead = usb_serial_jtag_read_bytes(recvBufferStart, COMM_MAX_BUF_SIZE - reassembler.end, pdMS_TO_TICKS(50));
        // when no more connection - restart WIFI/BLE COEX poll after paused
        if (coex_resume_count > 0)
        {
            coex_resume_count--;
            if (coex_resume_count == 0)
            {
                wifi_coex_enable();
            }
        }
        if (numRead <= 0) continue;

        // got data - stop WIFI/BLE COEX poll
        wifi_coex_disable();
        coex_resume_count = 20*30;  // Resume after 30sec

        /* numRead bytes were already read directly into recvBufferStart, i.e.
         * straight into the reassembler's own buffer — just advance and parse. */
        obu_comm_feed_inplace(&reassembler, numRead, on_usb_frame, NULL);
    }
}

void usb_config_rx_start(void)
{
    xTaskCreate(usb_config_rx_task, "usb_cfg_rx", 4096, NULL, 3, NULL);
}
