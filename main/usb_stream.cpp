#include "usb_stream.h"

#include <cstdint>
#include <cstdio>

#include "driver/usb_serial_jtag.h"
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

constinit UsbStream UsbStream::s_instance;

uint16_t UsbStream::nextSeqNo()
{
    uint16_t seqNo = seqNo_++;
    if (seqNo_ == 0) seqNo_ = 1;
    return seqNo;
}

void UsbStream::writeFrame(uint16_t type, const void *payload, uint16_t len)
{
    if (!ready_) return;

    CommHeader hdr;
    hdr.magic = htons(COMM_MAGIC);
    hdr.xferHeader.type = htons(type);
    hdr.xferHeader.length = htons(len);

    xSemaphoreTake(txLock_, portMAX_DELAY);
    hdr.seqNo = htons(nextSeqNo());
    usb_serial_jtag_write_bytes(&hdr, sizeof(hdr), pdMS_TO_TICKS(50));
    if (len > 0) {
        usb_serial_jtag_write_bytes(payload, len, pdMS_TO_TICKS(50));
    }
    xSemaphoreGive(txLock_);
}

void UsbStream::init()
{
    if (ready_) return;

    if (!txLock_) txLock_ = xSemaphoreCreateMutex();
    if (!txLock_) {
        ESP_LOGE(TAG, "create tx lock failed");
        return;
    }

    usb_serial_jtag_driver_config_t cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    cfg.tx_buffer_size = 4096;
    cfg.rx_buffer_size = 4096;  /* was 256 — FW_DATA update chunks can arrive faster than
                                 * rxTask's 50ms poll loop can drain a small ring */

    esp_err_t err = usb_serial_jtag_driver_install(&cfg);
    if (err == ESP_OK || err == ESP_ERR_INVALID_STATE) {
        ready_ = true;
    } else {
        ESP_LOGE(TAG, "usb_serial_jtag_driver_install failed: %s", esp_err_to_name(err));
    }
    seqNo_ = 1;
}

void UsbStream::publishPacket(const wifi_packet_t *pkt)
{
    if (!ready_) init();
    if (!ready_) return;

    writeFrame(COMM_FRAME_RX, pkt->payload, pkt->length);

    LedController::instance().pulseUsb();
}

/* ── USB RX config channel ──────────────────────────────────────────────
 * Reassembly and command dispatch (PING/CMD_REBOOT/CMD_CONFIG) live in
 * obu_comm.cpp, shared with the BLE write path in bt_stream.cpp. This file
 * just feeds bytes in and writes replies back out over USB-CDC.
 * ──────────────────────────────────────────────────────────────────────*/
void UsbStream::reply(uint16_t type, const uint8_t *payload, uint16_t len, void *ctx)
{
    static_cast<UsbStream *>(ctx)->writeFrame(type, payload, len);
}

void UsbStream::onFrame(uint16_t type, uint16_t length, uint8_t *payload, void *ctx)
{
    obu_comm_process_frame(type, length, payload, reply, ctx);
}

void UsbStream::rxTaskEntry(void *arg)
{
    static_cast<UsbStream *>(arg)->rxTask();
}

void UsbStream::rxTask()
{
    obu_comm_reassembler_t reassembler;
    obu_comm_reassembler_init(&reassembler, commBuffer_, COMM_MAX_BUF_SIZE);
    int coex_resume_count = 0;

    for (;;)
    {
        // append to any existing data
        //  | x x x x x x x x | . . . . . . . . . . . . . |
        //  ^ Buffer          ^ End                      ^ MAX
        uint8_t* recvBufferStart = commBuffer_ + reassembler.end;
        int numRead = usb_serial_jtag_read_bytes(recvBufferStart, COMM_MAX_BUF_SIZE - reassembler.end, pdMS_TO_TICKS(50));
        // when no more connection - release our coex hold after the idle period
        if (coex_resume_count > 0)
        {
            coex_resume_count--;
            if (coex_resume_count == 0)
            {
                WifiController::instance().coexRelease();
            }
        }
        if (numRead <= 0) continue;

        // got data - hold WIFI/BLE COEX poll; count>0 means we already hold it
        if (coex_resume_count == 0) WifiController::instance().coexHold();
        coex_resume_count = 20*30;  // Resume after 30sec

        /* numRead bytes were already read directly into recvBufferStart, i.e.
         * straight into the reassembler's own buffer — just advance and parse. */
        obu_comm_feed_inplace(&reassembler, numRead, onFrame, this);
    }
}

void UsbStream::startRx()
{
    xTaskCreate(rxTaskEntry, "usb_cfg_rx", 4096, this, 3, NULL);
}
