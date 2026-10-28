#include "bt_stream.h"

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_coexist.h"
#include "esp_wifi.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "driver/usb_serial_jtag.h"

#include "obu_wifi.h"
#include "obu_comm.h"
#include "led.h"

/* Private NimBLE API: forces the controller's public address. Used here so
 * old Android stacks (Galaxy S6 / AOSP) accept the advertiser — they ignore
 * adv from random-static addresses they have not paired with.
 */
extern "C" int ble_hs_id_set_pub(const uint8_t *pub_addr);

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_hs_mbuf.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char TAG[] = "ble";
static const char DEVICE_NAME[] = "OBU-ESP32";

#define FRAME_QUEUE_DEPTH      16

/* Vendor UUIDs (NimBLE stores them little-endian, so the bytes below are
 * reversed vs. the canonical string form):
 *   Service:  b6e57e90-12d8-4a47-9b21-3f0000000001
 *   Notify :  b6e57e90-12d8-4a47-9b21-3f0000000002
 *   Config :  b6e57e90-12d8-4a47-9b21-3f0000000003 (8 B = 4×u16 LE)
 */
static const ble_uuid128_t SVC_UUID = BLE_UUID128_INIT(
    0x01, 0x00, 0x00, 0x00, 0x00, 0x3f, 0x21, 0x9b,
    0x47, 0x4a, 0xd8, 0x12, 0x90, 0x7e, 0xe5, 0xb6);

static const ble_uuid128_t CHR_UUID = BLE_UUID128_INIT(
    0x02, 0x00, 0x00, 0x00, 0x00, 0x3f, 0x21, 0x9b,
    0x47, 0x4a, 0xd8, 0x12, 0x90, 0x7e, 0xe5, 0xb6);

static const ble_uuid128_t CFG_UUID = BLE_UUID128_INIT(
    0x03, 0x00, 0x00, 0x00, 0x00, 0x3f, 0x21, 0x9b,
    0x47, 0x4a, 0xd8, 0x12, 0x90, 0x7e, 0xe5, 0xb6);

typedef struct {
    uint16_t total_len;        /* HEADER_LEN + payload_len */
    uint8_t  data[];           /* preformatted ITS5 frame */
} qframe_t;

/* One inbound GATT write, queued from chrAccessCb (NimBLE host task) to
 * readerTask for reassembly — keeps obu_comm/CMD_REBOOT's blocking calls
 * off the host task. */
typedef struct {
    uint16_t len;
    uint8_t  data[];
} rxchunk_t;

#define RX_QUEUE_DEPTH  8

#define CFG_NS          "btstream"
#define CFG_KEY         "cycle"
#define COEX_CHUNK_MS   200

static_assert(BtStream::NO_CONN == BLE_HS_CONN_HANDLE_NONE);

constinit BtStream BtStream::s_instance;

int BtStream::chrAccessCb(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    auto *self = static_cast<BtStream *>(arg);

    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        ESP_LOGW(TAG, "READ op not permitted");
        return BLE_ATT_ERR_READ_NOT_PERMITTED;
    }

    /* Phone -> OBU data write. Runs on the NimBLE host task: just flatten and
     * hand off, the actual reassembly/dispatch happens on readerTask. */
    uint16_t total = OS_MBUF_PKTLEN(ctxt->om);
    if (total == 0 || total > RX_BUF_SIZE) {
        ESP_LOGW(TAG, "Write len exceeded");
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    rxchunk_t *chunk = (rxchunk_t *)malloc(sizeof(rxchunk_t) + total);
    if (chunk == NULL) {
        ESP_LOGW(TAG, "Write oom");
        return BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    if (ble_hs_mbuf_to_flat(ctxt->om, chunk->data, total, NULL) != 0) {
        free(chunk);
        ESP_LOGW(TAG, "Write error");
        return BLE_ATT_ERR_UNLIKELY;
    }
    chunk->len = total;

    if (xQueueSend(self->rxQueue_, &chunk, 0) != pdTRUE) {
        ESP_LOGW(TAG, "rx queue full, dropping write");
        free(chunk);
    }
    else {
        ESP_LOGI(TAG, "Write %d bytes", chunk->len);
    }
    return 0;
}

int BtStream::cfgAccessCb(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    auto *self = static_cast<BtStream *>(arg);
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        uint8_t buf[8];
        for (int i = 0; i < 4; i++) {
            uint16_t v = self->cycleMs_[i];
            buf[i*2]     = (uint8_t)(v        & 0xff);
            buf[i*2 + 1] = (uint8_t)((v >> 8) & 0xff);
        }
        int rc = os_mbuf_append(ctxt->om, buf, sizeof(buf));
        return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        uint16_t total = OS_MBUF_PKTLEN(ctxt->om);
        if (total != 8) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        uint8_t buf[8];
        if (ble_hs_mbuf_to_flat(ctxt->om, buf, sizeof(buf), NULL) != 0) {
            return BLE_ATT_ERR_UNLIKELY;
        }
        for (int i = 0; i < 4; i++) {
            uint16_t v = (uint16_t)buf[i*2] | ((uint16_t)buf[i*2 + 1] << 8);
            /* Sanity: clamp to 100..60000 ms; rejecting silently is more
             * forgiving than returning an error and locking the client out. */
            if (v < 100)    v = 100;
            if (v > 60000)  v = 60000;
            self->cycleMs_[i] = v;
        }
        self->cfgSave();
        return 0;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

const struct ble_gatt_chr_def BtStream::chrDefs_[] = {
    {
        .uuid       = &CHR_UUID.u,
        .access_cb  = chrAccessCb,
        .arg        = &s_instance,
        .flags      = BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_WRITE_NO_RSP,
        .val_handle = &s_instance.chrValHandle_,
    },
    {
        .uuid       = &CFG_UUID.u,
        .access_cb  = cfgAccessCb,
        .arg        = &s_instance,
        .flags      = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
        .val_handle = &s_instance.cfgValHandle_,
    },
    {},
};

const struct ble_gatt_svc_def BtStream::svcDefs_[] = {
    {
        .type            = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid            = &SVC_UUID.u,
        .characteristics = chrDefs_,
    },
    {},
};

void BtStream::cfgLoad()
{
    nvs_handle_t h;
    if (nvs_open(CFG_NS, NVS_READONLY, &h) != ESP_OK) return;
    uint16_t tmp[4];
    size_t sz = sizeof(tmp);
    if (nvs_get_blob(h, CFG_KEY, tmp, &sz) == ESP_OK && sz == sizeof(tmp)) {
        for (int i = 0; i < 4; i++) {
            uint16_t v = tmp[i];
            if (v >= 100 && v <= 60000) cycleMs_[i] = v;
        }
    }
    nvs_close(h);
}

void BtStream::cfgSave()
{
    nvs_handle_t h;
    if (nvs_open(CFG_NS, NVS_READWRITE, &h) != ESP_OK) return;
    uint16_t tmp[4];
    for (int i = 0; i < 4; i++) tmp[i] = cycleMs_[i];
    nvs_set_blob(h, CFG_KEY, tmp, sizeof(tmp));
    nvs_commit(h);
    nvs_close(h);
}

void BtStream::startAdvertising()
{
    struct ble_hs_adv_fields fields = {};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (uint8_t *)DEVICE_NAME;
    fields.name_len = strlen(DEVICE_NAME);
    fields.name_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv_set_fields rc=%d", rc);
        return;
    }

    /* Put the 128-bit service UUID in the scan response so the adv packet
     * stays under the 31-byte legacy limit. */
    struct ble_hs_adv_fields scan_rsp = {};
    scan_rsp.uuids128 = (ble_uuid128_t *)&SVC_UUID;
    scan_rsp.num_uuids128 = 1;
    scan_rsp.uuids128_is_complete = 1;
    rc = ble_gap_adv_rsp_set_fields(&scan_rsp);
    if (rc != 0) {
        ESP_LOGW(TAG, "adv_rsp_set_fields rc=%d", rc);
    }

    struct ble_gap_adv_params p = {};
    p.conn_mode = BLE_GAP_CONN_MODE_UND;
    p.disc_mode = BLE_GAP_DISC_MODE_GEN;
    /* 30 ms / 50 ms in 0.625 ms units — fast enough that Samsung-S6 BLE
     * scan windows reliably catch us. Defaults are slower. */
    p.itvl_min = 0x30;
    p.itvl_max = 0x50;
    rc = ble_gap_adv_start(addrType_, NULL, BLE_HS_FOREVER, &p, gapEventCb, this);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv_start rc=%d", rc);
    } else {
        ESP_LOGI(TAG, "advertising as %s", DEVICE_NAME);
    }
}

int BtStream::gapEventCb(struct ble_gap_event *ev, void *arg)
{
    auto *self = static_cast<BtStream *>(arg);
    switch (ev->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (ev->connect.status == 0) {
            self->connHandle_ = ev->connect.conn_handle;
            self->notifyEnabled_ = false;
            LedController::instance().setBleConnected(true);
            ESP_LOGI(TAG, "connect: handle=%d", self->connHandle_);
            /* Stretch supervision timeout to 30 s — the coex cycle has
             * windows where the link can't be serviced and the default
             * 7.2 s would tear down every connection. */
            struct ble_gap_upd_params up = {
                .itvl_min            = 0x18,   /* 30 ms */
                .itvl_max            = 0x28,   /* 50 ms */
                .latency             = 0,
                .supervision_timeout = 3000,   /* 30 s */
                .min_ce_len          = 0,
                .max_ce_len          = 0,
            };
            ble_gap_update_params(self->connHandle_, &up);
        } else {
            ESP_LOGW(TAG, "connect failed status=%d", ev->connect.status);
            self->startAdvertising();
        }
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "disconnect: reason=0x%x", ev->disconnect.reason);
        self->connHandle_ = NO_CONN;
        self->notifyEnabled_ = false;
        LedController::instance().setBleConnected(false);
        self->startAdvertising();
        break;
    case BLE_GAP_EVENT_SUBSCRIBE:
        if (ev->subscribe.attr_handle == self->chrValHandle_) {
            self->notifyEnabled_ = ev->subscribe.cur_notify;
            ESP_LOGI(TAG, "notify subscription=%d", self->notifyEnabled_);
        }
        break;
    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "mtu: handle=%d value=%d",
                 ev->mtu.conn_handle, ev->mtu.value);
        break;
    default:
        break;
    }
    return 0;
}

void BtStream::onSync()
{
    BtStream &self = s_instance;
    /* Override the controller-supplied address with the factory BT MAC so the
     * advertiser comes up as a Public address — random-static is silently
     * dropped by some older Android stacks (Galaxy S6 etc.). */
    uint8_t mac_be[6] = {0};
    if (esp_read_mac(mac_be, ESP_MAC_BT) == ESP_OK) {
        uint8_t mac_le[6] = {
            mac_be[5], mac_be[4], mac_be[3],
            mac_be[2], mac_be[1], mac_be[0],
        };
        int prc = ble_hs_id_set_pub(mac_le);
        ESP_LOGI(TAG, "[bt] forced public mac %02X:%02X:%02X:%02X:%02X:%02X rc=%d",
                   mac_be[0], mac_be[1], mac_be[2], mac_be[3], mac_be[4], mac_be[5], prc);
    } else {
        ESP_LOGI(TAG, "[bt] esp_read_mac BT failed");
    }
    self.addrType_ = BLE_OWN_ADDR_PUBLIC;

    uint8_t addr[6] = {0};
    ble_hs_id_copy_addr(self.addrType_, addr, NULL);
    ESP_LOGI(TAG, "BLE addr=%02X:%02X:%02X:%02X:%02X:%02X",
             addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);
    self.startAdvertising();
}

void BtStream::onReset(int reason)
{
    ESP_LOGW(TAG, "stack reset: %d", reason);
}

void BtStream::hostTask(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

/* The single ESP32-C5 RF transceiver can't service Wi-Fi promisc and BLE adv
 * simultaneously: while the sniffer is active (with phy_11p_set + 5.9 GHz
 * channel locked), BLE TX is starved and no scanner (Galaxy S6, USB BT
 * dongle) can find the device. iPhones are the rare exception because their
 * scan duty cycle is aggressive enough to catch the few packets that slip
 * through.
 *
 * Resolution: time-share the radio. Sniff for the sniff window, then pause the
 * sniffer (releases promisc + 802.11p PHY + esp_wifi_stop) for the BLE window
 * so scanners see the adv and a connected client gets its notification queue
 * drained. While paused, captured frames sit in the BLE queue until the
 * window opens; while sniffing, the BLE link is starved (clients usually
 * survive the supervision-timeout window with a fresh adv arriving in time).
 */
/* Sleep in short chunks so the loop notices a BLE link state change within
 * COEX_CHUNK_MS instead of holding the radio for the full sniff window. Returns
 * early when the connection state flips relative to `wasConnected`.
 *
 * Why this matters: in wardriving mode (sniff 8000 ms / BLE 400 ms), a
 * disconnect mid-sniff would otherwise leave the phone scanning blind for
 * up to 8 s before the next BLE window opens. Likewise, a fresh connect
 * mid-sniff would starve the link past the host supervision timeout (~5 s
 * on Android default) and trigger an immediate reconnect-loop. */
void BtStream::chunkedDelay(uint32_t totalMs, bool wasConnected)
{
    uint32_t left = totalMs;
    while (left > 0) {
        uint32_t step = (left > COEX_CHUNK_MS) ? COEX_CHUNK_MS : left;
        vTaskDelay(pdMS_TO_TICKS(step));
        left -= step;
        if (connected() != wasConnected) return;
    }
}

void BtStream::coexTask(void *arg)
{
    auto *self = static_cast<BtStream *>(arg);
    /* Boot-time: give BLE the radio first so a user has a chance to
     * connect before the sniffer takes over. */
    WifiController::instance().pause();
    self->chunkedDelay(self->cycleMs_[1], false);
    int n = 0;
    for (;;)
    {
        bool connected = self->connected();
        uint16_t sniff_ms = connected ? self->cycleMs_[2] : self->cycleMs_[0];
        uint16_t ble_ms   = connected ? self->cycleMs_[3] : self->cycleMs_[1];
        WifiController::instance().resume();
        self->chunkedDelay(sniff_ms, connected);
        WifiController::instance().pause();
        if ((n++ & 0xF) == 0) {
            ESP_LOGI(TAG, "[bt] cycle %s: sniff=%u ble=%u",
                       connected ? "CONN" : "DISC", sniff_ms, ble_ms);
        }
        self->chunkedDelay(ble_ms, connected);
    }
}

void BtStream::writerTask(void *arg)
{
    auto *self = static_cast<BtStream *>(arg);
    qframe_t *qf = NULL;
    for (;;) {
        if (xQueueReceive(self->txQueue_, &qf, portMAX_DELAY) != pdTRUE) continue;

        if (!self->linkUp()) {
            free(qf);
            continue;
        }

        uint16_t mtu = ble_att_mtu(self->connHandle_);
        uint16_t chunk = (mtu > 3) ? (uint16_t)(mtu - 3) : 20;
        uint16_t sent = 0;
        while (sent < qf->total_len) {
            uint16_t n = qf->total_len - sent;
            if (n > chunk) n = chunk;
            struct os_mbuf *om = ble_hs_mbuf_from_flat(qf->data + sent, n);
            if (om == NULL) {
                ESP_LOGW(TAG, "mbuf alloc failed, dropping frame");
                break;
            }
            int rc = ble_gatts_notify_custom(self->connHandle_, self->chrValHandle_, om);
            if (rc != 0) {
                ESP_LOGW(TAG, "notify rc=%d, dropping", rc);
                /* om is consumed by NimBLE on success; on failure we leak —
                 * but there is no public API to free it explicitly here. */
                break;
            }
            LedController::instance().pulseBle();
            sent += n;
        }
        free(qf);
    }
}

/* Builds a CommHeader-framed qframe_t and queues it for writerTask — shared
 * by ITS5-frame notifications (COMM_FRAME_RX) and command replies (PONG,
 * CMD_GOOD/FAIL) so both reuse the same MTU-chunked send path. */
void BtStream::enqueueFrame(uint16_t type, const uint8_t *payload, uint16_t len)
{
    size_t total = sizeof(CommHeader) + len;
    qframe_t *qf = (qframe_t *)malloc(sizeof(qframe_t) + total);
    if (qf == NULL) return;

    qf->total_len = (uint16_t)total;

    CommHeader *hdr = (CommHeader *)qf->data;
    hdr->magic = htons(COMM_MAGIC);
    hdr->seqNo = htons(0);  /* not used on the BLE side */
    hdr->xferHeader.type = htons(type);
    hdr->xferHeader.length = htons(len);
    if (len > 0) {
        memcpy(qf->data + sizeof(CommHeader), payload, len);
    }

    if (xQueueSend(txQueue_, &qf, 0) != pdTRUE) {
        free(qf);  /* queue full — drop newest, keep stream live */
    }
}

void BtStream::reply(uint16_t type, const uint8_t *payload, uint16_t len, void *ctx)
{
    auto *self = static_cast<BtStream *>(ctx);
    if (!self->linkUp()) return;
    self->enqueueFrame(type, payload, len);
}

void BtStream::onFrame(uint16_t type, uint16_t length, uint8_t *payload, void *ctx)
{
    obu_comm_process_frame(type, length, payload, reply, ctx);
}

void BtStream::readerTask(void *arg)
{
    auto *self = static_cast<BtStream *>(arg);
    obu_comm_reassembler_t reassembler;
    obu_comm_reassembler_init(&reassembler, self->rxBuf_, sizeof(self->rxBuf_));

    rxchunk_t *chunk = NULL;
    for (;;) {
        if (xQueueReceive(self->rxQueue_, &chunk, portMAX_DELAY) != pdTRUE) continue;
        obu_comm_feed(&reassembler, chunk->data, chunk->len, onFrame, self);
        free(chunk);
    }
}

void BtStream::init()
{
    if (ready_) return;

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "nvs_flash_init: %s", esp_err_to_name(err));
    }

    cfgLoad();
    ESP_LOGI(TAG, "[bt] cfg load: %u/%u %u/%u",
               cycleMs_[0], cycleMs_[1], cycleMs_[2], cycleMs_[3]);

    /* Wi-Fi promiscuous mode otherwise hogs the RF; with BALANCE the BLE adv
     * is so rare that BLE 4.x scanners (Galaxy S6, generic USB BT dongle)
     * miss every window. iPhone catches it because its scan duty cycle is
     * far higher than what Android exposes. PREFER_BT lets BLE pre-empt
     * Wi-Fi long enough to emit a complete adv packet. */
    esp_err_t cerr = esp_coex_preference_set(ESP_COEX_PREFER_BT);
    ESP_LOGI(TAG, "[bt] init: esp_coex_preference_set(PREFER_BT) rc=%d", cerr);

    err = nimble_port_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "[bt] nimble_port_init ok");

    ble_hs_cfg.sync_cb  = onSync;
    ble_hs_cfg.reset_cb = onReset;

    int rc = ble_svc_gap_device_name_set(DEVICE_NAME);
    if (rc != 0) ESP_LOGW(TAG, "name_set rc=%d", rc);

    ble_svc_gap_init();
    ble_svc_gatt_init();

    rc = ble_gatts_count_cfg(svcDefs_);
    if (rc != 0) {
        ESP_LOGE(TAG, "count_cfg rc=%d", rc);
        return;
    }
    rc = ble_gatts_add_svcs(svcDefs_);
    if (rc != 0) {
        ESP_LOGE(TAG, "add_svcs rc=%d", rc);
        return;
    }

    txQueue_ = xQueueCreate(FRAME_QUEUE_DEPTH, sizeof(qframe_t *));
    if (txQueue_ == NULL) {
        ESP_LOGE(TAG, "queue create failed");
        return;
    }
    rxQueue_ = xQueueCreate(RX_QUEUE_DEPTH, sizeof(rxchunk_t *));
    if (rxQueue_ == NULL) {
        ESP_LOGE(TAG, "rx queue create failed");
        return;
    }
    BaseType_t xrc = xTaskCreate(writerTask, "bt_writer", 4096, this, 5, NULL);
    if (xrc != pdPASS) {
        ESP_LOGE(TAG, "writer task create failed");
        return;
    }
    xrc = xTaskCreate(readerTask, "bt_reader", 4096, this, 5, NULL);
    if (xrc != pdPASS) {
        ESP_LOGE(TAG, "reader task create failed");
        return;
    }
    xTaskCreate(coexTask, "bt_coex", 3072, this, 4, NULL);

    nimble_port_freertos_init(hostTask);
    ready_ = true;
    ESP_LOGI(TAG, "initialized");
}

void BtStream::publishFrame(uint16_t type, const uint8_t *payload, uint16_t len)
{
    if (!ready_) return;
    if (!linkUp()) return;
    enqueueFrame(type, payload, len);
}

void BtStream::publishPacket(const wifi_packet_t *pkt)
{
    publishFrame(COMM_FRAME_RX, pkt->payload, pkt->length);
}
