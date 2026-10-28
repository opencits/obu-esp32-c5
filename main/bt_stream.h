#pragma once

#include <cstdint>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "obu_wifi.h"

struct ble_gap_event;
struct ble_gatt_access_ctxt;
struct ble_gatt_chr_def;
struct ble_gatt_svc_def;

/* BLE GATT link to the phone (notify stream + config characteristic) and the
 * Wi-Fi/BLE radio time-slicing scheduler (coexTask). */
class BtStream {
public:
    static BtStream &instance() { return s_instance; }

    static constexpr uint16_t NO_CONN = 0xffff; // == BLE_HS_CONN_HANDLE_NONE, checked in .cpp

    BtStream(const BtStream &) = delete;
    BtStream &operator=(const BtStream &) = delete;

    void init();
    // no-op unless a client is connected and subscribed to notifications
    void publishPacket(const wifi_packet_t *pkt);
    void publishFrame(uint16_t type, const uint8_t *payload, uint16_t len);

private:
    static constexpr int RX_BUF_SIZE = 2048;    // phone-write reassembly: control/config traffic only

    constexpr BtStream() = default;

    static BtStream s_instance;

    bool linkUp() const { return connHandle_ != NO_CONN && notifyEnabled_; }
    bool connected() const { return connHandle_ != NO_CONN; }

    void cfgLoad();
    void cfgSave();
    void startAdvertising();
    void chunkedDelay(uint32_t totalMs, bool wasConnected);
    void enqueueFrame(uint16_t type, const uint8_t *payload, uint16_t len);

    static int chrAccessCb(uint16_t conn_handle, uint16_t attr_handle,
                           struct ble_gatt_access_ctxt *ctxt, void *arg);
    static int cfgAccessCb(uint16_t conn_handle, uint16_t attr_handle,
                           struct ble_gatt_access_ctxt *ctxt, void *arg);
    static int gapEventCb(struct ble_gap_event *ev, void *arg);
    static void onSync();
    static void onReset(int reason);
    static void hostTask(void *param);
    static void coexTask(void *arg);
    static void writerTask(void *arg);
    static void readerTask(void *arg);
    static void reply(uint16_t type, const uint8_t *payload, uint16_t len, void *ctx);
    static void onFrame(uint16_t type, uint16_t length, uint8_t *payload, void *ctx);

    static const struct ble_gatt_chr_def chrDefs_[];
    static const struct ble_gatt_svc_def svcDefs_[];

    uint8_t addrType_ = 0;
    volatile uint16_t connHandle_ = NO_CONN;
    uint16_t chrValHandle_ = 0;
    uint16_t cfgValHandle_ = 0;
    volatile bool notifyEnabled_ = false;
    bool ready_ = false;
    QueueHandle_t txQueue_ = nullptr;  // qframe_t*, writerTask frees
    QueueHandle_t rxQueue_ = nullptr;  // rxchunk_t*, readerTask frees

    /* Coex cycle parameters, runtime-tunable via the config characteristic.
     * Values are milliseconds, clamped to 100..60000; volatile because read
     * by coexTask and written from the GATT access callback.
     *   0..1: discovery cycle sniff/ble (no client connected)
     *   2..3: connected cycle sniff/ble (client present)
     */
    volatile uint16_t cycleMs_[4] = { 10000, 2000, 800, 400 };
    uint8_t rxBuf_[RX_BUF_SIZE] = {};
};
