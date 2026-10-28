#pragma once

#include <cstdint>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "obu_comm.h"
#include "obu_wifi.h"

/* CommHeader-framed link to the host over USB-Serial-JTAG: TX of captured
 * frames/replies, RX of host commands via the shared obu_comm reassembler. */
class UsbStream {
public:
    static UsbStream &instance() { return s_instance; }

    UsbStream(const UsbStream &) = delete;
    UsbStream &operator=(const UsbStream &) = delete;

    void init();
    void publishPacket(const wifi_packet_t *pkt);
    void publishFrame(uint16_t type, const uint8_t *payload, uint16_t len) { writeFrame(type, payload, len); }
    void sendPing() { writeFrame(COMM_FRAME_PING, nullptr, 0); }

    /* Start the USB RX config channel task: reassembles host frames and
     * dispatches them via obu_comm_process_frame(), replying over USB. */
    void startRx();

private:
    constexpr UsbStream() = default;

    static UsbStream s_instance;

    void writeFrame(uint16_t type, const void *payload, uint16_t len);
    uint16_t nextSeqNo();
    static void reply(uint16_t type, const uint8_t *payload, uint16_t len, void *ctx);
    static void onFrame(uint16_t type, uint16_t length, uint8_t *payload, void *ctx);
    static void rxTaskEntry(void *arg);
    void rxTask();

    bool ready_ = false;
    uint16_t seqNo_ = 0;  // next outgoing CommHeader.seqNo; skips 0 on wrap
    /* sniffTask, the rx task and tCbrInd all write frames; hold this across
     * seqno + header + payload so frames never interleave on the wire. */
    SemaphoreHandle_t txLock_ = nullptr;
    uint8_t commBuffer_[COMM_MAX_BUF_SIZE] = {}; // RX reassembly buffer, written in place by rxTask
};
