#pragma once

#include <cstdint>

#include "esp_ota_ops.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "obu_comm.h"

/* Wire-protocol firmware push (FW_BEGIN/DATA/END/ABORT/CONFIRM), into the
 * non-running OTA partition via esp_ota_*. Dispatched from obu_comm.cpp,
 * shared by BLE and USB transports (each has its own obu_comm_reply_fn).
 * One session at a time; every entry point is serialized on an internal lock. */
class FwUpdater {
public:
    static FwUpdater &instance() { return s_instance; }

    FwUpdater(const FwUpdater &) = delete;
    FwUpdater &operator=(const FwUpdater &) = delete;

    void init();

    void begin(const uint8_t *payload, uint16_t length, obu_comm_reply_fn reply, void *ctx);
    void data(const uint8_t *payload, uint16_t length, obu_comm_reply_fn reply, void *ctx);
    void end(obu_comm_reply_fn reply, void *ctx);
    void abort(obu_comm_reply_fn reply, void *ctx);
    // marks the running partition valid (see OtaManager::confirmRunningPartition)
    void confirm(obu_comm_reply_fn reply, void *ctx);

private:
    constexpr FwUpdater() = default;

    static FwUpdater s_instance;

    void resetState();
    void armIdleTimer();
    static void onIdleTimeout(void *arg);

    void beginLocked(const uint8_t *payload, uint16_t length, obu_comm_reply_fn reply, void *ctx);
    void dataLocked(const uint8_t *payload, uint16_t length, obu_comm_reply_fn reply, void *ctx);
    void endLocked(obu_comm_reply_fn reply, void *ctx);
    void abortLocked(obu_comm_reply_fn reply, void *ctx);

    bool active_ = false;
    esp_ota_handle_t handle_ = 0;
    const esp_partition_t *partition_ = nullptr;
    uint32_t total_ = 0;    // announced image size, bytes
    uint32_t written_ = 0;  // bytes written so far == next expected offset
    esp_timer_handle_t idleTimer_ = nullptr;
    int64_t lastActivityUs_ = 0;
    /* BLE reader, USB rx and esp_timer tasks all enter here; guards the state and the OTA handle. */
    SemaphoreHandle_t lock_ = nullptr;
};
