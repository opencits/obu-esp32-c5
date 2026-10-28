#include "fw_update.h"

#include <cstring>

#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "lwip/def.h" /* htonl/ntohl */

#include "obu_wifi.h"
#include "ota.h"

static const char TAG[] = "FW_UPDATE";

#define FW_UPDATE_IDLE_TIMEOUT_US ((uint64_t)15 * 1000 * 1000)

namespace
{

class LockGuard
{
  public:
    explicit LockGuard(SemaphoreHandle_t m)
        : m_(m)
    {
        xSemaphoreTake(m_, portMAX_DELAY);
    }
    ~LockGuard() { xSemaphoreGive(m_); }
    LockGuard(const LockGuard&) = delete;
    LockGuard& operator=(const LockGuard&) = delete;

  private:
    SemaphoreHandle_t m_;
};

void reply_ack(obu_comm_reply_fn reply, void* ctx, uint32_t value)
{
    FwAckPayload p = {.value = htonl(value)};
    reply(COMM_FRAME_FW_ACK, (const uint8_t*)&p, sizeof(p), ctx);
}

void reply_nak(obu_comm_reply_fn reply, void* ctx, uint32_t value)
{
    FwAckPayload p = {.value = htonl(value)};
    reply(COMM_FRAME_FW_NAK, (const uint8_t*)&p, sizeof(p), ctx);
}

/* Radio hand-off for the transfer duration: frozen paused, so BLE gets the
 * whole airtime. Hold first so coex_window_task can't resume in between;
 * then force the pause, since the USB rx task already holds coex before
 * dispatching FW_BEGIN and plain pause() would no-op. cmd_tx.cpp's bare
 * hold isn't enough: tx wants the radio frozen active, this wants it paused. */
void radio_take_for_update()
{
    WifiController::instance().coexHold();
    WifiController::instance().pauseForce();
}

void radio_release_after_update()
{
    WifiController::instance().coexRelease();
}

} // namespace

constinit FwUpdater FwUpdater::s_instance;

void FwUpdater::resetState()
{
    active_ = false;
    handle_ = 0;
    partition_ = nullptr;
    total_ = 0;
    written_ = 0;
}

void FwUpdater::onIdleTimeout(void* arg)
{
    auto* self = static_cast<FwUpdater*>(arg);
    LockGuard lock(self->lock_);
    /* Activity may have re-armed the timer while this callback waited on the
     * lock; only abort if the session really went idle. */
    if (self->active_ &&
        esp_timer_get_time() - self->lastActivityUs_ >= (int64_t)FW_UPDATE_IDLE_TIMEOUT_US)
    {
        ESP_LOGW(TAG,
            "FW update idle timeout, aborting (%lu/%lu written)",
            (unsigned long)self->written_,
            (unsigned long)self->total_);
        esp_ota_abort(self->handle_);
        self->resetState();
        radio_release_after_update();
    }
}

void FwUpdater::armIdleTimer()
{
    lastActivityUs_ = esp_timer_get_time();
    esp_timer_stop(idleTimer_);
    esp_timer_start_once(idleTimer_, FW_UPDATE_IDLE_TIMEOUT_US);
}

void FwUpdater::init()
{
    lock_ = xSemaphoreCreateMutex();
    configASSERT(lock_);

    const esp_timer_create_args_t create_args = {
        .callback = onIdleTimeout, .arg = this, .name = "fw_update_idle"};
    ESP_ERROR_CHECK(esp_timer_create(&create_args, &idleTimer_));
}

void FwUpdater::beginLocked(
    const uint8_t* payload, uint16_t length, obu_comm_reply_fn reply, void* ctx)
{
    if (active_)
    {
        ESP_LOGW(TAG, "FW_BEGIN while already active");
        reply_nak(reply, ctx, written_);
        return;
    }

    if (length != sizeof(FwBeginPayload))
    {
        ESP_LOGW(TAG, "FW_BEGIN bad length %d", length);
        reply_nak(reply, ctx, 0);
        return;
    }

    FwBeginPayload begin;
    memcpy(&begin, payload, sizeof(begin));
    begin.totalSize = ntohl(begin.totalSize);

    const esp_partition_t* partition = esp_ota_get_next_update_partition(NULL);
    if (partition == NULL)
    {
        ESP_LOGE(TAG, "FW_BEGIN: no update partition");
        reply_nak(reply, ctx, 0);
        return;
    }
    if (begin.totalSize == 0 || begin.totalSize > partition->size)
    {
        ESP_LOGW(TAG,
            "FW_BEGIN: totalSize %lu exceeds partition size %lu",
            (unsigned long)begin.totalSize,
            (unsigned long)partition->size);
        reply_nak(reply, ctx, 0);
        return;
    }

    radio_take_for_update();

    esp_ota_handle_t handle;
    esp_err_t err = esp_ota_begin(partition, begin.totalSize, &handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        radio_release_after_update();
        reply_nak(reply, ctx, 0);
        return;
    }

    active_ = true;
    handle_ = handle;
    partition_ = partition;
    total_ = begin.totalSize;
    written_ = 0;
    armIdleTimer();

    ESP_LOGI(TAG, "FW update started: %lu bytes -> %s", (unsigned long)total_, partition->label);
    reply_ack(reply, ctx, 0);
}

void FwUpdater::dataLocked(
    const uint8_t* payload, uint16_t length, obu_comm_reply_fn reply, void* ctx)
{
    if (!active_)
    {
        reply_nak(reply, ctx, 0);
        return;
    }
    if (length < sizeof(FwDataHeader))
    {
        ESP_LOGW(TAG, "FW_DATA too short: %d", length);
        reply_nak(reply, ctx, written_);
        return;
    }

    FwDataHeader hdr;
    memcpy(&hdr, payload, sizeof(hdr));
    hdr.offset = ntohl(hdr.offset);
    const uint8_t* data = payload + sizeof(hdr);
    uint16_t data_len = length - sizeof(hdr);

    if (hdr.offset != written_)
    {
        ESP_LOGW(TAG,
            "FW_DATA offset mismatch: got %lu, expected %lu",
            (unsigned long)hdr.offset,
            (unsigned long)written_);
        reply_nak(reply, ctx, written_);
        return;
    }
    if (written_ + data_len > total_)
    {
        ESP_LOGW(TAG,
            "FW_DATA overruns totalSize (%lu + %u > %lu)",
            (unsigned long)written_,
            data_len,
            (unsigned long)total_);
        reply_nak(reply, ctx, written_);
        return;
    }

    esp_err_t err = esp_ota_write(handle_, data, data_len);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_ota_write failed: %s", esp_err_to_name(err));
        esp_ota_abort(handle_);
        resetState();
        radio_release_after_update();
        reply_nak(reply, ctx, 0);
        return;
    }

    written_ += data_len;
    armIdleTimer();
    reply_ack(reply, ctx, written_);
}

void FwUpdater::endLocked(obu_comm_reply_fn reply, void* ctx)
{
    if (!active_)
    {
        reply_nak(reply, ctx, 0);
        return;
    }
    if (written_ != total_)
    {
        ESP_LOGW(TAG,
            "FW_END with short transfer: %lu/%lu",
            (unsigned long)written_,
            (unsigned long)total_);
        reply_nak(reply, ctx, written_);
        return;
    }

    esp_timer_stop(idleTimer_);

    esp_err_t err = esp_ota_end(handle_);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_ota_end failed: %s", esp_err_to_name(err));
        resetState();
        radio_release_after_update();
        reply_nak(reply, ctx, 0);
        return;
    }

    err = esp_ota_set_boot_partition(partition_);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
        resetState();
        radio_release_after_update();
        reply_nak(reply, ctx, 0);
        return;
    }

    uint32_t written = written_;
    ESP_LOGI(
        TAG, "FW update finished: %lu bytes, reboot required to apply", (unsigned long)written);
    resetState();
    radio_release_after_update();
    reply_ack(reply, ctx, written);
}

void FwUpdater::abortLocked(obu_comm_reply_fn reply, void* ctx)
{
    if (active_)
    {
        esp_timer_stop(idleTimer_);
        esp_ota_abort(handle_);
        resetState();
        radio_release_after_update();
        ESP_LOGI(TAG, "FW update aborted");
    }
    reply_ack(reply, ctx, 0);
}

void FwUpdater::begin(const uint8_t* payload, uint16_t length, obu_comm_reply_fn reply, void* ctx)
{
    LockGuard lock(lock_);
    beginLocked(payload, length, reply, ctx);
}

void FwUpdater::data(const uint8_t* payload, uint16_t length, obu_comm_reply_fn reply, void* ctx)
{
    LockGuard lock(lock_);
    dataLocked(payload, length, reply, ctx);
}

void FwUpdater::end(obu_comm_reply_fn reply, void* ctx)
{
    LockGuard lock(lock_);
    endLocked(reply, ctx);
}

void FwUpdater::abort(obu_comm_reply_fn reply, void* ctx)
{
    LockGuard lock(lock_);
    abortLocked(reply, ctx);
}

void FwUpdater::confirm(obu_comm_reply_fn reply, void* ctx)
{
    esp_err_t err = OtaManager::instance().confirmRunningPartition();
    if (err != ESP_OK)
    {
        reply_nak(reply, ctx, 0);
        return;
    }
    reply_ack(reply, ctx, 0);
}
