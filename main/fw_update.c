#include "fw_update.h"

#include <string.h>

#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "lwip/def.h"  /* htonl/ntohl */

#include "obu_wifi.h"
#include "ota.h"

static const char TAG[] = "FW_UPDATE";

#define FW_UPDATE_IDLE_TIMEOUT_US ((uint64_t)15 * 1000 * 1000)

static bool s_active;
static esp_ota_handle_t s_handle;
static const esp_partition_t *s_partition;
static uint32_t s_total;
static uint32_t s_written;
static esp_timer_handle_t s_idle_timer;
static int64_t s_last_activity_us;
/* BLE reader, USB rx and esp_timer tasks all enter here; guards s_* and the OTA handle. */
static SemaphoreHandle_t s_lock;

static void reply_ack(obu_comm_reply_fn reply, void *ctx, uint32_t value)
{
    FwAckPayload p = { .value = htonl(value) };
    reply(COMM_FRAME_FW_ACK, (const uint8_t *)&p, sizeof(p), ctx);
}

static void reply_nak(obu_comm_reply_fn reply, void *ctx, uint32_t value)
{
    FwAckPayload p = { .value = htonl(value) };
    reply(COMM_FRAME_FW_NAK, (const uint8_t *)&p, sizeof(p), ctx);
}

/* Radio hand-off for the transfer duration: frozen paused, so BLE gets the
 * whole airtime. Coex off first so coex_window_task can't resume in between;
 * then force the pause, since the USB rx task has already disabled coex
 * before dispatching FW_BEGIN and plain wifi_pause() would no-op.
 * cmd_tx.c's bare wifi_coex_disable() isn't reused: tx wants the radio
 * frozen active, this wants it frozen paused. */
static void radio_take_for_update(void)
{
    wifi_coex_disable();
    wifi_pause_force();
}

static void radio_release_after_update(void)
{
    wifi_coex_enable();
}

static void reset_state(void)
{
    s_active = false;
    s_handle = 0;
    s_partition = NULL;
    s_total = 0;
    s_written = 0;
}

static void idle_timeout_cb(void *arg)
{
    (void)arg;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    /* Activity may have re-armed the timer while this callback waited on the
     * lock; only abort if the session really went idle. */
    if (s_active && esp_timer_get_time() - s_last_activity_us >= (int64_t)FW_UPDATE_IDLE_TIMEOUT_US) {
        ESP_LOGW(TAG, "FW update idle timeout, aborting (%lu/%lu written)",
                 (unsigned long)s_written, (unsigned long)s_total);
        esp_ota_abort(s_handle);
        reset_state();
        radio_release_after_update();
    }
    xSemaphoreGive(s_lock);
}

static void idle_timer_arm(void)
{
    s_last_activity_us = esp_timer_get_time();
    esp_timer_stop(s_idle_timer);
    esp_timer_start_once(s_idle_timer, FW_UPDATE_IDLE_TIMEOUT_US);
}

void fw_update_init(void)
{
    s_lock = xSemaphoreCreateMutex();
    configASSERT(s_lock);

    const esp_timer_create_args_t create_args = {
        .callback = idle_timeout_cb,
        .arg = NULL,
        .name = "fw_update_idle"
    };
    ESP_ERROR_CHECK(esp_timer_create(&create_args, &s_idle_timer));
}

static void begin_locked(const uint8_t *payload, uint16_t length, obu_comm_reply_fn reply, void *ctx)
{
    if (s_active) {
        ESP_LOGW(TAG, "FW_BEGIN while already active");
        reply_nak(reply, ctx, s_written);
        return;
    }

    if (length != sizeof(FwBeginPayload)) {
        ESP_LOGW(TAG, "FW_BEGIN bad length %d", length);
        reply_nak(reply, ctx, 0);
        return;
    }

    FwBeginPayload begin;
    memcpy(&begin, payload, sizeof(begin));
    begin.totalSize = ntohl(begin.totalSize);

    const esp_partition_t *partition = esp_ota_get_next_update_partition(NULL);
    if (partition == NULL) {
        ESP_LOGE(TAG, "FW_BEGIN: no update partition");
        reply_nak(reply, ctx, 0);
        return;
    }
    if (begin.totalSize == 0 || begin.totalSize > partition->size) {
        ESP_LOGW(TAG, "FW_BEGIN: totalSize %lu exceeds partition size %lu",
                 (unsigned long)begin.totalSize, (unsigned long)partition->size);
        reply_nak(reply, ctx, 0);
        return;
    }

    radio_take_for_update();

    esp_ota_handle_t handle;
    esp_err_t err = esp_ota_begin(partition, begin.totalSize, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        radio_release_after_update();
        reply_nak(reply, ctx, 0);
        return;
    }

    s_active = true;
    s_handle = handle;
    s_partition = partition;
    s_total = begin.totalSize;
    s_written = 0;
    idle_timer_arm();

    ESP_LOGI(TAG, "FW update started: %lu bytes -> %s", (unsigned long)s_total, partition->label);
    reply_ack(reply, ctx, 0);
}

static void data_locked(const uint8_t *payload, uint16_t length, obu_comm_reply_fn reply, void *ctx)
{
    if (!s_active) {
        reply_nak(reply, ctx, 0);
        return;
    }
    if (length < sizeof(FwDataHeader)) {
        ESP_LOGW(TAG, "FW_DATA too short: %d", length);
        reply_nak(reply, ctx, s_written);
        return;
    }

    FwDataHeader hdr;
    memcpy(&hdr, payload, sizeof(hdr));
    hdr.offset = ntohl(hdr.offset);
    const uint8_t *data = payload + sizeof(hdr);
    uint16_t data_len = length - sizeof(hdr);

    if (hdr.offset != s_written) {
        ESP_LOGW(TAG, "FW_DATA offset mismatch: got %lu, expected %lu",
                 (unsigned long)hdr.offset, (unsigned long)s_written);
        reply_nak(reply, ctx, s_written);
        return;
    }
    if (s_written + data_len > s_total) {
        ESP_LOGW(TAG, "FW_DATA overruns totalSize (%lu + %u > %lu)",
                 (unsigned long)s_written, data_len, (unsigned long)s_total);
        reply_nak(reply, ctx, s_written);
        return;
    }

    esp_err_t err = esp_ota_write(s_handle, data, data_len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_write failed: %s", esp_err_to_name(err));
        esp_ota_abort(s_handle);
        reset_state();
        radio_release_after_update();
        reply_nak(reply, ctx, 0);
        return;
    }

    s_written += data_len;
    idle_timer_arm();
    reply_ack(reply, ctx, s_written);
}

static void end_locked(obu_comm_reply_fn reply, void *ctx)
{
    if (!s_active) {
        reply_nak(reply, ctx, 0);
        return;
    }
    if (s_written != s_total) {
        ESP_LOGW(TAG, "FW_END with short transfer: %lu/%lu", (unsigned long)s_written, (unsigned long)s_total);
        reply_nak(reply, ctx, s_written);
        return;
    }

    esp_timer_stop(s_idle_timer);

    esp_err_t err = esp_ota_end(s_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed: %s", esp_err_to_name(err));
        reset_state();
        radio_release_after_update();
        reply_nak(reply, ctx, 0);
        return;
    }

    err = esp_ota_set_boot_partition(s_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
        reset_state();
        radio_release_after_update();
        reply_nak(reply, ctx, 0);
        return;
    }

    uint32_t written = s_written;
    ESP_LOGI(TAG, "FW update finished: %lu bytes, reboot required to apply", (unsigned long)written);
    reset_state();
    radio_release_after_update();
    reply_ack(reply, ctx, written);
}

static void abort_locked(obu_comm_reply_fn reply, void *ctx)
{
    if (s_active) {
        esp_timer_stop(s_idle_timer);
        esp_ota_abort(s_handle);
        reset_state();
        radio_release_after_update();
        ESP_LOGI(TAG, "FW update aborted");
    }
    reply_ack(reply, ctx, 0);
}

void fw_update_confirm(obu_comm_reply_fn reply, void *ctx)
{
    esp_err_t err = ota_confirm_running_partition();
    if (err != ESP_OK) {
        reply_nak(reply, ctx, 0);
        return;
    }
    reply_ack(reply, ctx, 0);
}

void fw_update_begin(const uint8_t *payload, uint16_t length, obu_comm_reply_fn reply, void *ctx)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    begin_locked(payload, length, reply, ctx);
    xSemaphoreGive(s_lock);
}

void fw_update_data(const uint8_t *payload, uint16_t length, obu_comm_reply_fn reply, void *ctx)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    data_locked(payload, length, reply, ctx);
    xSemaphoreGive(s_lock);
}

void fw_update_end(obu_comm_reply_fn reply, void *ctx)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    end_locked(reply, ctx);
    xSemaphoreGive(s_lock);
}

void fw_update_abort(obu_comm_reply_fn reply, void *ctx)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    abort_locked(reply, ctx);
    xSemaphoreGive(s_lock);
}
