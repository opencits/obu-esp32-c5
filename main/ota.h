#pragma once

#include "esp_err.h"
#include "esp_event.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* HTTPS pull update (`ota -u`) plus the anti-brick rollback timer: an
 * unconfirmed app is invalidated and rolled back 30 min after boot. */
class OtaManager {
public:
    static OtaManager &instance() { return s_instance; }

    OtaManager(const OtaManager &) = delete;
    OtaManager &operator=(const OtaManager &) = delete;

    void init();

    /* Marks the running app partition valid and cancels the anti-brick rollback
     * timer armed by init(). Shared by `ota -C` and the wire-protocol
     * FW_CONFIRM handler (FwUpdater::confirm). */
    esp_err_t confirmRunningPartition();

    // false if the URL doesn't fit or there's no other app partition
    bool startHttpsUpdate(const char *url, bool reboot);

private:
    constexpr OtaManager() = default;

    static OtaManager s_instance;

    static void taskEntry(void *arg);
    void task();
    static void onEvent(void *arg, esp_event_base_t base, int32_t id, void *data);
    static void onRollbackTimeout(void *arg);
    void initRollbackTimer();

    TaskHandle_t taskHandle_ = nullptr;
    char httpsUrl_[1024] = {};
    bool rebootAfterUpdate_ = false;
    esp_timer_handle_t rollbackTimer_ = nullptr;
    esp_http_client_config_t httpConfig_ = {};  // url points at httpsUrl_, set in init()
    esp_https_ota_config_t otaConfig_ = {};
};

void register_ota_cmd(void);
