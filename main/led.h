#pragma once

#include <cstdint>

#include "esp_event.h"
#include "esp_timer.h"
#include "led_convert.h"
#include "led_indicator.h"

/* Unified activity/status indicator for boards with a single onboard WS2812
 * (Waveshare ESP32-C5-WIFI6-KIT etc.). LED 0 is the activity LED; the multi-
 * LED layout for the original i5r-r1 board still drives indices 1–4.
 *
 * Each pulse briefly overrides the idle background; the background reflects
 * wifi + BLE link state (setBleConnected/setWifiRunning). Calls are safe from
 * any task context.
 */
class LedController {
public:
    static LedController &instance() { return s_instance; }

    LedController(const LedController &) = delete;
    LedController &operator=(const LedController &) = delete;

    void init();
    void update(); // re-read brightness from config, repaint, start idle refresh

    void pulseWifi() { pulse(0xFF, 0x00, 0x00, 40); } // red flash — 802.11p frame
    void pulseBle()  { pulse(0x00, 0x00, 0xFF, 60); } // blue flash — BLE notification sent
    void pulseUsb()  { pulse(0x00, 0xFF, 0x00, 40); } // green flash — USB frame written

    void setBleConnected(bool connected);
    void setWifiRunning(bool running);

private:
    constexpr LedController() = default;

    static LedController s_instance;

    void pulse(uint8_t r, uint8_t g, uint8_t b, int ms);
    void setWithBrightness(uint32_t irgb);

    void setCommDisconnected();
    void setCommConnected();
    void setCitsIdle();
    void setCitsActive();
    void flashCitsActive();
    void setWifiStopped();
    void setWifiRunningLed();
    void applyActivityLed();

    static void onWifiEvent(void *arg, esp_event_base_t base, int32_t id, void *data);
    static void onCommEvent(void *arg, esp_event_base_t base, int32_t id, void *data);
    static void onPulseEnd(void *arg);
    static void onSystemTimer(void *arg);
    static void onCitsTimer(void *arg);

    led_indicator_handle_t handle_ = nullptr;

    bool wifiRunning_ = false;
    bool commConnected_ = false;
    bool bleConnected_ = false;

    struct {
        uint8_t r, g, b;
        bool active;
    } pulse_ = {};
    esp_timer_handle_t pulseTimer_ = nullptr;

    // strip indices of the i5r-r1 status LEDs; index 0 is the activity LED
    static constexpr uint8_t LED_WIFI = 1;
    static constexpr uint8_t LED_COMM = 2;
    static constexpr uint8_t LED_CITS = 3;

    // last painted IRGB per status LED, re-applied by update()
    uint32_t wifiLedState_ = SET_IRGB(LED_WIFI, 0xFF, 0, 0);
    uint32_t citsLedState_ = SET_IRGB(LED_CITS, 0, 0, 0);
    uint32_t commLedState_ = SET_IRGB(LED_COMM, 0, 0, 0);

    esp_timer_handle_t systemLedTimer_ = nullptr;
    esp_timer_handle_t citsLedTimer_ = nullptr;

    uint8_t brightness_ = 0; // 0..255 scale applied to every colour
};
