#include "sdkconfig.h"

#include <cstdint>

#include "esp_event.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "led_indicator.h"
#include "led_indicator_strips.h"

#include "config.h"
#include "events.h"

#include "led.h"

#define LED_IRGB(i, r, g, b) SET_IRGB(i, r, g, b)

#define LED_SYSTEM  0

#define LED_ETH_COLOR_100M LED_IRGB(LED_ETH,    0, 0xFF, 0)
#define LED_ETH_COLOR_10M  LED_IRGB(LED_ETH, 0xFF, 0xA0, 0)

constinit LedController LedController::s_instance;

void LedController::setWithBrightness(uint32_t irgb)
{
    uint8_t i = GET_INDEX(irgb);
    uint8_t r = GET_RED(irgb);
    uint8_t g = GET_GREEN(irgb);
    uint8_t b = GET_BLUE(irgb);

    r = ((uint32_t)r) * ((uint32_t)brightness_) / 255u;
    g = ((uint32_t)g) * ((uint32_t)brightness_) / 255u;
    b = ((uint32_t)b) * ((uint32_t)brightness_) / 255u;

    led_indicator_set_rgb(handle_, SET_IRGB(i, r, g, b));
}

void LedController::setCommDisconnected()
{
    commLedState_ = LED_IRGB(LED_COMM, 0xFF, 0xFF, 0);
    setWithBrightness(commLedState_);
}

void LedController::setCommConnected()
{
    commLedState_ = LED_IRGB(LED_COMM, 0, 0xFF, 0);
    setWithBrightness(commLedState_);
}

void LedController::setCitsIdle()
{
    citsLedState_ = LED_IRGB(LED_CITS, 0, 0, 0);
    setWithBrightness(citsLedState_);
}

void LedController::setCitsActive()
{
    citsLedState_ = commConnected_ ? LED_IRGB(LED_CITS, 0, 0, 0xFF) : LED_IRGB(LED_CITS, 0xFF, 0xA0, 0);
    setWithBrightness(citsLedState_);
}

void LedController::flashCitsActive()
{
    setCitsActive();
    if (esp_timer_restart(citsLedTimer_, 50000) == ESP_ERR_INVALID_STATE)
    {
        esp_timer_start_once(citsLedTimer_, 50000);
    }
}

void LedController::setWifiStopped()
{
    wifiLedState_ = LED_IRGB(LED_WIFI, 0xFF, 0, 0);
    setWithBrightness(wifiLedState_);
}

void LedController::setWifiRunningLed()
{
    wifiLedState_ = LED_IRGB(LED_WIFI, 0, 0xFF, 0);
    setWithBrightness(wifiLedState_);
}

void LedController::onWifiEvent(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    auto *self = static_cast<LedController *>(arg);
    switch (id)
    {
    case WIFI_RECEIVED_PACKET:
    case WIFI_TRANSMIT_PACKET:
        self->flashCitsActive();
        self->pulseWifi();
        break;
    case WIFI_STARTED:
        self->setWifiRunningLed();
        self->setCitsIdle();
        self->setWifiRunning(true);
        break;
    case WIFI_STOPPED:
        self->setWifiStopped();
        self->setCitsIdle();
        self->setWifiRunning(false);
        break;
    }
}

void LedController::onCommEvent(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    auto *self = static_cast<LedController *>(arg);
    switch (id)
    {
    case COMM_CONNECTED:
        self->commConnected_ = true;
        self->setCommConnected();
        break;
    case COMM_DISCONNECTED:
        self->commConnected_ = false;
        self->setCommDisconnected();
        break;
    }
}

void LedController::applyActivityLed()
{
    if (pulse_.active) {
        setWithBrightness(LED_IRGB(LED_SYSTEM, pulse_.r, pulse_.g, pulse_.b));
        return;
    }
    /* Idle background: encodes whether the sniffer is in its capture phase
     * and whether a BLE client is hanging on. Mid brightness — pulses still
     * pop because they go to 0xFF. */
    uint8_t r = 0, g = 0, b = 0;
    if (wifiRunning_ && bleConnected_) {
        r = 0x00; g = 0x40; b = 0x40;        /* teal — all good */
    } else if (wifiRunning_) {
        r = 0x60; g = 0x18; b = 0x00;        /* orange — capturing, no BLE */
    } else if (bleConnected_) {
        r = 0x00; g = 0x00; b = 0x60;        /* blue — BLE only */
    } else {
        r = 0x20; g = 0x20; b = 0x20;        /* white — idle */
    }
    setWithBrightness(LED_IRGB(LED_SYSTEM, r, g, b));
}

void LedController::onPulseEnd(void *arg)
{
    auto *self = static_cast<LedController *>(arg);
    self->pulse_.active = false;
    self->applyActivityLed();
}

void LedController::pulse(uint8_t r, uint8_t g, uint8_t b, int ms)
{
    pulse_.r = r; pulse_.g = g; pulse_.b = b;
    pulse_.active = true;
    applyActivityLed();
    if (esp_timer_restart(pulseTimer_, ms * 1000) == ESP_ERR_INVALID_STATE) {
        esp_timer_start_once(pulseTimer_, ms * 1000);
    }
}

void LedController::setBleConnected(bool c)
{
    bleConnected_ = c;
    /* Show the transition with a brief signal pulse: cyan = link up,
     * red = link gone, then settle into the new idle background. */
    pulse(c ? 0x00 : 0xFF, c ? 0xFF : 0x00, c ? 0xFF : 0x00, 250);
}

void LedController::setWifiRunning(bool r)
{
    wifiRunning_ = r;
    applyActivityLed();
}

void LedController::onSystemTimer(void *arg)
{
    /* No longer used for blinking — applyActivityLed drives LED 0
     * directly. We keep this around to refresh the idle colour periodically
     * in case some external write touched the strip. */
    auto *self = static_cast<LedController *>(arg);
    if (!self->pulse_.active) self->applyActivityLed();
}

void LedController::onCitsTimer(void *arg)
{
    static_cast<LedController *>(arg)->setCitsIdle();
}

void LedController::update()
{
    uint8_t brightness;
    // brightness receives the default value of 255 in config.cpp, and thus cannot fail
    ESP_ERROR_CHECK(config_get_u8(CONFIG_INDEX_LED_BRIGHTNESS, &brightness));
    brightness_ = brightness;

    applyActivityLed();
    setWithBrightness(wifiLedState_);
    setWithBrightness(commLedState_);
    setWithBrightness(citsLedState_);

    /* Refresh idle background once per second as a fail-safe in case some
     * other writer touched the strip. applyActivityLed skips if a pulse
     * is currently active. */
    esp_timer_start_periodic(systemLedTimer_, 1000000);
}

void LedController::init()
{
    led_indicator_strips_config_t strips_config = {
        .led_strip_cfg = {
            .strip_gpio_num = CONFIG_LEDSTRIP_PIN,
            .max_leds = 5,
            .led_model = LED_MODEL_WS2812,
            .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        },
        .led_strip_driver = LED_STRIP_RMT,
        .led_strip_rmt_cfg = {},
    };
    led_indicator_config_t led_config = {
        .blink_lists = NULL,
        .blink_list_num = 0
    };
    ESP_ERROR_CHECK(led_indicator_new_strips_device(&led_config, &strips_config, &handle_));

    esp_timer_create_args_t create_args = {
        .callback = onCitsTimer,
        .arg = this,
        .name = "cits_led"
    };
    ESP_ERROR_CHECK(esp_timer_create(&create_args, &citsLedTimer_));

    create_args.callback = onSystemTimer;
    create_args.name = "system_led";
    ESP_ERROR_CHECK(esp_timer_create(&create_args, &systemLedTimer_));

    create_args.callback = onPulseEnd;
    create_args.name = "led_pulse";
    ESP_ERROR_CHECK(esp_timer_create(&create_args, &pulseTimer_));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT_BASE, ESP_EVENT_ANY_ID, onWifiEvent, this));
    ESP_ERROR_CHECK(esp_event_handler_register(COMM_EVENT_BASE, ESP_EVENT_ANY_ID, onCommEvent, this));

    uint8_t brightness;
    // brightness receives the default value of 255 in config.cpp, and thus cannot fail
    ESP_ERROR_CHECK(config_get_u8(CONFIG_INDEX_LED_BRIGHTNESS, &brightness));
    brightness_ = brightness;

    /* Boot self-test on LED 0 so a missing onboard WS2812 / wrong GPIO is
     * obvious at first glance: bright R → G → B → W, ~150 ms each. */
    setWithBrightness(LED_IRGB(LED_SYSTEM, 0xFF, 0x00, 0x00));
    vTaskDelay(pdMS_TO_TICKS(150));
    setWithBrightness(LED_IRGB(LED_SYSTEM, 0x00, 0xFF, 0x00));
    vTaskDelay(pdMS_TO_TICKS(150));
    setWithBrightness(LED_IRGB(LED_SYSTEM, 0x00, 0x00, 0xFF));
    vTaskDelay(pdMS_TO_TICKS(150));
    setWithBrightness(LED_IRGB(LED_SYSTEM, 0xFF, 0xFF, 0xFF));
    vTaskDelay(pdMS_TO_TICKS(150));
    setWithBrightness(LED_IRGB(LED_SYSTEM, 0x00, 0x00, 0x00));
}
