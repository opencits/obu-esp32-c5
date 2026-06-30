#include "sdkconfig.h"

#include <stdint.h>
#include <stdbool.h>

#include "esp_event.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "led_indicator.h"
#include "led_indicator_strips.h"

#include "config.h"
#include "events.h"

#include "led.h"

led_indicator_handle_t led_handle;

bool wifi_running;
bool comm_connected;

/* LED 0 (the only WS2812 on the Waveshare devboard) doubles as a status +
 * activity indicator. Idle background reflects sniffer + BLE state; pulses
 * (wifi-rx, ble-tx, usb-tx) briefly override it for visual feedback. */
static bool s_ble_connected;
static struct {
    uint8_t  r, g, b;
    bool     active;
} s_pulse;
static esp_timer_handle_t s_pulse_timer;

#define LED_IRGB(i, r, g, b) SET_IRGB(i, r, g, b)

#define LED_SYSTEM  0
#define LED_WIFI    1
#define LED_COMM    2
#define LED_CITS    3

#define LED_ETH_COLOR_100M LED_IRGB(LED_ETH,    0, 0xFF, 0)
#define LED_ETH_COLOR_10M  LED_IRGB(LED_ETH, 0xFF, 0xA0, 0)

static uint32_t system_led_state  = LED_IRGB(LED_SYSTEM,  0xFF, 0xFF, 0xFF);
static uint32_t wifi_led_state    = LED_IRGB(LED_WIFI,    0xFF,    0,    0);
static uint32_t cits_led_state    = LED_IRGB(LED_CITS,       0,    0,    0);
static uint32_t comm_led_state    = LED_IRGB(LED_COMM,       0,    0,    0);

static bool system_led_blink_state;

static esp_timer_handle_t system_led_timer_handle;
static esp_timer_handle_t cits_led_timer_handle;

static uint8_t led_brightness;

static void set_led_with_brightness(led_indicator_handle_t handle, uint32_t irgb)
{
    uint8_t i = GET_INDEX(irgb);
    uint8_t r = GET_RED(irgb);
    uint8_t g = GET_GREEN(irgb);
    uint8_t b = GET_BLUE(irgb);

    r = ((uint32_t)r) * ((uint32_t)led_brightness) / 255u;
    g = ((uint32_t)g) * ((uint32_t)led_brightness) / 255u;
    b = ((uint32_t)b) * ((uint32_t)led_brightness) / 255u;

    led_indicator_set_rgb(handle, SET_IRGB(i, r, g, b));
}

static void set_comm_led_destroyed(void)
{
    comm_led_state = LED_IRGB(LED_COMM, 0, 0, 0);
    set_led_with_brightness(led_handle, comm_led_state);
}

static void set_comm_led_disconnected(void)
{
    comm_led_state = LED_IRGB(LED_COMM, 0xFF, 0xFF, 0);
    set_led_with_brightness(led_handle, comm_led_state);
}

static void set_comm_led_connected(void)
{
    comm_led_state = LED_IRGB(LED_COMM, 0, 0xFF, 0);
    set_led_with_brightness(led_handle, comm_led_state);
}

static void set_cits_led_idle(void)
{
    cits_led_state = LED_IRGB(LED_CITS, 0, 0, 0);
    set_led_with_brightness(led_handle, cits_led_state);
}

static void set_cits_led_active(void)
{
    cits_led_state = comm_connected ? LED_IRGB(LED_CITS, 0, 0, 0xFF) : LED_IRGB(LED_CITS, 0xFF, 0xA0, 0);
    set_led_with_brightness(led_handle, cits_led_state);
}

static void set_wifi_led_stopped(void)
{
    wifi_led_state = LED_IRGB(LED_WIFI, 0xFF, 0, 0);
    set_led_with_brightness(led_handle, wifi_led_state);
}

static void set_wifi_led_running(void)
{
    wifi_led_state = LED_IRGB(LED_WIFI, 0, 0xFF, 0);
    set_led_with_brightness(led_handle, wifi_led_state);
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                  int32_t event_id, void* event_data)
{
    switch (event_id)
    {
    case WIFI_RECEIVED_PACKET:
        set_cits_led_active();
        if (esp_timer_restart(cits_led_timer_handle, 50000) == ESP_ERR_INVALID_STATE)
        {
            esp_timer_start_once(cits_led_timer_handle, 50000);
        }
        led_pulse_wifi_rx();
        break;
    case WIFI_TRANSMIT_PACKET:
        set_cits_led_active();
        if (esp_timer_restart(cits_led_timer_handle, 50000) == ESP_ERR_INVALID_STATE)
        {
            esp_timer_start_once(cits_led_timer_handle, 50000);
        }
        led_pulse_wifi_tx();
        break;
    case WIFI_STARTED:
        set_wifi_led_running();
        set_cits_led_idle();
        led_status_wifi_running(true);
        break;
    case WIFI_STOPPED:
        set_wifi_led_stopped();
        set_cits_led_idle();
        led_status_wifi_running(false);
        break;
    }
}

static void comm_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    switch (event_id)
    {
    case COMM_CONNECTED:
        comm_connected = true;
        set_comm_led_connected();
        break;
    case COMM_DISCONNECTED:
        comm_connected = false;
        set_comm_led_disconnected();
        break;
    }
}

static void apply_activity_led(void)
{
    if (s_pulse.active) {
        set_led_with_brightness(led_handle,
            LED_IRGB(LED_SYSTEM, s_pulse.r, s_pulse.g, s_pulse.b));
        return;
    }
    /* Idle background: encodes whether the sniffer is in its capture phase
     * and whether a BLE client is hanging on. Mid brightness — pulses still
     * pop because they go to 0xFF. */
    uint8_t r = 0, g = 0, b = 0;
    if (wifi_running && s_ble_connected) {
        r = 0x00; g = 0x40; b = 0x40;        /* teal — all good */
    } else if (wifi_running) {
        r = 0x60; g = 0x18; b = 0x00;        /* orange — capturing, no BLE */
    } else if (s_ble_connected) {
        r = 0x00; g = 0x00; b = 0x60;        /* blue — BLE only */
    } else {
        r = 0x20; g = 0x20; b = 0x20;        /* white — idle */
    }
    set_led_with_brightness(led_handle, LED_IRGB(LED_SYSTEM, r, g, b));
}

static void pulse_end_cb(void *arg)
{
    (void)arg;
    s_pulse.active = false;
    apply_activity_led();
}

static void led_pulse(uint8_t r, uint8_t g, uint8_t b, int ms)
{
    s_pulse.r = r; s_pulse.g = g; s_pulse.b = b;
    s_pulse.active = true;
    apply_activity_led();
    if (esp_timer_restart(s_pulse_timer, ms * 1000) == ESP_ERR_INVALID_STATE) {
        esp_timer_start_once(s_pulse_timer, ms * 1000);
    }
}

void led_pulse_wifi_rx(void)    { led_pulse(0x00, 0xFF, 0x00, 80); }
void led_pulse_wifi_tx(void)    { led_pulse(0xFF, 0x00, 0x00, 80); }
void led_pulse_ble_tx(void)     { led_pulse(0x00, 0x00, 0xFF, 60); }
void led_pulse_usb_tx(void)     { led_pulse(0x00, 0xFF, 0xFF, 40); }

void led_status_ble_connected(bool c)
{
    s_ble_connected = c;
    /* Show the transition with a brief signal pulse: cyan = link up,
     * red = link gone, then settle into the new idle background. */
    led_pulse(c ? 0x00 : 0xFF, c ? 0xFF : 0x00, c ? 0xFF : 0x00, 250);
}

void led_status_wifi_running(bool r)
{
    wifi_running = r;
    apply_activity_led();
}

static void system_led_timer_cb(void *)
{
    /* No longer used for blinking — apply_activity_led drives LED 0
     * directly. We keep this around to refresh the idle colour periodically
     * in case some external write touched the strip. */
    if (!s_pulse.active) apply_activity_led();
}

static void cits_led_timer_cb(void *)
{
    set_cits_led_idle();
}

void led_update(void)
{
    uint8_t brightness;
    // brightness receives the default value of 255 in led.c, and thus cannot fail
    ESP_ERROR_CHECK(config_get_u8(CONFIG_INDEX_LED_BRIGHTNESS, &brightness));
    led_brightness = brightness;

    apply_activity_led();
    set_led_with_brightness(led_handle, wifi_led_state);
    set_led_with_brightness(led_handle, comm_led_state);
    set_led_with_brightness(led_handle, cits_led_state);

    /* Refresh idle background once per second as a fail-safe in case some
     * other writer touched the strip. apply_activity_led skips if a pulse
     * is currently active. */
    esp_timer_start_periodic(system_led_timer_handle, 1000000);
}

void led_init(void)
{
    led_indicator_strips_config_t strips_config = {
        .led_strip_cfg = {
            .strip_gpio_num = CONFIG_LEDSTRIP_PIN,
            .max_leds = 5,
            .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
            .led_model = LED_MODEL_WS2812
        },
        .led_strip_driver = LED_STRIP_RMT,
        .led_strip_rmt_cfg = {0}
    };
    led_indicator_config_t led_config = {
        .blink_lists = NULL,
        .blink_list_num = 0
    };
    ESP_ERROR_CHECK(led_indicator_new_strips_device(&led_config, &strips_config, &led_handle));

    esp_timer_create_args_t create_args = {
        .callback = cits_led_timer_cb,
        .arg = NULL,
        .name = "cits_led"
    };
    ESP_ERROR_CHECK(esp_timer_create(&create_args, &cits_led_timer_handle));

    create_args.callback = system_led_timer_cb;
    create_args.name = "system_led";
    ESP_ERROR_CHECK(esp_timer_create(&create_args, &system_led_timer_handle));

    create_args.callback = pulse_end_cb;
    create_args.name = "led_pulse";
    ESP_ERROR_CHECK(esp_timer_create(&create_args, &s_pulse_timer));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT_BASE, ESP_EVENT_ANY_ID, wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(COMM_EVENT_BASE, ESP_EVENT_ANY_ID, comm_event_handler, NULL));

    uint8_t brightness;
    // brightness receives the default value of 255 in led.c, and thus cannot fail
    ESP_ERROR_CHECK(config_get_u8(CONFIG_INDEX_LED_BRIGHTNESS, &brightness));
    led_brightness = brightness;

    set_led_with_brightness(led_handle, LED_IRGB(0, 0xFF,    0,    0));
    set_led_with_brightness(led_handle, LED_IRGB(1, 0xFF, 0xFF,    0));
    set_led_with_brightness(led_handle, LED_IRGB(2,    0, 0xFF,    0));
    set_led_with_brightness(led_handle, LED_IRGB(3,    0,    0, 0xFF));
    set_led_with_brightness(led_handle, LED_IRGB(4, 0xFF,    0, 0xFF));

    /* Boot self-test on LED 0 so a missing onboard WS2812 / wrong GPIO is
     * obvious at first glance: bright R → G → B → W, ~150 ms each. */
    set_led_with_brightness(led_handle, LED_IRGB(LED_SYSTEM, 0xFF, 0x00, 0x00));
    vTaskDelay(pdMS_TO_TICKS(150));
    set_led_with_brightness(led_handle, LED_IRGB(LED_SYSTEM, 0x00, 0xFF, 0x00));
    vTaskDelay(pdMS_TO_TICKS(150));
    set_led_with_brightness(led_handle, LED_IRGB(LED_SYSTEM, 0x00, 0x00, 0xFF));
    vTaskDelay(pdMS_TO_TICKS(150));
    set_led_with_brightness(led_handle, LED_IRGB(LED_SYSTEM, 0xFF, 0xFF, 0xFF));
    vTaskDelay(pdMS_TO_TICKS(150));
}
