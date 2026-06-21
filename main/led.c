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
#include "ethernet.h"
#include "events.h"

#include "led.h"

led_indicator_handle_t led_handle;
bool sniffer_running;
bool comm_connected;

/* LED 0 (the only WS2812 on the Waveshare devboard) doubles as a status +
 * activity indicator. Idle background reflects sniffer + BLE state; pulses
 * (sniffer-rx, ble-tx, usb-tx) briefly override it for visual feedback. */
static bool s_ble_connected;
static struct {
    uint8_t  r, g, b;
    bool     active;
} s_pulse;
static esp_timer_handle_t s_pulse_timer;

#define LED_IRGB(i, r, g, b) SET_IRGB(i, r, g, b)

#define LED_SYSTEM  0
#define LED_SNIFFER 1
#define LED_ETH     2
#define LED_COMM    3
#define LED_CITS    4

#define LED_ETH_COLOR_100M LED_IRGB(LED_ETH,    0, 0xFF, 0)
#define LED_ETH_COLOR_10M  LED_IRGB(LED_ETH, 0xFF, 0xA0, 0)

static uint32_t system_led_state  = LED_IRGB(LED_SYSTEM,  0xFF, 0xFF, 0xFF);
static uint32_t sniffer_led_state = LED_IRGB(LED_SNIFFER, 0xFF,    0,    0);
static uint32_t eth_led_state     = LED_IRGB(LED_ETH,        0,    0,    0);
static uint32_t cits_led_state    = LED_IRGB(LED_CITS,       0,    0,    0);
static uint32_t comm_led_state    = LED_IRGB(LED_COMM,       0,    0,    0);

static bool system_led_blink_state;

static esp_timer_handle_t system_led_timer_handle;
static esp_timer_handle_t cits_led_timer_handle;
static esp_timer_handle_t eth_led_timer_handle;

static eth_speed_t eth_speed;
static bool eth_link_state;
static bool eth_led_blink_state;

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

static void set_eth_led_disconnected(void)
{
    esp_timer_stop_blocking(eth_led_timer_handle, 10 / portTICK_PERIOD_MS);

    eth_led_state = LED_IRGB(LED_ETH, 0, 0, 0);
    set_led_with_brightness(led_handle, eth_led_state);
}

static void set_eth_led_connected(void)
{
    esp_timer_stop_blocking(eth_led_timer_handle, 10 / portTICK_PERIOD_MS);

    eth_led_blink_state = false;
    eth_led_state = LED_IRGB(LED_ETH, 0, 0, 0);
    set_led_with_brightness(led_handle, eth_led_state);

    esp_timer_start_periodic(eth_led_timer_handle, 500000);
}

static void set_eth_led_connected_with_ip(void)
{
    esp_timer_stop_blocking(eth_led_timer_handle, 10 / portTICK_PERIOD_MS);

    eth_led_state = eth_speed == ETH_SPEED_100M ? LED_ETH_COLOR_100M : LED_ETH_COLOR_10M;
    set_led_with_brightness(led_handle, eth_led_state);
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

static void set_sniffer_led_stopped(void)
{
    sniffer_led_state = LED_IRGB(LED_SNIFFER, 0xFF, 0, 0);
    set_led_with_brightness(led_handle, sniffer_led_state);
}

static void set_sniffer_led_running(void)
{
    sniffer_led_state = LED_IRGB(LED_SNIFFER, 0, 0xFF, 0);
    set_led_with_brightness(led_handle, sniffer_led_state);
}

static void app_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    switch (event_id)
    {
    case APP_ETHERNET_MGMT_INTERFACE_CONNECTED:
        eth_link_state = true;
        eth_speed = ethernet_get_mgmt_if_link_speed();
        set_eth_led_connected();
        break;
    case APP_ETHERNET_MGMT_INTERFACE_GOT_IP:
        set_eth_led_connected_with_ip();
        break;
    case APP_ETHERNET_MGMT_INTERFACE_LOST_IP:
        if (eth_link_state)
            set_eth_led_connected();
        break;
    case APP_ETHERNET_MGMT_INTERFACE_DISCONNECTED:
        eth_link_state = false;
        set_eth_led_disconnected();
        break;
    }
}

static void sniffer_event_handler(void* arg, esp_event_base_t event_base,
                                  int32_t event_id, void* event_data)
{
    switch (event_id)
    {
    case SNIFFER_RECEIVED_PACKET:
        set_cits_led_active();
        if (esp_timer_restart(cits_led_timer_handle, 50000) == ESP_ERR_INVALID_STATE)
        {
            esp_timer_start_once(cits_led_timer_handle, 50000);
        }
        led_pulse_sniffer_rx();
        break;
    case SNIFFER_STARTED:
        set_sniffer_led_running();
        set_cits_led_idle();
        led_status_sniffer_running(true);
        break;
    case SNIFFER_STOPPED:
        set_sniffer_led_stopped();
        set_cits_led_idle();
        led_status_sniffer_running(false);
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
    if (sniffer_running && s_ble_connected) {
        r = 0x00; g = 0x40; b = 0x40;        /* teal — all good */
    } else if (sniffer_running) {
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

void led_pulse_sniffer_rx(void) { led_pulse(0x00, 0xFF, 0x00, 80); }
void led_pulse_ble_tx(void)     { led_pulse(0x00, 0x00, 0xFF, 60); }
void led_pulse_usb_tx(void)     { led_pulse(0x00, 0xFF, 0xFF, 40); }

void led_status_ble_connected(bool c)
{
    s_ble_connected = c;
    /* Show the transition with a brief signal pulse: cyan = link up,
     * red = link gone, then settle into the new idle background. */
    led_pulse(c ? 0x00 : 0xFF, c ? 0xFF : 0x00, c ? 0xFF : 0x00, 250);
}

void led_status_sniffer_running(bool r)
{
    sniffer_running = r;
    apply_activity_led();
}

static void system_led_timer_cb(void *)
{
    /* No longer used for blinking — apply_activity_led drives LED 0
     * directly. We keep this around to refresh the idle colour periodically
     * in case some external write touched the strip. */
    if (!s_pulse.active) apply_activity_led();
}

static void eth_led_timer_cb(void *)
{
    eth_led_state = eth_led_blink_state ?
                (eth_speed == ETH_SPEED_100M ? LED_ETH_COLOR_100M : LED_ETH_COLOR_10M) :
                LED_IRGB(LED_ETH, 0, 0, 0);
    set_led_with_brightness(led_handle, eth_led_state);

    eth_led_blink_state = !eth_led_blink_state;
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
    set_led_with_brightness(led_handle, sniffer_led_state);
    set_led_with_brightness(led_handle, eth_led_state);
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

    create_args.callback = eth_led_timer_cb;
    create_args.name = "eth_led";
    ESP_ERROR_CHECK(esp_timer_create(&create_args, &eth_led_timer_handle));

    create_args.callback = system_led_timer_cb;
    create_args.name = "system_led";
    ESP_ERROR_CHECK(esp_timer_create(&create_args, &system_led_timer_handle));

    create_args.callback = pulse_end_cb;
    create_args.name = "led_pulse";
    ESP_ERROR_CHECK(esp_timer_create(&create_args, &s_pulse_timer));

    ESP_ERROR_CHECK(esp_event_handler_register(SNIFFER_EVENT_BASE, ESP_EVENT_ANY_ID, sniffer_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(COMM_EVENT_BASE, ESP_EVENT_ANY_ID, comm_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(APP_EVENT_BASE, ESP_EVENT_ANY_ID, app_event_handler, NULL));

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
