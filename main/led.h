#pragma once

#include <stdbool.h>

void led_update(void);
void led_init(void);

/* Unified activity/status indicator for boards with a single onboard WS2812
 * (Waveshare ESP32-C5-WIFI6-KIT etc.). LED 0 is the activity LED; the multi-
 * LED layout for the original i5r-r1 board still drives indices 1–4.
 *
 * Each pulse briefly overrides the idle background; the background reflects
 * sniffer + BLE link state (set via led_status_*). Calls are safe from any
 * task context.
 */
void led_pulse_sniffer_rx(void);   /* green flash — 802.11p frame captured */
void led_pulse_ble_tx(void);       /* blue flash — BLE notification sent  */
void led_pulse_usb_tx(void);       /* cyan flash — USB frame written      */

void led_status_ble_connected(bool connected);
void led_status_sniffer_running(bool running);
