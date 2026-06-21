# Design Notes

Central implementation/design reference for the ITS-G5 receiver firmware (ESP32-C5). 
Built for Waveshare ESP32-C5-WIFI6-KIT

Build/flash commands: `skills/build_deploy.md`. Hard agent rules: `AGENTS.md`.

Forked from https://github.com/pit711/V2X2MAP 

## Purpose

Sniffs 802.11p (ITS-G5, 5.9 GHz) frames and fans each captured frame out to up to three sinks simultaneously: 
 - a local pcap file/buffer
 - BLE GATT notifications
 - and a USB-CDC stream. WiFi/Ethernet/MQTT connectivity is for management (config, OTA), not capture.

## Boot sequence (`main/main.c::app_main`)

Single FreeRTOS app, no custom task framework beyond ESP-IDF's own. Order matters:
`esp_event_loop_create_default → config_init → led_init → filesystem (history FAT) → SPI → WiFi driver init (esp_wifi_init, STA mode, not started) → console REPL → sniffer_init → ota_init → register all esp_console commands → sniffer_autostart → console_start_repl → post APP_BOOT_FINISHED`.

Most features are exposed as `esp_console` commands (`sniffer`, `pcap`, `ota`, `mount`/`unmount`, config setters, `reboot`) registered at startup, not standalone tasks — the REPL is the primary control surface.

**Known gap (as of this writing):** `wifi_manager_init/start`, `bt_stream_init`, `usb_config_rx_start`, and `initialize_ethernet` are implemented but **not called from `app_main`**. The one exception: `usb_stream_init` self-inits lazily — `usb_stream_publish_packet()` calls it on first use, and `sniffer_task` calls that per packet — so **USB frame output is the only external sink that actually runs today**; BLE notifications, the WiFi cycle task, the USB config-RX protocol, and Ethernet are all dormant until something calls their init functions from `main.c`. The BLE coex scheme and WiFi/sniffer time-slicing described below are real, implemented logic — just not currently reachable at boot. Check `main.c` before relying on any of it being live.

## The shared-radio problem

The ESP32-C5 has a single RF transceiver shared between 5 GHz WiFi/802.11p and 2.4 GHz BLE. It cannot service WiFi promiscuous RX and BLE advertising/connections at the same time — whichever mode is active starves the other. Two independent time-sharing schemes exist for this, layered on top of each other:

1. **Sniffer ⟷ WiFi STA** (`wifi_manager.c`, modes via `CONFIG_INDEX_SNIFF_MODE`):
   - `SNIFF_MODE_REALTIME` — STA connects once, then promiscuous mode runs *on top of* the STA connection on the same channel. No time-sharing once connected.
   - `SNIFF_MODE_CYCLE` / `_INDIVIDUAL` — `cycle_task` alternates: sniff for `sniff_ms` → `sniffer_pause()` (drops promisc + 802.11p PHY + `esp_wifi_stop`) → STA connect, give MQTT `wifi_ms/3` (min 200 ms) to flush → disconnect → `sniffer_resume()`. Defaults: `SNIFF_MS_DEFAULT=10000`, `WIFI_MS_DEFAULT=2000` (`wifi_manager.h`).
   - `CONFIG_INDEX_CONN_MODE` gates whether WiFi runs at all (`CONN_MODE_BLE` / `_WIFI` / `_BOTH`).

2. **Sniffer ⟷ BLE** (`bt_stream.c::coex_window_task`): independent of the WiFi cycle. Alternates sniffing and BLE-advertising windows so BLE scanners (notably older Android stacks with slow scan duty cycles — Galaxy S6 was the motivating case) actually see the advertisement between sniffer bursts. Durations are NVS-persisted (`btstream`/`cycle`) and runtime-tunable via the BLE config characteristic: `CYC_DISC_SNIFF/CYC_DISC_BLE` (no client, default 10000/2000 ms) and `CYC_CONN_SNIFF/CYC_CONN_BLE` (client connected, default 800/400 ms) — connected clients get serviced more often since NimBLE's software coex layer can handle scheduled connection events correctly once linked. `chunked_delay()` sleeps in 200 ms steps so a BLE connect/disconnect mid-window is noticed quickly rather than starving the link for the full window.
   - `esp_coex_preference_set(ESP_COEX_PREFER_BT)` is set at BLE init so WiFi yields enough airtime for a full BLE adv packet to go out; flipped to `ESP_COEX_PREFER_WIFI` during a USB-triggered WiFi scan (`usb_stream.c::cfg_scan_task`).

Net effect: there is no global arbiter — `cmd_sniffer.c` (`sniffer_pause`/`sniffer_resume`), `wifi_manager.c`, and `bt_stream.c` each pause/resume their own slice independently, assuming the others cooperate. Changing one cycle's timing can starve the other.

## Capture/distribution pipeline

- `cmd_sniffer.c` — owns `esp_wifi` promiscuous mode and the 802.11p PHY. Its companion `cmd_pcap.c` carries an Espressif/Unlicense SPDX header, i.e. vendored from ESP-IDF's wifi-sniffer example; `cmd_sniffer.c` itself has no such header but follows the same structure, so it's likely adapted from the same example (unconfirmed). `phy_change_channel`/`phy_11p_set` are forward-declared `extern` with no header: `phy_11p_set` resolves to a mask-ROM address (`esp-idf/components/esp_rom/esp32c5/ld/esp32c5.rom.eco3.ld`), `phy_change_channel` to the precompiled `esp_wifi` blob libs (`esp-idf/components/esp_wifi/lib/`) — not in any source you can grep. How channel tuning actually works (the 4-step PHY dance, frequency-vs-channel-number confusion, known rough edges): `docs/radio_80211p_notes.md`.
  - `wifi_sniffer_cb` (promiscuous RX callback) queues `sniffer_packet_info_t {payload, length, seconds, microseconds}` to `work_queue` (`CONFIG_SNIFFER_WORK_QUEUE_LEN`, default 128).
  - `sniffer_task` drains the queue and fans each packet to: `packet_capture()` (`cmd_pcap.c`, only if `write_pcap`) → `usb_stream_publish_packet()` → `bt_stream_publish_packet()`.
  - `sniffer_autostart()` reads `CONFIG_INDEX_AUTOSTART_CHAN`; only starts if channel is in `5800..5900` (802.11p band sanity check). `CONFIG_INDEX_BROADCAST_ONLY` filters to broadcast frames.
  - `sniffer_pause/resume` are the radio hand-off primitives both `wifi_manager.c` and (indirectly, via `cmd_sniffer.h`) the rest of the app rely on.

- `cmd_pcap.c` — pcap sink, vendored from Espressif's example, two mutually-exclusive destinations selected at build time (`SNIFFER_PCAP_DESTINATION_{JTAG,MEMORY}`, `Kconfig.projbuild`): JTAG/App Trace (host-side capture, flushed on a 1 s timer), or a fixed in-memory ring (`CONFIG_SNIFFER_PCAP_MEMORY_SIZE`, default 4096 B; current default on this board, no SD card slot exists). An SD-card destination existed upstream but was removed along with `sdcard.c` — see Waveshare specifics below.

- `bt_stream.c` — NimBLE GATT server (service `b6e57e90-12d8-4a47-9b21-3f0000000001`). Two characteristics:
  - **Notify** (`...02`) — captured frames, framed as `"ITS5" + sec(u32 LE) + usec(u32 LE) + len(u16 LE) + payload` (`HEADER_LEN=14`, `MAX_PAYLOAD=2048`). `writer_task` drains `s_queue` (depth 16, drop-newest on overflow) and chunks each frame to `ble_att_mtu - 3` per `ble_gatts_notify_custom` call.
  - **Config** (`...03`) — 8 bytes = 4×`u16` LE, the coex cycle timings above; persisted to NVS namespace `btstream` on write.
  - Forces the controller's BT MAC as a **public** BLE address (`ble_hs_id_set_pub`, a private NimBLE API) because random-static addresses are silently ignored by some older Android BLE stacks (Galaxy S6) that haven't paired with that address before.
  - ASCII diagnostics are piggy-backed onto the USB-CDC stream (`dbg_printf`, `\r\n`-terminated) — safe because the USB frame reader on the other end resyncs on the `"ITS5"` magic and drops anything that isn't a real frame.

- `usb_stream.c` — same `"ITS5"` framing over USB-CDC (`usb_serial_jtag` driver, 4 KB TX / 256 B RX buffers). Also runs a line-oriented RX task (`usb_config_rx_task`) implementing a tiny text protocol for an external setup wizard:
  - `CFG:key=value\n` → looked up against a key table (`cfg_key_to_index`) mapping to `config_index_t`, typed by table (`is_u8_key`/`is_u32_key`/else string) → `config_set_*` → NVS. Replies `CFG_OK:key\n` / `CFG_ERR:key:msg\n`.
  - `SCAN\n` → spawns `cfg_scan_task`: pauses BLE, flips coex preference to WiFi, brings up STA just long enough to active-scan (country `DE`, channels 1–13), streams `SCAN_AP:ssid,chN,authM\n` lines, then restores BLE/coex. `usb_cfg_is_scanning()` is checked by `wifi_manager.c`'s `WIFI_EVENT_STA_START` handler so the normal STA-connect logic doesn't fight the scan.
  - `REBOOT\n` → acks then `esp_restart()`.

## Connectivity (management plane, not capture)

- `wifi_manager.c` — STA only, no AP mode. Credentials tried in order: `wifi1ssid/pass` → `wifi2ssid/pass` → (if `CONFIG_INDEX_WIFI_OPEN`) scan-and-join the first open AP found. Static IP via `CONFIG_INDEX_WIFI_IP/NM/GW/DNS` (empty IP = DHCP). Country fixed to `DE` (channels 1–13) everywhere STA is started.
- `ethernet.c` — supports multiple Ethernet ports (`ethernet_init_all`); port index `ETH_MANAGEMENT_INTERFACE` (currently hardcoded `0`, marked `// TODO make configurable?`) is the "management" interface: gets static-IP/DNS config from NVS, MTU clamped to `MAX_MTU=1420`, and is the only interface that posts `APP_ETHERNET_MGMT_INTERFACE_*` events. Non-management ports are just started, with a commented-out hook for registering them with the sniffer (`sniffer_reg_eth_intf`, not implemented).
- **MQTT**: `MQTT_EVENT_BASE`/`MQTT_CONNECTED`/`MQTT_DISCONNECTED` events and an `mqtt_connected` flag (read by `led.c`) and `CONFIG_INDEX_MQTT_URI` config key all exist, and both `ethernet.c` and `wifi_manager.c` have comments about triggering MQTT on connect — but **no MQTT client component is wired into `CMakeLists.txt`** yet (the `espressif/mqtt` dependency is declared in `idf_component.yml` but unused in `main/`). Treat MQTT as designed-for but not implemented.

## Config (`config.c`)

Single NVS namespace `"its"`. `config_index_t` (`config.h`) is the canonical key enum; `config_keys[]` maps each index to its NVS key name, type, and an optional custom getter (used for computed defaults, e.g. `nodeid` falls back to the Ethernet MAC formatted as hex if unset). All persistent settings — node id, WiFi creds, MQTT URI, Ethernet static IP, sniffer/WiFi cycle timing — go through `config_get_*`/`config_set_*`, never raw `nvs_*` calls from other modules (the one exception is `bt_stream.c`, which keeps its coex-cycle config in its own NVS namespace `btstream` rather than going through `config.c` — those values aren't part of `config_index_t`).

## Events (`events.c`/`events.h`)

Four `esp_event` bases: `APP_EVENT_BASE` (boot, Ethernet mgmt link/IP up/down), `CONFIG_EVENT_BASE` (events are `config_index_t` values, posted by every `config_set_*` in `config.c` — no subscriber exists in `main/` yet), `SNIFFER_EVENT_BASE` (start/stop/packet-received), `MQTT_EVENT_BASE` (connected/disconnected — see MQTT gap above).

## LED (`led.c`)

Two display models share the same driver, selected by board layout:
- **Single WS2812** (Waveshare-style devboards): LED 0 doubles as activity+status — idle background reflects sniffer-running/BLE-connected state (`led_status_sniffer_running`, `led_status_ble_connected`), and `led_pulse_sniffer_rx/ble_tx/usb_tx` briefly flash green/blue/cyan over it per event without disturbing the background.
- **Legacy 4-LED i5r-r1 layout**: indices 1–4 are dedicated System/Sniffer/Ethernet/MQTT(+CITS) indicators, each with its own blink/color state machine (e.g. Ethernet LED color encodes link speed: green=100M, amber=10M; MQTT LED: green=connected, yellow=disconnected, off=handler destroyed).
`CONFIG_INDEX_LED_BRIGHTNESS` scales all RGB output globally (`set_led_with_brightness`).

## OTA (`ota.c`)

HTTPS OTA (`esp_https_ota`, cert via `esp_crt_bundle`, SPIRAM-backed buffers) into the non-running app partition (`esp_ota_get_next_update_partition`). `ota` console command: `-u <url> [-r]` (update, optional reboot), `-C` (confirm/cancel rollback), `-R` (force rollback+reboot), `-S` (switch boot partition), `-i` (partition info).
**Anti-brick safety net**: if the running partition isn't marked `VALID` at boot, a 30-minute one-shot timer (`invalidate_and_reboot_timer`) is armed that force-rolls-back if the new firmware never calls `ota -C` to confirm itself. `HW_VARIANT` (`Kconfig.projbuild`) is a hardware-compatibility tag intended to gate OTA feeds, but it is **not referenced anywhere in firmware C code** — it exists only as a Kconfig value today; any enforcement happens outside this codebase (e.g. external OTA-feed tooling), if at all.

## Hardware variants

- **Custom hardware** (OpenTrafficMap `its-g5-receiver`) — the board this codebase was originally written for. No extra wiring; this is the implicit baseline for default pin choices.
- **Waveshare ESP32-C5-WIFI6-KIT devboard — current target hardware.** See dedicated section below.
- Any hardware/sdkconfig deviation from a known board → set `HW_VARIANT=custom`, otherwise the device may pull an OTA image built for different hardware (though note `HW_VARIANT` isn't actually enforced in firmware code — see OTA section above).
- **Doc gap**: `skills/build_deploy.md` references `sdkconfig.proto-w5500`/`sdkconfig.proto-enc28j60` as copy-over base configs for devboard use — **neither file exists in this repo.** There is currently no working "copy this file" path for devboard Ethernet; if you need it, it has to be created from scratch (see Ethernet note below for why it's currently disabled instead).

### Waveshare ESP32-C5-WIFI6-KIT specifics

Verified against the board's published pinout diagram (`docs/hardware/`) and the ESP32-C5-WROOM-1 datasheet/TRM.

- **Two USB-C ports, two different peripherals** — confirmed against real `dmesg` output from this board:
  - **"UART" port** → external USB-UART bridge chip → ESP32-C5 UART0 (GPIO11/U0TXD, GPIO12/U0RXD). This is where the `esp_console` REPL lives (`CONFIG_ESP_CONSOLE_UART_DEFAULT=y`). Shows up as a CH9102-style "USB Single Serial" device.
  - **"USB" port** → the chip's native USB-Serial-JTAG peripheral (GPIO13/USB_D-, GPIO14/USB_D+). This is what `usb_stream.c` opens (`usb_serial_jtag_driver_install`) for the `"ITS5"` packet stream + `CFG:`/`SCAN`/`REBOOT` protocol. Shows up as "USB JTAG/serial debug unit", Espressif VID `303a`.
  - These don't conflict — different hardware peripherals, can run simultaneously.
  - **The native USB-Serial-JTAG controller cannot be repurposed as a USB-Ethernet (CDC-ECM/NCM/RNDIS) device.** Per the datasheet (§5.2.1.5): "USB 2.0 full speed... CDC-ACM virtual serial port and JTAG adapter functionality" — it's fixed-function (serial + JTAG only), not a general-purpose USB device stack. There is no other USB-OTG peripheral on this chip. Confirmed: this board cannot get Ethernet over either USB port.
- **GPIO27 = onboard addressable RGB LED** (visible in the pinout diagram, between the two USB-C ports). The firmware's `CONFIG_LEDSTRIP_PIN` is now set to **27** (was 26, which drives nothing on this board) so `led.c` controls the LED that's actually populated.
- **No Ethernet module wired** (confirmed) — disabled in `sdkconfig`: `CONFIG_ETHERNET_SPI_SUPPORT` is unset (no SPI Ethernet device gets instantiated; `ethernet_init_all()` cleanly returns 0 ports, `main/ethernet.c::initialize_ethernet()` already handles that as a no-op). `CONFIG_ETH_USE_SPI_ETHERNET=y` is deliberately left **on** despite this — it's a separate, lower-level esp-idf switch that just keeps `esp_eth_mac_spi.h` available; the vendored `managed_components/espressif__w5500` component includes that header unconditionally regardless of whether SPI Ethernet is actually used, and turning this off breaks the build (`Missing "esp_eth_mac_spi.h"`) even though no W5500 code path runs.
- **GPIO6 capacitor erratum** (this board specifically): a 100 nF cap on GPIO6 blocks SPI clock signal integrity unless zero-ohm resistor `R39` (right of the 5V pin) is desoldered. Previously latent (the now-removed SD-card SPI clock pin defaulted to GPIO6, and `main.c` claimed it unconditionally at boot regardless of whether a card was mounted). No longer relevant — see SD card removal below — but worth remembering if any future SPI peripheral on this board is tempted to use GPIO6.
- **SD card support removed entirely.** This board has no SD card slot. Removed: `main/sdcard.c`/`.h`, the `mount`/`unmount` console commands, the `ENABLE_SD`/`SNIFFER_SD_MODE`/`SNIFFER_SD_SPI_MODE`/`SNIFFER_PIN_{MOSI,MISO,CLK,CS}`/`SNIFFER_MOUNT_POINT` Kconfig options, the `SNIFFER_PCAP_DESTINATION_SD` pcap choice (`cmd_pcap.c`'s SD-destination code paths are still present but now permanently unreachable — vendored Espressif example code, left alone like the JTAG/Memory branches it's structured alongside), and the SD-SPI branch in `spi.c` (Ethernet's SPI branch is kept — see Ethernet note above). Pcap destination default is now `SNIFFER_PCAP_DESTINATION_MEMORY`. If a future board has a real SD slot, this needs re-adding from scratch — avoid GPIO6 (capacitor erratum) and GPIO15–22 (flash/PSRAM, see I2C note below) when picking pins.
- Per the TRM, **boot-strapping pins** on this chip are GPIO7 (JTAG signal source, no internal pull), GPIO25/26/27/28 (boot mode), and MTDI/GPIO3 (clock-edge sampling). The Ethernet pin choices inherited from the custom-hardware defaults (`CONFIG_ETHERNET_SPI_*_GPIO`: SCLK=8 MOSI=10 MISO=9 CS0=3 INT0=4) touch GPIO3 — worth keeping in mind if/when Ethernet wiring for this board is revisited, though no concrete problem has been observed (not currently exercised since Ethernet is disabled).
- **GPIO15–22 are reserved for embedded flash/PSRAM on this module — confirmed by a real bootloop, not just datasheet reading.** This codebase used to have an I2C-attached LM75 temperature sensor (`temperature.c`, custom-hardware-only feature, since **removed entirely** — see below); its default pins (GPIO21/22) corrupted the chip's own flash/PSRAM access when reconfigured for I2C, causing a confirmed `CPU_LOCKUP` → `TG1_WDT_HPSYS` bootloop. A first fix attempt moved to GPIO15/23, trusting the Waveshare marketing pinout diagram that draws GPIO15 as a plain broken-out pin — **that diagram is wrong/incomplete**: GPIO15 is also flash/PSRAM-reserved (`esp_gpio_reserve()` flagged it at runtime — `"GPIO 15 is not usable, maybe conflict with others"` — followed by a hard crash in `spi_flash_chip_generic_set_io_mode`). The datasheet §5.2.1.2 is the authoritative source: "SPI0/1... multiplexed with **GPIO15~18 and GPIO20~22**" via the IO MUX. **Treat GPIO15–22 as unusable on this board for anything**, regardless of what the pinout diagram shows. If you need I2C or another GPIO-hungry peripheral here in the future, watch the boot log for `esp_gpio_reserve`-driven `"GPIO %d is not usable"` warnings before trusting a new pin choice.
- **LM75/I2C temperature sensor support removed entirely** (not just disabled) — this board has no LM75 chip, and the custom OpenTrafficMap hardware was the only consumer. Removed: `main/temperature.c`/`.h`, the `temperature.c` SRCS entry and `CONFIG_ENABLE_TEMPERATURE` Kconfig option, the `temperature_init()` call in `app_main`, and the `esp-idf-lib/lm75` (+ transitively `esp-idf-lib/i2cdev`) dependency from `idf_component.yml`. If a future board needs an I2C sensor again, it'll need to be re-added from scratch — pick pins outside GPIO15–22.

## Other modules

- `spi.c` — SPI bus init for the Ethernet module (SD card support removed — see Waveshare specifics above).
- `tx_custom.c` — thin wrapper exposing `esp_wifi_80211_tx_custom` for raw 802.11p frame injection (TX side of ITS-G5, currently just the primitive — no caller in `main/` yet).

## Legal

The 5875–5935 MHz band is safety-critical ITS spectrum in the EU. Transmitting (TX path, `tx_custom.c`/802.11p PHY) requires complying with local regulations — this is a hard constraint on any TX-enabling work, not just a README disclaimer.
