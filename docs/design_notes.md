# Design Notes

Central implementation/design reference for the ITS-G5 receiver/transmitter firmware (ESP32-C5).
Built for Waveshare ESP32-C5-WIFI6-KIT

Build/flash commands: `skills/build_deploy.md`. Hard agent rules: `AGENTS.md`.

## Purpose

Sniffs 802.11p (ITS-G5, 5.9 GHz) frames and fans each captured frame out to up to two sinks simultaneously:
 - BLE GATT notifications
 - a USB-CDC stream

A connected client (BLE or USB) can also submit a preformatted raw 802.11 frame (header onward) for
transmission in the same band — see "Wire protocol" and "Transmit pipeline" below. WiFi STA/Ethernet/MQTT
plumbing exists in the config layer but is not wired to anything live (see "Connectivity" below) — it is not
required for capture or transmit.

## Boot sequence (`main/main.c::app_main`)

Single FreeRTOS app, no custom task framework beyond ESP-IDF's own. Order matters:
`esp_event_loop_create_default → config_init → led_init → filesystem (history FAT) → SPI → WiFi driver init
(esp_wifi_init, STA mode, started but never connected) → console REPL → wifi_init → sniffer_init → ota_init →
register all esp_console commands (sniffer, tx, reboot, loglevel, ota, config) → usb_stream_init →
usb_config_rx_start → bt_stream_init → wifi_autostart → usb_stream_send_ping → console_start_repl → led_update`.

All three external sinks/inputs — USB frame stream, USB config-RX (`CFG:`/`SCAN`/`REBOOT`), and BLE — are
initialized unconditionally from `app_main` today; there is no dormant/not-wired-up module left over from
earlier iterations of this codebase (that used to be true of an earlier revision — it no longer is, don't
trust older assumptions here without re-checking `main.c`).

Most features are also exposed as `esp_console` commands (`sniffer`, `tx`, `ota`, config setters, `reboot`,
`loglevel`) registered at startup — the REPL is a secondary control surface for bench use; the primary control
path for an attached client (phone/host app) is the binary wire protocol over BLE/USB described next.

## Wire protocol (`main/obu_comm.c`)

Both external transports (USB-CDC via `usb_stream.c`, BLE GATT via `bt_stream.c`) speak the same
length-prefixed binary framing and share one dispatcher, so the protocol only needs describing once:

- `CommHeader` (`obu_comm.h`, packed, network byte order): `magic(u16)=0x7384 | seqNo(u16) | type(u16) | length(u16)`,
  followed by `length` bytes of payload. `obu_comm_reassembler_t` + `obu_comm_feed()`/`obu_comm_feed_inplace()`
  incrementally reassemble frames out of a transport's raw byte stream (USB: read straight into the reassembly
  buffer, `COMM_MAX_BUF_SIZE` = 64 KiB; BLE: reassembled per-write-chunk into a smaller 2 KiB buffer, control
  traffic only). An invalid magic or an oversized `length` resets the reassembler and resyncs on the next magic.
- `obu_comm_process_frame()` dispatches by `type` (`obu_comm.h`):
  - `COMM_FRAME_PING` → replies `COMM_FRAME_PONG` with a small JSON status blob (`{"txpower":...,"rx":...,"rxErr":...,"tx":...,"txErr":...}`, `wifi_get_txpower()`/`wifi_get_stats()`).
  - `COMM_FRAME_PONG` → logged only (host replying to our own boot-time ping, see `usb_stream_send_ping()`).
  - `COMM_FRAME_CMD_REBOOT` → replies `COMM_FRAME_CMD_GOOD`, then `esp_restart()`.
  - `COMM_FRAME_CMD_CONFIG` → payload is a `CFG:key=value` string (same mini-protocol as below), replies `COMM_FRAME_CMD_GOOD`/`_FAIL`.
  - `COMM_FRAME_TX` → payload is `CommFrameHeader` + raw 802.11 frame bytes, handed to `wifi_tx_frame()` (see Transmit pipeline). No reply is sent either way.
  - `COMM_FRAME_RX` → sink-only, sent *from* device to client for each captured frame (not dispatched, no client can send it in).
- `CFG:key=value` sub-protocol (`handle_config_command`/`cfg_key_to_index` in `obu_comm.c`): maps a fixed set of
  string keys to `config_index_t` entries in `config.c` and calls `config_set_str/u8/u32`. Currently mapped:
  `nodeid, mqtturi, wifi1ssid, wifi1pass, wifi2ssid, wifi2pass, wifiip, wifinm, wifigw, wifidns, wifiopen,
  connmode, sniffmode, sniffms, wifims`. **Several of these are accepted and persisted but not consumed by any
  code path** — see "Dead/unwired config keys" below; a setup wizard will get `CFG_OK` back for them and nothing
  will actually change behavior.

## Capture pipeline (RX) — `main/obu_wifi.c`

- Owns `esp_wifi` promiscuous mode and the 802.11p PHY. `phy_change_channel`/`phy_11p_set` are forward-declared
  `extern` with no header: `phy_11p_set` resolves to a mask-ROM address
  (`esp-idf/components/esp_rom/esp32c5/ld/esp32c5.rom.eco3.ld`), `phy_change_channel` to the precompiled
  `esp_wifi` blob libs (`esp-idf/components/esp_wifi/lib/`) — not in any source you can grep. How channel tuning
  actually works (the 4-step PHY dance, frequency-vs-channel-number confusion, known rough edges):
  `docs/radio_80211p_notes.md`.
- `wifi_receive_cb` (promiscuous RX callback) builds a `CommFrameHeader` (channel, rate, rssi, noise, timestamp,
  frame length) and queues `{header, raw frame}` to `work_queue` (`CONFIG_SNIFFER_WORK_QUEUE_LEN`, default 128)
  via `queue_packet()`.
- `wifi_task` drains the queue and fans each packet to `usb_stream_publish_packet()` →
  `bt_stream_publish_packet()`, then posts `WIFI_RECEIVED_PACKET` (drives the LED) and frees the buffer. There is
  no pcap/SD sink in this codebase — that used to be true of an earlier iteration built around a vendored
  Espressif `cmd_pcap.c`/`cmd_sniffer.c` sniffer example; that code (and `sdcard.c`, `temperature.c`,
  `wifi_manager.c`, `ethernet.c`) is gone from this tree. `obu_wifi.c` includes `pcap.h` (still a component
  dependency) but calls nothing from it — dead include.
- `wifi_autostart()` reads `CONFIG_INDEX_AUTOSTART_CHAN`; only starts if channel is in `5800..5900` (802.11p band
  sanity check). `sniffer_pause/resume` (`wifi_pause`/`wifi_resume`) are the radio hand-off primitives the BLE
  coex scheduler (below) relies on.
- `wifi_stats_t` (`rx/rxErr/tx/txErr` counters, `wifi_get_stats()`) is reported back to a client via
  `COMM_FRAME_PING` → `COMM_FRAME_PONG`.

## Transmit pipeline (TX) — `main/obu_wifi.c`, `main/cmd_tx.c`

- `wifi_tx_frame(payload, length)`: payload is `CommFrameHeader` + raw 802.11 frame bytes (same header struct as
  RX, though only `frameLen` is meaningful here — it must equal `length - sizeof(CommFrameHeader)` or the frame
  is rejected as a length mismatch). Validates the header actually fits before subtracting its size off
  `length` (guards against a short/malformed `COMM_FRAME_TX` frame underflowing the `uint16_t` length). Picks a
  PHY rate from `tx_datarate` (`wifi_set_datarate()`, default 6 Mbps) and calls `esp_wifi_80211_tx_custom()`.
- `esp_wifi_80211_tx_custom()` is a from-scratch reimplementation of the (not-public) raw-TX hook into the
  vendored `esp_wifi` blob's internal ebuf/txdesc structures (`x_ebuf_t`/`x_eb_txdesc_t`, field names are
  guesses reverse-engineered from disassembly, hence `field_XX` — treat any change here as high-risk).
  `ieee80211_raw_frame_sanity_check()` is deliberately **not called** (commented out) — there is no independent
  sanity check on the frame content before it reaches the driver; `wifi_tx_frame`'s length check above is the
  only guard on the whole path.
- TX power: `wifi_set_txpower(int dBm)` clamps to 2..20 dBm and calls `esp_wifi_set_max_tx_power()`, which takes
  units of 0.25 dBm — the conversion happens once, in this function. Reachable from: the `tx -p <dbm>` console
  command, and `CONFIG_INDEX_WIFI_TX_POWER` (`"txpower"` NVS key) read back by `cmd_sniffer.c` on `sniffer`
  command autostart. **Note:** `config.c`'s own setter for that NVS key, `config_set_tx_power()`, validates the
  *stored* value against `8..84` (i.e. expects the value already in quarter-dBm units) but then calls
  `wifi_set_txpower(value * 4)` (which expects plain dBm and multiplies by 4 itself) — the validation range and
  the value's actual unit disagree with each other and with `cmd_sniffer.c`'s own read-back (`2..20`, plain
  dBm). Nothing currently drives `config_set_tx_power` from the wire protocol (`"txpower"` isn't in
  `cfg_key_to_index`), so this is latent rather than field-triggered, but do not use `config_set_*` to change TX
  power until this is reconciled — use the `tx -p` console command or fix the setter first.
- `cmd_tx.c`'s `tx` console command transmits a hardcoded sample frame (`SAMPLE_PACKET_HEX`) on a timer
  (`-i <ms>`, default 1000 ms; `--stop` halts it) for bench testing without a connected client. Note the hex
  blob has a synthetic 14-byte `CommFrameHeader` prefix spliced onto the real captured frame specifically so it
  passes `wifi_tx_frame`'s length check — don't mistake the first 14 bytes for part of the 802.11 frame when
  reading/editing that constant.
- Starting `tx` disables the BLE/sniffer coex scheduler (`wifi_coex_disable()`) for the lifetime of the task —
  transmitting and the sniff/BLE duty-cycle below don't run concurrently.

## BLE ⟷ sniffer coexistence (`main/bt_stream.c::coex_window_task`)

The ESP32-C5 has a single RF transceiver shared between 5 GHz WiFi/802.11p and 2.4 GHz BLE — it cannot service
WiFi promiscuous RX and BLE advertising/connections at the same time. `coex_window_task` alternates: promiscuous
mode on for a sniff window (`wifi_resume()`) → off for a BLE window (`wifi_pause()`, drops promisc + 802.11p PHY
+ `esp_wifi_stop`) → repeat. This exists so BLE scanners (notably older Android stacks with slow scan duty
cycles — Galaxy S6 was the motivating case) actually see the advertisement between sniffer bursts.

- Durations are NVS-persisted (`btstream`/`cycle`) and runtime-tunable via the BLE config characteristic:
  `CYC_DISC_SNIFF/CYC_DISC_BLE` (no client, default 10000/2000 ms) and `CYC_CONN_SNIFF/CYC_CONN_BLE` (client
  connected, default 800/400 ms) — connected clients get serviced more often since NimBLE's software coex layer
  can handle scheduled connection events correctly once linked.
- `chunked_delay()` sleeps in 200 ms steps so a BLE connect/disconnect mid-window is noticed quickly rather than
  starving the link for the full window.
- `esp_coex_preference_set(ESP_COEX_PREFER_BT)` is set at BLE init so WiFi yields enough airtime for a full BLE
  adv packet to go out; flipped to `ESP_COEX_PREFER_WIFI` during a USB-triggered WiFi scan
  (`usb_stream.c::cfg_scan_task`) and disabled entirely (`wifi_coex_disable()`) while the USB config-RX channel
  has recent traffic or while `tx` is transmitting, so those don't fight the coex scheduler for the radio.
- This is now the *only* WiFi/radio scheduling in the codebase — there is no separate "sniffer ⟷ WiFi STA"
  cycle mode layered on top of it (see Connectivity below for why).

## `main/bt_stream.c` — NimBLE GATT server

Service `b6e57e90-12d8-4a47-9b21-3f0000000001`, two characteristics:
- **Notify/Write** (`...02`) — carries the same `CommHeader`-framed wire protocol described above (not a
  bespoke `"ITS5"` framing). `bt_writer_task` drains a queue (depth 16, drop-newest on overflow) and chunks each
  frame to `ble_att_mtu - 3` per `ble_gatts_notify_custom` call. Writes from the phone are queued to
  `bt_reader_task`, reassembled the same way as USB, and dispatched through `obu_comm_process_frame`.
- **Config** (`...03`) — 8 bytes = 4×`u16` LE, the coex cycle timings above; persisted to NVS namespace
  `btstream` on write. Unrelated to the `config.c` "its" namespace / `CFG:` protocol.
- Forces the controller's BT MAC as a **public** BLE address (`ble_hs_id_set_pub`, a private NimBLE API) because
  random-static addresses are silently ignored by some older Android BLE stacks (Galaxy S6) that haven't paired
  with that address before.

## `main/usb_stream.c` — USB-CDC transport

Same `CommHeader` framing over USB-CDC (`usb_serial_jtag` driver, 4 KB TX / 256 B RX buffers). Also runs a
line-oriented RX task (`usb_config_rx_task`) that feeds the same `obu_comm_feed_inplace`/dispatch path as
above, plus two USB-only text commands (handled before/alongside the binary protocol on that same read):
- `SCAN\n` → spawns `cfg_scan_task`: pauses BLE, flips coex preference to WiFi, brings up STA just long enough
  to active-scan (country `DE`, channels 1–13), streams `SCAN_AP:ssid,chN,authM\n` lines, then restores
  BLE/coex.
- `REBOOT\n` → acks then `esp_restart()` (also reachable as `COMM_FRAME_CMD_REBOOT` over the binary protocol).

## Connectivity (config surface exists, nothing behind it runs)

`main.c` brings WiFi up in STA mode (`esp_wifi_set_mode(WIFI_MODE_STA)`, `esp_wifi_start()`) purely so the
promiscuous-mode sniffer has a driver to attach to — **it never associates to an access point.** There is no
`wifi_manager.c`/STA-connect logic, no `ethernet.c`, and no MQTT client wired into `CMakeLists.txt` in this
codebase (an earlier iteration of this project had all three; they were removed along with pcap/SD/temperature
support when this tree was rebuilt around the BLE/USB wire protocol above). Config keys for all of them still
exist in `config.c` and some are still settable via `CFG:` — see below.

### Dead/unwired config keys

These are accepted by `config_set_*`/`CFG:key=value` (persisted to NVS) but read back by **no code** in
`main/`, so setting them silently does nothing observable:
- `wifi1ssid/pass`, `wifi2ssid/pass`, `wifiopen`, `wifiip/nm/gw/dns` — WiFi STA credentials/static-IP; no STA
  connect logic exists to consume them.
- `connmode`, `sniffmode`, `sniffms`, `wifims` — remnants of the removed WiFi/sniffer duty-cycle mode.
- `ethip/nm/gw/dns0/1/2` — Ethernet static config; no `ethernet.c` in this tree. Not reachable via `CFG:` either
  (missing from `cfg_key_to_index`), only via raw NVS/console.
- `mqtturi` — no MQTT client component wired into `CMakeLists.txt` (the `espressif/mqtt` dependency is declared
  in `idf_component.yml` but unused in `main/`).
- `broadcastonly` — has an NVS default-value getter (`config_get_broadcast_only`) but nothing reads the value
  back to actually filter broadcast frames.

Live/consumed config keys: `nodeid` (console prompt, falls back to Ethernet-MAC-as-hex if unset even though
Ethernet itself is dead — cosmetic only), `ledbrightness` (`led.c`), `autostartchan` (`wifi_autostart`),
`txpower` (`cmd_sniffer.c`, see the unit mismatch note in Transmit pipeline above).

If any of this connectivity is meant to come back (WiFi STA for management, Ethernet, MQTT), it needs
re-implementing from scratch against the current `obu_wifi.c`/`config.c` — don't assume the removed
`wifi_manager.c`/`ethernet.c` can just be restored, the surrounding code (events, LED, coex) has moved on.

## Events (`events.c`/`events.h`)

Three `esp_event` bases: `CONFIG_EVENT_BASE` (events are `config_index_t` values, posted by every
`config_set_*` in `config.c` — no subscriber exists in `main/`), `WIFI_EVENT_BASE`
(`WIFI_STARTED/STOPPED/RECEIVED_PACKET/TRANSMIT_PACKET`, posted by `obu_wifi.c`, subscribed by `led.c` to drive
the sniffer/TX activity LED), `COMM_EVENT_BASE` (`COMM_CONNECTED/DISCONNECTED`, subscribed by `led.c` but
**never posted anywhere** — the comm-connected LED color path is wired up but currently unreachable dead code).

## LED (`led.c`)

Only the single-WS2812 model is actually live today: LED index 0 doubles as activity+status — idle background
encodes sniffer-running/BLE-connected state (`apply_activity_led`), and `led_pulse_wifi_rx/wifi_tx/ble_tx/usb_tx`
briefly flash over it per event without disturbing the background. `CONFIG_INDEX_LED_BRIGHTNESS` scales all RGB
output globally (`set_led_with_brightness`). `led_init()` also lights strip indices 1–4 once at boot as a
self-test sweep, but nothing updates them afterward — the previously-documented "legacy 4-LED i5r-r1 layout"
(dedicated System/Sniffer/Ethernet/MQTT indicators) is not wired up in this tree; treat `LED_WIFI`/`LED_COMM`/
`LED_CITS` state variables and their setter functions as currently-unused scaffolding, not a live display mode.

## OTA (`ota.c`)

HTTPS OTA (`esp_https_ota`, cert via `esp_crt_bundle`, SPIRAM-backed buffers) into the non-running app partition
(`esp_ota_get_next_update_partition`). `ota` console command: `-u <url> [-r]` (update, optional reboot), `-C`
(confirm/cancel rollback), `-R` (force rollback+reboot), `-S` (switch boot partition), `-i` (partition info).
**Anti-brick safety net**: if the running partition isn't marked `VALID` at boot, a 30-minute one-shot timer
(`invalidate_and_reboot_timer`) is armed that force-rolls-back if the new firmware never calls `ota -C` to
confirm itself. `HW_VARIANT` (`Kconfig.projbuild`) is a hardware-compatibility tag intended to gate OTA feeds,
but it is **not referenced anywhere in firmware C code** — it exists only as a Kconfig value today; any
enforcement happens outside this codebase (e.g. external OTA-feed tooling), if at all.

## Hardware variants

- **Custom hardware** (OpenTrafficMap `its-g5-receiver`) — the board this codebase was originally written for.
  No extra wiring; this is the implicit baseline for default pin choices.
- **Waveshare ESP32-C5-WIFI6-KIT devboard — current target hardware.** See dedicated section below.
- Any hardware/sdkconfig deviation from a known board → set `HW_VARIANT=custom`, otherwise the device may pull
  an OTA image built for different hardware (though note `HW_VARIANT` isn't actually enforced in firmware code
  — see OTA section above).
- **Doc gap**: `skills/build_deploy.md` references `sdkconfig.proto-w5500`/`sdkconfig.proto-enc28j60` as
  copy-over base configs for devboard use — **neither file exists in this repo.** There is currently no working
  "copy this file" path for devboard Ethernet, and Ethernet isn't wired into the firmware at all right now
  regardless (see Connectivity above).

### Waveshare ESP32-C5-WIFI6-KIT specifics

Verified against the board's published pinout diagram and the ESP32-C5-WROOM-1 datasheet/TRM. The pinout
image/datasheet/TRM previously kept under `docs/hardware/` have been removed from the repo and gitignored
(likely licensing — Espressif/Waveshare copyrighted material); the bullet points below are what was verified
against them while those files were still present, treat them as the record now that the source files are gone.

- **Two USB-C ports, two different peripherals** — confirmed against real `dmesg` output from this board:
  - **"UART" port** → external USB-UART bridge chip → ESP32-C5 UART0 (GPIO11/U0TXD, GPIO12/U0RXD). This is where
    the `esp_console` REPL lives (`CONFIG_ESP_CONSOLE_UART_DEFAULT=y`). Shows up as a CH9102-style "USB Single
    Serial" device.
  - **"USB" port** → the chip's native USB-Serial-JTAG peripheral (GPIO13/USB_D-, GPIO14/USB_D+). This is what
    `usb_stream.c` opens (`usb_serial_jtag_driver_install`) for the wire-protocol stream. Shows up as "USB
    JTAG/serial debug unit", Espressif VID `303a`.
  - These don't conflict — different hardware peripherals, can run simultaneously.
  - **The native USB-Serial-JTAG controller cannot be repurposed as a USB-Ethernet (CDC-ECM/NCM/RNDIS) device.**
    Per the datasheet (§5.2.1.5): "USB 2.0 full speed... CDC-ACM virtual serial port and JTAG adapter
    functionality" — it's fixed-function (serial + JTAG only), not a general-purpose USB device stack. There is
    no other USB-OTG peripheral on this chip. Confirmed: this board cannot get Ethernet over either USB port.
- **GPIO27 = onboard addressable RGB LED** (visible in the pinout diagram, between the two USB-C ports). The
  firmware's `CONFIG_LEDSTRIP_PIN` is now set to **27** (was 26, which drives nothing on this board) so `led.c`
  controls the LED that's actually populated.
- **No Ethernet module wired** (confirmed) — disabled in `sdkconfig`: `CONFIG_ETHERNET_SPI_SUPPORT` is unset (no
  SPI Ethernet device gets instantiated). `CONFIG_ETH_USE_SPI_ETHERNET=y` is deliberately left **on** despite
  this — it's a separate, lower-level esp-idf switch that just keeps `esp_eth_mac_spi.h` available; the
  vendored `managed_components/espressif__w5500` component includes that header unconditionally regardless of
  whether SPI Ethernet is actually used, and turning this off breaks the build (`Missing "esp_eth_mac_spi.h"`)
  even though no W5500 code path runs (there's no `ethernet.c` in this tree at all now — see Connectivity).
- **GPIO6 capacitor erratum** (this board specifically): a 100 nF cap on GPIO6 blocks SPI clock signal integrity
  unless zero-ohm resistor `R39` (right of the 5V pin) is desoldered. Previously latent (the now-removed
  SD-card SPI clock pin defaulted to GPIO6, and `main.c` claimed it unconditionally at boot regardless of
  whether a card was mounted). No longer relevant — see SD card removal below — but worth remembering if any
  future SPI peripheral on this board is tempted to use GPIO6.
- **SD card support removed entirely.** This board has no SD card slot. Removed: `main/sdcard.c`/`.h`, the
  `mount`/`unmount` console commands, the `ENABLE_SD`/`SNIFFER_SD_MODE`/`SNIFFER_SD_SPI_MODE`/
  `SNIFFER_PIN_{MOSI,MISO,CLK,CS}`/`SNIFFER_MOUNT_POINT` Kconfig options, `cmd_pcap.c` and its
  `SNIFFER_PCAP_DESTINATION_*` build-time destinations entirely (JTAG/App-Trace, in-memory ring, SD — all gone
  along with the file, not just the SD branch), and the SD-SPI branch in `spi.c` (Ethernet's SPI branch is kept
  — see Ethernet note above, even though nothing currently instantiates an Ethernet device on this board). If a
  future board has a real SD slot, this needs re-adding from scratch — avoid GPIO6 (capacitor erratum) and
  GPIO15–22 (flash/PSRAM, see I2C note below) when picking pins.
- Per the TRM, **boot-strapping pins** on this chip are GPIO7 (JTAG signal source, no internal pull), GPIO25/26/27/28
  (boot mode), and MTDI/GPIO3 (clock-edge sampling). The Ethernet pin choices inherited from the custom-hardware
  defaults (`CONFIG_ETHERNET_SPI_*_GPIO`: SCLK=8 MOSI=10 MISO=9 CS0=3 INT0=4) touch GPIO3 — worth keeping in
  mind if/when Ethernet wiring for this board is revisited, though no concrete problem has been observed (not
  currently exercised since Ethernet is disabled).
- **GPIO15–22 are reserved for embedded flash/PSRAM on this module — confirmed by a real bootloop, not just
  datasheet reading.** This codebase used to have an I2C-attached LM75 temperature sensor (`temperature.c`,
  custom-hardware-only feature, since **removed entirely** — see below); its default pins (GPIO21/22) corrupted
  the chip's own flash/PSRAM access when reconfigured for I2C, causing a confirmed `CPU_LOCKUP` →
  `TG1_WDT_HPSYS` bootloop. A first fix attempt moved to GPIO15/23, trusting the Waveshare marketing pinout
  diagram that draws GPIO15 as a plain broken-out pin — **that diagram is wrong/incomplete**: GPIO15 is also
  flash/PSRAM-reserved (`esp_gpio_reserve()` flagged it at runtime — `"GPIO 15 is not usable, maybe conflict
  with others"` — followed by a hard crash in `spi_flash_chip_generic_set_io_mode`). The datasheet §5.2.1.2 is
  the authoritative source: "SPI0/1... multiplexed with **GPIO15~18 and GPIO20~22**" via the IO MUX. **Treat
  GPIO15–22 as unusable on this board for anything**, regardless of what the pinout diagram shows. If you need
  I2C or another GPIO-hungry peripheral here in the future, watch the boot log for `esp_gpio_reserve`-driven
  `"GPIO %d is not usable"` warnings before trusting a new pin choice.
- **LM75/I2C temperature sensor support removed entirely** (not just disabled) — this board has no LM75 chip,
  and the custom OpenTrafficMap hardware was the only consumer. Removed: `main/temperature.c`/`.h`, the
  `temperature.c` SRCS entry and `CONFIG_ENABLE_TEMPERATURE` Kconfig option, the `temperature_init()` call in
  `app_main`, and the `esp-idf-lib/lm75` (+ transitively `esp-idf-lib/i2cdev`) dependency from
  `idf_component.yml`. If a future board needs an I2C sensor again, it'll need to be re-added from scratch — pick
  pins outside GPIO15–22.

## Other modules

- `spi.c` — SPI bus init for the Ethernet module (SD card support removed — see Waveshare specifics above; note
  Ethernet itself has no live consumer either — see Connectivity above).
- `config.c` — single NVS namespace `"its"`. `config_index_t` (`config.h`) is the canonical key enum;
  `config_keys[]` maps each index to its NVS key name, type, and optional custom getter/setter (e.g. `nodeid`
  falls back to the Ethernet MAC formatted as hex if unset; `txpower` has a custom setter, see the unit-mismatch
  note in Transmit pipeline). All persistent settings go through `config_get_*`/`config_set_*`, never raw
  `nvs_*` calls from other modules — the one exception is `bt_stream.c`, which keeps its coex-cycle config in
  its own NVS namespace `btstream` rather than going through `config.c`.

## Legal

The 5875–5935 MHz band is safety-critical ITS spectrum in the EU. Transmitting (TX path, `obu_wifi.c`'s
`esp_wifi_80211_tx_custom`/`wifi_tx_frame`, driven by `cmd_tx.c` or a client's `COMM_FRAME_TX`) requires
complying with local regulations — this is a hard constraint on any TX-enabling work, not just a README
disclaimer. In particular, verify `wifi_set_txpower()`'s behavior against actual RF output before relying on it
for compliance — see the unit-mismatch note under Transmit pipeline.
