# 802.11p Radio Tuning Notes

How `cmd_sniffer.c` switches the WiFi radio into 802.11p (ITS-G5, 5.9 GHz OCB) mode and tunes frequency. Central design doc: `docs/design_notes.md`.

## Two layers

1. **Standard ESP-IDF WiFi promiscuous mode** — public API (`esp_wifi_set_promiscuous*`). Captures raw 802.11 frames regardless of modulation; doesn't by itself put the radio into 802.11p mode.
2. **Private PHY retuning** — `phy_11p_set()` and `phy_change_channel()`, both `extern`-declared in `cmd_sniffer.c` (lines 173–174) with no header, no public ESP-IDF API, no source you can read in this tree. `phy_11p_set` resolves to a mask-ROM address (`esp-idf/components/esp_rom/esp32c5/ld/esp32c5.rom.eco3.ld`); `phy_change_channel` resolves into the precompiled `esp_wifi` blob libs (`esp-idf/components/esp_wifi/lib/`). These are undocumented hooks Espressif ships in the WiFi blob, likely for their own 802.11p/V2X experimentation — there is no reference for what every parameter does beyond what the source comments themselves guess.

802.11p differs from normal 802.11a at the PHY level: half-clocked timing, 10 MHz-wide channels instead of 20 MHz, "outside the context of a BSS" (OCB) operation — no association/beacon machinery. `phy_11p_set` is what reconfigures the PHY's clocking for this.

## The tuning sequence

From `sniffer_start()` (`cmd_sniffer.c:203–217`), mirrored in `sniffer_resume()` (`:395–407`):

**1. Arm standard promiscuous capture** (still nominally a regular WiFi radio at this point):
```c
wifi_filter.filter_mask = filter;
esp_wifi_set_promiscuous_filter(&wifi_filter);
esp_wifi_set_promiscuous_rx_cb(wifi_sniffer_cb);
esp_wifi_set_promiscuous(true);
```

**2. Enable 802.11p mode on the PHY:**
```c
phy_11p_set(1, 0);
```
First arg is enable/disable. The source comment marks the second arg `unknown (must be 0)` — even the original author wasn't certain what it does.

**3. Coerce the driver into 5 GHz band state via a dummy channel:**
```c
esp_wifi_set_channel(140, WIFI_SECOND_CHAN_NONE);
```
`esp_wifi_set_channel()` is the *public* API, but only accepts real WiFi channel numbers — there's no WiFi channel for 5900 MHz. Channel 140 (5700 MHz) is just close enough to get the driver's internal state into "5 GHz" mode before the real tune. Source comment: `set a channel with a frequency close to our desired frequency (not sure if strictly needed)`. An earlier attempt to pass the raw frequency directly to this call is still visible, commented out, at line 210 — presumably abandoned because it doesn't work (not a valid channel index).

**4. The actual frequency tune:**
```c
phy_change_channel(channel, 1, 0, 0);
```
This is the real switch. `channel` here is a **frequency in MHz**, not a WiFi channel number, despite the variable name. Confirmed by the `sniffer` console command's `-c`/`--channel` help text:
```
G5CC = 5900, G5SC2 = 5890, G5SC1 = 5880, G5SC3 = 5870, G5SC4 = 5860
```
the standard ETSI ITS-G5 control/service channel frequencies. The middle two args are marked `ignored, ignored` in the source comment; the last is `ht_mode?` (set to `0`, presumably keeping plain narrow-channel width, no bonding). Only the first argument is understood with confidence.

## Pause/resume

`sniffer_pause()`/`sniffer_resume()` (used to hand the radio to WiFi STA or BLE — see "shared-radio problem" in `design_notes.md`) do the same dance in reverse/forward: pause disables promiscuous, calls `phy_11p_set(0, 0)`, then `esp_wifi_stop()`; resume calls `esp_wifi_start()` then redoes all four steps above identically. Stopping the WiFi driver apparently resets the PHY state, hence the full redo rather than just re-enabling promiscuous.

## Known rough edges

- **`SNIFFER_DEFAULT_CHANNEL` (`cmd_sniffer.c:30`) is `1`** — a leftover from whatever generic WiFi-channel-number sniffer example this was adapted from. Running `sniffer -P` without `-c` calls `phy_change_channel(1, ...)`, which is not a sensible 802.11p frequency.
- **No range validation on `-c` in the interactive console path** (`do_sniffer_cmd`) — any integer is passed straight through to `phy_change_channel`. Only the NVS-autostart path (`sniffer_autostart`, `cmd_sniffer.c:373`) sanity-checks the value (`5800–5900`).

## TX side: `tx_custom.c`

Everything above is RX. `main/tx_custom.c::esp_wifi_80211_tx_custom()` is the TX-side equivalent — a hand-built, reverse-engineered clone of Espressif's internal raw-TX path, needed because the *public* `esp_wifi_80211_tx()` API doesn't expose PHY rate/mode/bandwidth control. **It has zero callers anywhere in `main/`** — pure unwired infrastructure. It also contains nothing 802.11p-specific itself; "802.11p-ness" is entirely a property of the PHY state set by `phy_11p_set`/`phy_change_channel` above — this function just transmits a raw frame at whatever rate/mode you ask for, on whatever channel the PHY is currently tuned to.

**Reverse-engineered structs** (`x_eb_txdesc_t`, `x_middle_data_t`, `x_ebuf_t`, lines 14–92) mirror Espressif's internal, undocumented WiFi driver buffer/descriptor layout. Most fields are generic `field_N` offsets — only a handful (`rate`, `sched`, `timestamp`, `header_length`, `data_length`, `txdesc`) were understood well enough to name. `static_assert(sizeof(...) == 0x48/0x40/0x10)` on each is the safety net: a future ESP-IDF/blob update that changes these layouts fails the build instead of corrupting memory at runtime. The `x_` prefix marks them as the author's own approximation, not real Espressif type names.

**Four more undocumented `extern` hooks** (lines 6–12), same family as `phy_11p_set`/`phy_change_channel`, no header, no public API:
- `ic_ebuf_alloc` — allocates the driver's internal packet buffer (`ic_` = the BSD net80211-style "interface common" naming Espressif's stack is built on — matches the `net80211 rom version` line in the boot log).
- `ic_get_default_sched` — default TX scheduler/queue context every frame needs.
- `ieee80211_post_hmac_tx` — hands the buffer to the actual hardware MAC TX path; the same function the public `esp_wifi_80211_tx()` ultimately calls internally.
- `ieee80211_raw_frame_sanity_check` — would validate the buffer, but is **commented out** (`esp_wifi_80211_tx_custom`, line 96: `result = 0;//ieee80211_raw_frame_sanity_check(...)`). Whatever buffer you pass goes straight to hardware unvalidated.

**What it does, step by step:**
1. Takes the WiFi driver's own global lock (`g_wifi_global_lock` via the OS-adapter table `g_osi_funcs_p`) — same lock the driver's internal machinery uses, so safe to call concurrently with normal WiFi operation.
2. `ic_ebuf_alloc(buffer, 1, len)` allocates a buffer sized for the raw frame.
3. Sets `header_length = len`, `data_length = 0` — treats the entire input buffer as one opaque blob rather than splitting MAC-header from payload. A commented-out alternate (`header_length = 0x1a` — 26 bytes, a standard 802.11 MAC header) shows a different split was tried and abandoned.
4. Picks a PHY rate: explicit `tx_rate_config->rate` if given, else 6 Mbps (`WIFI_PHY_RATE_6M`) for 5 GHz band, else 0 ("auto"/lowest) otherwise.
5. Packs PHY-mode-specific bits for HE20 (WiFi 6, including ERSU/DCM fields) or VHT20 (802.11ac) — **no special case for plain legacy OFDM or 802.11p**, confirming this function doesn't know about 802.11p at all.
6. Sets a 40 MHz bandwidth bit — source comment admits `// No idea if this is correct, but this is what the original code does...`.
7. Packs the interface index (`ifx`) and an `en_sys_seq` flag (driver-assigned vs caller-supplied 802.11 sequence number — a real, public-API concept too).
8. `ieee80211_post_hmac_tx(eb)` queues it for actual over-the-air transmission; unlocks the mutex; returns `ESP_ERR_NO_MEM` if step 2's allocation failed.

**To actually send 802.11p traffic**, both halves are needed: PHY tuned via `phy_11p_set`/`phy_change_channel` (this doc, above) first, then a fully-formed raw 802.11 frame buffer passed to `esp_wifi_80211_tx_custom()`. Nothing in this codebase currently does the second part.

## Is any of this officially supported by Espressif?

No — but the chip's own ROM says the capability is deliberate, not a fluke.

**No official support.** The ESP32-C5 datasheet, the official ESP-IDF programming guide, and Espressif's product page describe only standard WiFi 6 (802.11ax/a/b/g/n/ac). None mention 802.11p, ITS-G5, V2X, or OCB mode. There's no Kconfig option, example project, app note, or public API for it anywhere in ESP-IDF.

**But it's burned into silicon, not patched in.** Checked directly against the vendored `esp-idf/` submodule:
- `phy_11p_set` isn't in any `.a` library — it's a **mask-ROM function** at a fixed silicon address: `esp-idf/components/esp_rom/esp32c5/ld/esp32c5.rom.eco3.ld:329: phy_11p_set = 0x400015f8;`. Espressif's own hardware/firmware team built 802.11p PHY-mode switching into the chip's ROM. They just never documented, announced, or exposed it through any public API.
- `ieee80211_post_hmac_tx`/`ieee80211_raw_frame_sanity_check` (the TX-side hooks `tx_custom.c` uses) are real symbols defined inside `libnet80211.a` (confirmed via `nm`) — and they're the **same internal functions the public `esp_wifi_80211_tx()` calls**. Not a separate hidden subsystem; the same WiFi stack (net80211-style naming) with a few extra undocumented entry points beyond what's publicly exposed.
- `phy_change_channel` is referenced from a `chm_phy_change_channel` (channel-manager) call site inside `libnet80211.a` but isn't defined as a global symbol in any `.a` checked — presumably ROM-resident too, like `phy_11p_set`, just not matched by name in the linker scripts searched.

**Community usage confirms this is a known, repeatable trick, not a one-off.** Besides this codebase, [pit711/V2X2MAP](https://github.com/pit711/V2X2MAP) explicitly ports this same ITS-G5/802.11p sniffer firmware to the Waveshare ESP32-C5-WIFI6-KIT (this board's target hardware) and adds BLE streaming — per [CNX Software's writeup](https://www.cnx-software.com/2026/05/25/monitor-live-traffic-from-v2x-signals-with-v2x2map-open-source-android-app-and-an-esp32-c5-development-board/). Separately, a patched fork of [esp-hosted-mcu](https://github.com/espressif/esp-hosted-mcu) reportedly exposes "every radio of an Espressif co-processor over SDIO... including 802.11p/ITS-G5/V2X," explicitly marked research/lab-use-only.

**Verdict**: deliberate-but-undocumented silicon capability, with no Espressif acknowledgment and no guarantee it survives a future chip revision or firmware blob update — exactly the fragility flagged in "Known rough edges" above and in the TX section's `static_assert` safety nets. A small community of independent V2X/ITS-G5 hobbyist projects has found and relies on the same undocumented hooks.
