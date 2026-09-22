# ETSI ITS-G5 DCC: Channel Busy Ratio (CBR) on ESP32-C5

Investigation into whether the SDK provides a channel-busy-rate measurement
for ITS-G5 Decentralized Congestion Control (ETSI TS 102 687), and hardware
validation of the undocumented hooks found instead. Central design doc:
`docs/design_notes.md`; 802.11p PHY background: `docs/radio_80211p_notes.md`.

**Status: mechanics and causal link to real channel traffic both confirmed on
hardware** (via a second OBU2X unit as reference load generator — see below).
Not wired into any production DCC logic yet; occasional start-pulse
miss-fires (below) still need handling in any real polling loop.

## No public API

Checked all of `esp-idf/components/esp_wifi/include/*.h` and the wider
`esp_wifi` tree: no `channel_busy`, `chan_util`, `bss_load`, `cca`, or "clear
channel" symbol anywhere. `esp_wifi_statis_dump()` just logs internal driver
stats to console, no structured busy-time output.

## The candidate: `phy_get_cca_cnt`/`phy_set_cca_cnt`

Undocumented symbols in `esp-idf/components/esp_phy/lib/esp32c5/libphy.a`
(`phy_feature.o`), found via `nm`. No caller anywhere in the esp-idf tree —
same undocumented-blob category as `phy_11p_set`/`phy_change_channel` (see
`radio_80211p_notes.md`), just unused by Espressif's own shipped code here.

Reverse-engineered from RISC-V disassembly (`riscv32-esp-elf-objdump`), then
confirmed against real hardware via the `ccatest` console command added to
`main/obu_wifi.c`:

```c
typedef struct {
    uint32_t total; // elapsed PHY clock cycles since last (re)start, ~40MHz, caps at 0x07FFFFFF
    uint32_t busy;  // CCA-busy cycles accumulated over the same window
} phy_cca_cnt_t;
void phy_set_cca_cnt(uint32_t value, int onOff);
bool phy_get_cca_cnt(phy_cca_cnt_t *out);
```

Backing MMIO registers (esp32c5):
- `0x600A7C58` — control reg (window ceiling + start-pulse bits)
- `0x600A7C5C` — `total` counter (28-bit raw, bit 27 = ceiling-reached flag)
- `0x600A7C60` — `busy` counter (28-bit raw, same masking)

## Confirmed hardware behavior

- **`phy_set_cca_cnt(value, onOff)`**: low 27 bits of `value` become a cycle-count
  ceiling in the control reg (top 5 bits preserved). If `onOff != 0`, also
  pulses bits 27–28 — **self-clearing**: reads back as 0 immediately after the
  call, which is why static disassembly alone looked inconsistent with the
  observed register value. The pulse resets and restarts both counters from 0.
- **Clock rate**: `total` free-runs at **~39.8 MHz**, stable across repeated runs
  (39,776,782 / 39,775,920 / 39,765,523 / 39,772,947 ticks measured over ~1.000s
  windows) — consistent with the PHY's 40 MHz reference clock.
- **Ceiling + freeze**: `total` climbs until it hits the configured ceiling
  (masked to 27 bits, e.g. `0x07FFFFFF` = 134,217,727, or `40,000,000` for a
  ~1s window), then **freezes** (doesn't wrap) and the ceiling-reached bit
  (`phy_get_cca_cnt`'s bool return) goes high. `busy` freezes at the same
  instant. Both stay frozen until the next start pulse. Max single-shot window
  at ~40MHz: `134217727 / 40e6 ≈ 3.35s`.
- **`busy` is a real, independent counter**, not noise: it resets to a small
  fresh value on each restart (doesn't just keep climbing), and busy/total
  ratios are small and plausible for a live-but-quiet 5.9GHz channel —
  observed range **0.0%–0.9%** across many samples, both with and without our
  own TX running.
- **Start-pulse reliability**: mostly reliable but not 100%. Isolated,
  controlled tests (fresh `reboot`, nothing but repeated `ccatest -n 1`, no
  `tx`): **15/15** started immediately. Same loop with `tx -i 10` running
  throughout: **14/15** started (only the first call, taken right as `tx`
  began, missed). Earlier interactive-session runs saw 2–3 consecutive misses
  before recovering — not reproduced under controlled retest, so treat as an
  occasional (~5–10%) single-shot miss-fire, **not** something specifically
  caused by TX activity. **Practical implication: don't trust a single start
  blindly — verify `total` is actually advancing (not stuck at a stale value)
  before relying on a window's ratio.**

## Causal link to real channel traffic — confirmed

First attempt (self-TX via `tx -i 10`) was inconclusive — busy/total ratios
during TX (0.04%–0.87%) overlapped heavily with the no-TX baseline
(0.0%–0.77%), no clear step change. Root cause: `cmd_tx.c`'s `tx_task` only
polls on a **100ms** `vTaskDelay` loop, so `-i 10` (or any interval < 100ms)
actually fires at ~100ms regardless of the requested value — the intended TX
duty cycle (~1.8% of airtime at `-i 10`, easily distinguishable from ambient
noise) was actually ~0.18%, small enough to hide inside the ambient `busy`
variance seen even at idle. **Not fixed** (out of scope for this
investigation, flagged for whoever touches `cmd_tx.c` next).

**Resolved with a second real OBU2X unit as an external reference load
generator** (known transmitter, independent of this device's own radio and
of `tx_task`'s timing bug). Fresh `reboot`, 15x `ccatest -n 1 -i 1000` while
the load generator was actively transmitting:

```
CBR (raw, incl. duplicate frozen-window reads): mean 9.06%, min 0.00%, max 11.22%
CBR (10 independent windows after collapsing miss-fire duplicates):
  10.164, 11.109, 11.221, 10.174, 1.688, 10.205, 10.091, 10.072, 0.000, 10.162 (%)
  -> 8/10 cluster tightly at 10.07%-11.22% (mean 10.40%); 2 low outliers
     (1.688%, 0.000%) are full completed windows (flag=1), most likely
     windows that happened to align with a gap in the generator's own duty
     cycle rather than a measurement artifact.
```

**~10x jump vs. the 0.0%–0.9% idle baseline, cleanly separated from ambient
noise.** `busy` tracks real external channel occupancy, not just self-noise
— the CBR primitive works as intended.

**Confirmed it also drops back down**: same protocol (reboot, 15x
`ccatest -n 1 -i 1000`) immediately after stopping the load generator:

```
CBR (raw, incl. duplicates): mean 0.226%, min 0.000%, max 0.613%
CBR (10 independent windows): 0.313, 0.103, 0.197, 0.217, 0.613, 0.263,
  0.000, 0.360, 0.211, 0.039 (%) -> mean 0.232%
```

| state                        | CBR              |
|-------------------------------|------------------|
| idle (original baseline)      | 0.0%–0.9%        |
| load generator transmitting   | 10.1%–11.2% (mean 10.4%) |
| load generator stopped        | 0.0%–0.6% (mean 0.23%)   |

Busy tracks channel occupancy both up and back down, not a one-way artifact
— strong end-to-end confirmation the counters measure what they appear to.

Also re-confirmed the start-pulse miss-fire from above under real traffic:
3 of the 15 raw reads were exact-duplicate frozen values from a restart that
silently didn't take between separate console commands (`[3]`/`[4]`,
`[6]`/`[7]`, `[11]`–`[14]`) — consistent with the "occasional single-shot
miss-fire" already documented, not something new. Confirms the practical
guidance above: a real DCC polling loop must detect a stale/repeated
`total` and re-arm, not trust every read as a fresh window.

## Bugs found and fixed along the way (unrelated to CCA itself)

- **`cmd_tx.c`'s `SAMPLE_PACKET_HEX` was stale since commit `8c20558`
  "Integration with Apps"**: that commit widened `CommFrameHeader.channel`/
  `.rate` from `uint8_t` to `uint16_t` (14→16 byte struct) but didn't update
  the hand-spliced synthetic header in the TX sample blob, so `wifi_tx_frame()`
  rejected every `tx` attempt (`frame length mismatch: 34816 != 119`) — no
  frame ever went over the air. Fixed: resplice to the 16-byte layout
  (`channel=0`, `rate=0` → use current defaults, `frameLen=121`). Also fixed
  the matching stale "14-byte" description in `docs/design_notes.md`.
- **`cmd_tx.c`'s `tx_task` 100ms polling floor** (described above) — found,
  not fixed.

## Production code: background measurement task + console tools

`wifi_cca_start()` (`main/obu_wifi.c`, called from `app_main()`) runs a
continuous background task built on the confirmed mechanics above: arms a
~1s window, waits, reads it, discards anything that isn't a genuinely
completed window (`flag==true && total==ceiling`, and not an exact-duplicate
of the previous read — both signatures of the known start-pulse miss-fire),
clamps the `busy>total` race, and keeps a 5-sample ring buffer. Exposes
`wifi_get_cca_stats()`: `latest_percent` (raw, unfiltered — legitimately 0%
on a real empty window) and `avg_percent` (median-centered, drops samples
below 50% of the buffer's median — codifies the low-outlier pattern seen by
hand against the reference load generator: dropouts were always low, never
high spikes).

**Confirmed under continuous back-to-back polling**: the start-pulse
miss-fire rate is markedly higher here (~50%, vs. 0–20% in the
gapped/interactive manual tests) — re-arming immediately after a freeze
seems to miss-fire more than re-arming after a short pause. Harmless
(rejected samples are correctly excluded from `avg_percent`), just means
real-world throughput is closer to 1 valid window per ~2s than per ~1.1s.

Console tools (read-only, for bench use): `ccastats` (one-shot dump),
`dccmon -i <ms> / --stop` (periodic `fprintf` printer — note: uses `fprintf`,
not `ESP_LOGI`, because this project's `sdkconfig` sets
`CONFIG_LOG_DEFAULT_LEVEL=2`/WARN, which silently drops INFO logs).

## Wire protocol: COMM_FRAME_CMD_CBR_READ/START/STOP

Added for host-app consumption (`main/obu_comm.h`/`.c`), alongside the
existing `CommHeader`-framed protocol (`docs/design_notes.md`,
`wireshark_bridge/obu_mon.py`):

- `COMM_FRAME_CMD_CBR_READ` (50) → device replies immediately with one
  `COMM_FRAME_CBR_IND` (53) from the current `wifi_get_cca_stats()`.
- `COMM_FRAME_CMD_CBR_START` (51), payload `CbrStartPayload{intervalMs}`
  (1..`CBR_IND_MAX_INTERVAL_MS` = 1h, else `CMD_FAIL`) → starts a background task
  (`wifi_cbr_ind_start`, same idle-sentinel pattern as `dccmon_task_fn`) that
  pushes `CBR_IND` on that interval to whichever transport(s) are connected —
  mirrors the existing RX-frame dual-publish pattern in `wifi_task`.
- `COMM_FRAME_CMD_CBR_STOP` (52) → stops it.
- `CbrIndPayload`: `avgPercentMilli`/`latestPercentMilli` (percent×1000,
  network byte order — no floats on the wire, same convention as the FW
  update payloads) plus `validSamples`/`rejectedSamples`. `validSamples==0`
  means no window has been measured yet, not a real 0% reading.

New generic publish helpers added to support the proactive (unsolicited)
push, since it has no live request `ctx` to reply through:
`usb_stream_publish_frame()` (wraps the existing `usb_reply`),
`bt_stream_publish_frame()` (wraps the existing `bt_enqueue_frame`).

**Hardware-verified** (`/dev/ttyACM1`, the native USB-Serial-JTAG port —
distinct from the UART console port used for `ccatest`/`dccmon`, see
`docs/design_notes.md`'s two-USB-ports hardware note): `CBR_READ` returns a
correct all-zero `CBR_IND` right after boot; `CBR_START` at 1000ms produces
`CBR_IND` pushes on that cadence with the same idle-baseline values
(0.0%–0.2%) and the same duplicate/rejected-sample pattern already seen via
`dccmon`; `CBR_STOP` cleanly halts pushes (confirmed silent over a 4s
window). One easy-to-miss precondition, not a bug: the periodic push is
gated by `obu_comm_usb_connected()`/`_ble_connected()` — same guard as
existing RX-frame publishing — so a host must send `COMM_FRAME_CMD_START`
with payload `"USB"` (or `"BLE"`) before `CBR_START` will produce any visible
output.

## Reproducing

`ccatest [-w <cycles>] [-n <samples>] [-i <interval_ms>] [--no-start]` in
`main/obu_wifi.c` (registered in `main.c`). Each sample prints
`phy_get_cca_cnt`'s decoded `total`/`busy`/flag, the raw register reads
(independent cross-check), a canary-guarded overrun check on the output
buffer, and the delta busy/total ratio since the previous sample. Default
window is `40000000` cycles (~1s at ~39.8MHz, matching ETSI's default CBR
measurement interval).

**Warning printed by the command itself still applies**: these are
undocumented PHY registers: worst case is a hang requiring power-cycle, not
observed in any run so far.
