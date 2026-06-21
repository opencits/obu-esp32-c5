# Build and Deploy skill

ESP-IDF project (target `esp32c5`), esp-idf vendored as git submodule at `esp-idf/`. Design/architecture: `docs/design_notes.md`.

## Clone

```
git clone https://github.com/opencits/obu-esp32-c5.git
cd obu-esp32-c5
git submodule update --init --recursive
```

## One-time toolchain install

```
esp-idf/install.sh
```

## Per-shell environment

```
. esp-idf/export.sh
```

## Hardware setup (devboard only — skip for custom hardware)

The committed `sdkconfig` is currently tuned for the **Waveshare ESP32-C5-WIFI6-KIT** (WiFi/BLE only, no Ethernet, onboard LED on GPIO27) — see `docs/design_notes.md` (Waveshare ESP32-C5-WIFI6-KIT specifics) for what's been changed and why. There is **no working `sdkconfig.proto-w5500`/`sdkconfig.proto-enc28j60` copy-over path** — those files don't exist in this repo despite being referenced historically; if you need devboard Ethernet, you'll need to create one.
If you deviate from a known board/sdkconfig, set `HW_VARIANT=custom` (`idf.py menuconfig`) or you may receive incompatible OTA images.

After editing `sdkconfig` by hand, run `idf.py reconfigure` (not just `idf.py build`) — this lets esp-idf's `kconfgen` regenerate dependent symbols correctly. Hand-editing alone can leave a `bool`/`menuconfig` toggle flipped but its `select`-driven dependents stale.

## Build

```
idf.py build
```

Board options (HW_VARIANT, SD pins, sniffer queue/task sizing, LED pin, temp sensor) live in `main/Kconfig.projbuild`, set via `idf.py menuconfig`.

## Flash + monitor

```
idf.py -p /dev/ttyACM0 -b 921600 flash monitor -b 115200
```
Replace `/dev/ttyACM0` with the actual serial device.

## Flash a pre-built release (no esp-idf build needed)

Still requires `esptool` from the esp-idf environment (`. esp-idf/export.sh` first).
```
esptool --chip esp32c5 -p /dev/ttyACM0 -b 921600 --before=default-reset --after=hard-reset write-flash \
  --flash-mode dio --flash-freq 80m --flash-size 8MB \
  0x2000 bootloader_<version>.bin \
  0x8000 partition-table_8M_<version>.bin \
  0x20000 its-g5-receiver-firmware_<variant>_<version>.bin
```

## Partition layout (`partitions_8M.csv`)

Dual OTA app slots (`ota_0`/`ota_1`, 0x370000 each) for `idf.py app-flash`/OTA updates, plus a `storage` FAT region (console command history only — SD card, if present, is a separate device at `/sdcard`, not this partition).

## CLion

See `.clion-idf-env.sh` (gitignored, machine-local) for the CLion toolchain "Environment file" — `esp-idf/export.sh` alone does not reliably propagate `PATH` (toolchain bin dirs) into CLion's CMake invocation; the wrapper hardcodes the resolved values instead. Regenerate its contents by running `. esp-idf/export.sh` in a terminal and copying the new `export` lines if you upgrade the ESP-IDF toolchain version.
