# ITS-G5 Receiver Firmware with TX Enabled

ESP32-C5 firmware that sniffs 802.11p (ITS-G5) frames and forwards them over BLE or USB.

A client can also transmit preformated (802.11 Header and up) frames.

Intended to be used by e.g. an Android based µOBU - Android code is not included here.

Forked and based on previous work from https://github.com/pit711/V2X2MAP and https://codeberg.org/opentrafficmap

## WARNING

**The frequency band from 5,875 to 5,935 MHz (ITS-G5) is used for safety-related applications of Intelligent Transport Systems in the EU. 
Please make sure to obey all applicable laws and regulations when transmitting in this band.**

## Hardware

Based for an ESP32-C5-WIFI6-KIT devboard.

## Building with CLion

CLion can build the firmware with plain CMake. You don't need `idf.py`, but ESP-IDF must already be installed
(`esp-idf/install.sh`, see `skills/build_deploy.md`). Both files below are machine-local and gitignored.

1. Create `.clion-idf-env.sh` in the repo root. Use the absolute path, because CLion may source this file from
   any directory, and from plain `sh`:

   ```sh
   export IDF_PATH=/absolute/path/to/c2x-obu-esp32-c5/esp-idf
   export IDF_PATH_FORCE=1
   . "$IDF_PATH/export.sh" >/dev/null
   ```

2. Create `CMakeUserPresets.json` in the repo root:

   ```json
   {
     "version": 3,
     "configurePresets": [{
       "name": "esp32c5",
       "displayName": "ESP-IDF esp32c5",
       "generator": "Ninja",
       "binaryDir": "${sourceDir}/cmake-build-esp32c5",
       "cacheVariables": { "IDF_TARGET": "esp32c5", "CCACHE_ENABLE": "0" },
       "vendor": { "jetbrains.com/clion": { "toolchain": "ESP-IDF" } }
     }],
     "buildPresets": [{ "name": "esp32c5", "configurePreset": "esp32c5" }]
   }
   ```

3. In CLion, open Settings → Build, Execution, Deployment → Toolchains and click **+** → *System*.
   - Name the toolchain exactly `ESP-IDF`, the name the preset refers to.
   - Click **Add environment** → **From file** and select `.clion-idf-env.sh`.
   - Leave the compilers on auto-detect; ESP-IDF's CMake files select the RISC-V compiler.
4. In Settings → … → CMake, enable the **ESP-IDF esp32c5** profile. Disable any default profile that uses the
   host compiler. Then reload the CMake project and build (Ctrl+F9).

CLion builds into `cmake-build-esp32c5/`. Note: once `CMakeUserPresets.json` exists, `idf.py build` (ESP-IDF 6.x)
picks up its first preset too and also builds into `cmake-build-esp32c5/` — `build/` then goes stale, so flash
images from `cmake-build-esp32c5/`. Both use the same `sdkconfig` in the repo root. ESP-IDF's `flash`, `app-flash`,
`size`, `menuconfig` and `monitor` targets appear in CLion's target list. `flash` uses the `ESPPORT`
environment variable for the serial port if it's set. `monitor` and `menuconfig` need an interactive terminal,
so run `idf.py monitor` or `idf.py menuconfig` in CLion's terminal instead.

## Host test tool (`tools/esp32-test`)

Command-line tool for quick evaluation over the native USB port, using the binary wire protocol (`main/obu_comm.h`).
Needs Python 3.8+ and `pyserial` (`pip install pyserial`). Without arguments it prints usage.

```
tools/esp32-test --status                    # FW version, rx/tx counters, config, CBR
tools/esp32-test --rx --cbr                  # RX fps / B/s + CBR %, one line, every 2 s, Ctrl-C stops
tools/esp32-test --chan 5880                 # store autostart channel (5860..5920) and reboot the device
tools/esp32-test --tx --tx-interval 10 --tx-length 300 [--tx-burst 8] [--tx-power 20] [--tx-datarate 6000]
tools/esp32-test --flash cmake-build-esp32c5/obu-esp32-c5.bin   # FW update via FW_BEGIN/DATA/END, reboot, confirm
tools/esp32-test --list-ports                # port is auto-detected (VID 0x303A), override with --port
```

- `--rx` and `--tx` can be combined on one board (one status line). A board does not report its own TX frames as
  RX — use a second board (second tool instance with its own `--port`) to verify TX over the air.
- `--chan` reboots the device: the channel is only applied at boot (no live-tune command).
- `--tx` transmits on the ITS-G5 band — see WARNING above. Ctrl-C stops the TX loop.
- Use the board's **"USB"** port (native USB-Serial-JTAG). The "UART" port is the text console, not this protocol.

### Windows 11

Not tested on Windows yet; the notes below are expected behaviour.

- **No driver needed.** The USB-Serial-JTAG port is a composite device; its CDC-ACM interface binds to the inbox
  `usbser.sys` driver and shows up as "USB Serial Device (COMx)". Only the JTAG interface would need a WinUSB driver
  (ESP-IDF installer), and only for OpenOCD debugging. The "UART" port (CH9102) may need the WCH driver (often
  installed via Windows Update) — not needed for this tool.
- **Install:** Python 3.8+ (python.org or Microsoft Store), then `py -m pip install pyserial`. Copy
  `tools/esp32-test`; it has no `.py` extension, so run it as `py esp32-test --status` (or rename to `.py`).
  Without Python on the target: `pyinstaller --onefile esp32-test` builds a standalone `.exe`.
- **COM ports are exclusive** — close `idf.py monitor`, serial terminals or bridges first, otherwise "access denied".
- Port auto-detect works (pyserial reports VID 0x303A); manual override `--port COM7`.
- `--chan`/`--flash` reboot the device; Windows normally keeps the same COM number (same USB serial number), and
  the tool retries the reopen for 15 s.
- The single-line `\r` status output and Ctrl-C work in Windows Terminal and cmd.

## Documentation

- **Build, flash, deploy**: `skills/build_deploy.md`
- **Host test tool**: `tools/esp32-test` (above)
- **Design, architecture, implementation notes**: `docs/design_notes.md`
- **Agent/contributor hard rules**: `AGENTS.md`
