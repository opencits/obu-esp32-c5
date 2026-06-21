# ITS-G5 Receiver Firmware with TX Enabled

ESP32-C5 firmware that sniffs 802.11p (ITS-G5) frames and forwards them over BLE, USB, and/or pcap. Management connectivity (WiFi/Ethernet, OTA, config) is separate from the capture path.

## WARNING

**The frequency band from 5,875 to 5,935 MHz is used for safety-related applications of Intelligent Transport Systems in the EU. Please make sure to obey all applicable laws and regulations when transmitting in this band.**

## Hardware

Recommended: our custom hardware, see [its-g5-receiver](https://codeberg.org/opentrafficmap/its-g5-receiver). An ESP32-C5-WIFI6-KIT devboard also works — see `docs/design_notes.md` for wiring.

## Documentation

- **Build, flash, deploy**: `skills/build_deploy.md`
- **Design, architecture, implementation notes**: `docs/design_notes.md`
- **Agent/contributor hard rules**: `AGENTS.md`
