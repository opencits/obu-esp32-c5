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

## Documentation

- **Build, flash, deploy**: `skills/build_deploy.md`
- **Design, architecture, implementation notes**: `docs/design_notes.md`
- **Agent/contributor hard rules**: `AGENTS.md`
