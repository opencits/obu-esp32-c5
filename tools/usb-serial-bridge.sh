#!/usr/bin/env bash
#
# Bridge a host USB serial device (e.g. ESP32 JTAG/serial) to a TCP port.
#
# Workaround for `emulator -usb-passthrough`, which only works on custom-built
# Android Automotive (AAOS) system images -- it silently does nothing on a
# standard phone/google_apis AVD (no error, device just never appears in the
# guest). Bridging over TCP instead lets code running in the emulator reach
# the serial device via the guest's host alias, 10.0.2.2.
#
# Usage:
#   tools/usb-serial-bridge.sh start  [--port 7777] [--vendor 0x303a] [--product 0x1001] [--baud 115200] [--foreground]
#   tools/usb-serial-bridge.sh stop   [--port 7777]
#   tools/usb-serial-bridge.sh status [--port 7777]
#
# From inside the AVD, connect to 10.0.2.2:<port> (plain TCP, raw bytes to/from the serial device).
#
set -euo pipefail

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
NC='\033[0m'

PORT=7777
VENDOR=0x303a
PRODUCT=0x1001
BAUD=115200
FOREGROUND=0

cmd="${1:-}"
shift || true

while [ $# -gt 0 ]; do
    case "$1" in
        --port) PORT="$2"; shift 2 ;;
        --vendor) VENDOR="$2"; shift 2 ;;
        --product) PRODUCT="$2"; shift 2 ;;
        --baud) BAUD="$2"; shift 2 ;;
        --foreground) FOREGROUND=1; shift ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

PIDFILE="/tmp/usb-serial-bridge-${PORT}.pid"
LOGFILE="/tmp/usb-serial-bridge-${PORT}.log"

find_device() {
    local vid="${VENDOR#0x}"
    local pid="${PRODUCT#0x}"
    local sys busid iface tnode
    for sys in /sys/bus/usb/devices/*; do
        [ -f "$sys/idVendor" ] || continue
        [ "$(cat "$sys/idVendor" 2>/dev/null)" = "$vid" ] || continue
        [ "$(cat "$sys/idProduct" 2>/dev/null)" = "$pid" ] || continue
        busid=$(basename "$sys")
        for iface in "$sys/$busid":*; do
            [ -d "$iface/tty" ] || continue
            for tnode in "$iface"/tty/*; do
                [ -e "$tnode" ] || continue
                echo "/dev/$(basename "$tnode")"
                return 0
            done
        done
    done
    return 1
}

require_socat() {
    if ! command -v socat > /dev/null 2>&1; then
        echo -e "${RED}Error: socat not installed.${NC} Install it (e.g. 'sudo apt install socat') and retry."
        exit 1
    fi
}

do_status() {
    if [ -f "$PIDFILE" ] && kill -0 "$(cat "$PIDFILE")" 2>/dev/null; then
        echo -e "${GREEN}running${NC}  pid=$(cat "$PIDFILE")  port=$PORT  log=$LOGFILE"
    else
        echo -e "${YELLOW}not running${NC}  port=$PORT"
        [ -f "$PIDFILE" ] && rm -f "$PIDFILE"
    fi
}

do_stop() {
    if [ -f "$PIDFILE" ] && kill -0 "$(cat "$PIDFILE")" 2>/dev/null; then
        kill "$(cat "$PIDFILE")"
        rm -f "$PIDFILE"
        echo -e "${GREEN}Stopped${NC} bridge on port $PORT."
    else
        echo -e "${YELLOW}No bridge running on port $PORT.${NC}"
        rm -f "$PIDFILE"
    fi
}

do_start() {
    require_socat

    if [ -f "$PIDFILE" ] && kill -0 "$(cat "$PIDFILE")" 2>/dev/null; then
        echo -e "${YELLOW}Already running${NC} (pid=$(cat "$PIDFILE")) on port $PORT."
        exit 0
    fi

    local dev
    if ! dev=$(find_device); then
        echo -e "${RED}Error: no USB device ${VENDOR}:${PRODUCT} found on the host.${NC}"
        echo "Check 'lsusb' and pass --vendor/--product if it differs."
        exit 1
    fi

    if [ ! -w "$dev" ]; then
        echo -e "${RED}Error: $dev is not writable by $(whoami).${NC}"
        echo "Add a udev rule, e.g.:"
        echo "  SUBSYSTEM==\"usb\", ATTR{idVendor}==\"${VENDOR#0x}\", ATTR{idProduct}==\"${PRODUCT#0x}\", MODE=\"0666\", TAG+=\"uaccess\""
        echo "in /etc/udev/rules.d/, then 'sudo udevadm control --reload-rules && sudo udevadm trigger' and replug."
        exit 1
    fi

    echo "Bridging $dev (baud $BAUD) <-> TCP :$PORT ..."
    echo "From inside the AVD, connect to 10.0.2.2:$PORT"

    local socat_cmd=(socat -d -d "TCP-LISTEN:${PORT},reuseaddr,fork" "${dev},b${BAUD},raw,echo=0")

    if [ "$FOREGROUND" = "1" ]; then
        exec "${socat_cmd[@]}"
    fi

    nohup "${socat_cmd[@]}" > "$LOGFILE" 2>&1 &
    echo $! > "$PIDFILE"
    sleep 0.3
    if kill -0 "$(cat "$PIDFILE")" 2>/dev/null; then
        echo -e "${GREEN}Started${NC} (pid=$(cat "$PIDFILE")). Log: $LOGFILE"
    else
        echo -e "${RED}Failed to start.${NC} Check $LOGFILE"
        rm -f "$PIDFILE"
        exit 1
    fi
}

case "$cmd" in
    start) do_start ;;
    stop) do_stop ;;
    status) do_status ;;
    *)
        echo "Usage: $0 {start|stop|status} [--port 7777] [--vendor 0x303a] [--product 0x1001] [--baud 115200] [--foreground]"
        exit 1
        ;;
esac
