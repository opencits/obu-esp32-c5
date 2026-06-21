import argparse
import logging
import struct
import sys
import time
from collections import deque

import serial
import serial.tools.list_ports

HEADER_FMT = "<4sIIH"
HEADER_LEN = struct.calcsize(HEADER_FMT)  # 14

ESP_VID      = 0x303A         # Espressif Systems
FALLBACK_VIDS = {0x10C4, 0x1A86}  # CP210x, CH340

HDR_LENGTHS = (24, 26, 30, 32)

COMM_TX_SEQNO = 1

COMM_FRAME_PING  = 1
COMM_FRAME_PONG  = 2
COMM_FRAME_CMD_REBOOT  = 20
COMM_FRAME_CMD_CONFIG  = 21
COMM_FRAME_CMD_GOOD  = 28
COMM_FRAME_CMD_FAIL  = 29
COMM_FRAME_RX  = 30
COMM_FRAME_TX  = 31

# ---------------------------------------------------------------------------
# PCAP writer
# ---------------------------------------------------------------------------

class PcapWriter:
    """Minimal libpcap writer — DLT_IEEE802_11 (105), open in Wireshark directly."""
    _GLOBAL = struct.pack("<IHHiIII", 0xA1B2C3D4, 2, 4, 0, 0, 65535, 105)

    def __init__(self, path: str):
        self._f = open(path, "wb")
        self._f.write(self._GLOBAL)

    def write(self, sec: int, usec: int, data: bytes):
        n = len(data)
        self._f.write(struct.pack("<IIII", sec, usec, n, n))
        self._f.write(data)
        self._f.flush()

    def close(self):
        try: self._f.close()
        except Exception: pass


# ---------------------------------------------------------------------------
# Frame parser
# ---------------------------------------------------------------------------

class FrameReader:
    """Stateful resync parser: scans a byte stream for ITS5 frames."""
    def __init__(self):
        self._buf = bytearray()

    def feed(self, chunk: bytes):
        self._buf.extend(chunk)
        return True

# ---------------------------------------------------------------------------
# Serial helpers
# ---------------------------------------------------------------------------

def _find_esp_port() -> str | None:
    for p in serial.tools.list_ports.comports():
        if p.vid == ESP_VID:
            return p.device
    for p in serial.tools.list_ports.comports():
        if p.vid in FALLBACK_VIDS:
            return p.device
    return None


def _open_serial(port: str, baud: int) -> serial.Serial:
    ser = serial.Serial(port, baud, timeout=0.5)
    ser.dtr = False
    ser.rts = False
    time.sleep(0.05)
    return ser

def _send_serial(ser: serial.Serial, type: int, data: bytes):
    global COMM_TX_SEQNO
    length = len(data)
    header = struct.pack(">HHHH", 0x7384, COMM_TX_SEQNO , type, length)
    COMM_TX_SEQNO += 1
    ser.write(header)
    if length > 0:
        ser.write(data)

# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------
def _pause_exit(code: int) -> int:
    """On frozen exe, pause before exit so the user can read error messages."""
    if getattr(sys, "frozen", False):
        input("\nPress Enter to quit…")
    return code

def main():
    p = argparse.ArgumentParser(
        description="ESP32-C5 Wireshark bridge\n",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--port", default=None,
                   help="serial port (e.g. COM7); omit to auto-detect or use config")
    p.add_argument("--list-ports", action="store_true",
                   help="print available serial ports with VID/PID and exit")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--reset-on-start", action="store_true",
                   help="pulse RTS to reboot the device on first connect")
    p.add_argument("--reconnect-delay", type=float, default=3.0,
                   help="seconds to wait before reconnecting after USB disconnect (0 = exit)")
    p.add_argument("--pcap-out", default="",
                   help="write all captured frames to a .pcap file (DLT_IEEE802_11)")
    p.add_argument("--verbose", "-v", action="store_true")
    args = p.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )

    if args.list_ports:
        for pt in serial.tools.list_ports.comports():
            vid_pid = f" [{pt.vid:04X}:{pt.pid:04X}]" if pt.vid else ""
            print(f"{pt.device:<12}  {pt.description}{vid_pid}")
        return 0

    port = args.port

    if not port:
        port = _find_esp_port()
        if port:
            logging.info("Port auto-detected: %s", port)
        else:
            print("\n" + "=" * 60)
            print("  Error: No serial port found.")
            print("  Connect the ESP32 or pass --port.")
            print("=" * 60)
            return _pause_exit(1)

    logging.info("Port: %s", port)

    # ── Static PCAP output (--pcap-out, always-on) ───────────────────────────
    pcap: PcapWriter | None = None
    if args.pcap_out:
        pcap = PcapWriter(args.pcap_out)
        logging.info("PCAP output: %s", args.pcap_out)

    n_frames    = 0
    n_bytes     = 0
    start_time  = time.monotonic()
    last_stats  = start_time
    rate_window: deque = deque()
    first_connect = True
    ser: serial.Serial | None = None
    done = False

    try:
        while not done:
            try:
                logging.info("Opening %s @ %d baud", port, args.baud)
                ser = _open_serial(port, args.baud)
            except serial.SerialException as exc:
                logging.warning("Cannot open %s: %s", port, exc)
                if args.reconnect_delay <= 0:
                    break
                time.sleep(args.reconnect_delay)
                continue

            if args.reset_on_start and first_connect:
                logging.info("Pulsing RTS to reset device")
                ser.rts = True
                time.sleep(0.1)
                ser.rts = False
                time.sleep(0.05)
                ser.reset_input_buffer()
            first_connect = False
            reader = FrameReader()

            try:
                while not done:
                    chunk = ser.read(4096)
                    if chunk:
                        n_bytes += len(chunk)
                        if reader.feed(chunk):
                            n_frames += 1
                            now_mono = time.monotonic()
                            rate_window.append(now_mono)
                            while rate_window and rate_window[0] < now_mono - 60.0:
                                rate_window.popleft()

                    now_mono = time.monotonic()
                    if now_mono - last_stats >= 5.0:
                        rate   = len(rate_window)
                        uptime = int(now_mono - start_time)
                        logging.info("frames=%d  rate=%d/min  bytes=%d  uptime=%ds",
                                     n_frames, rate, n_bytes, uptime)
                        last_stats = now_mono
                        _send_serial(ser, COMM_FRAME_PING, b"")

            except serial.SerialException as exc:
                logging.warning("Serial error: %s", exc)
                try: ser.close()
                except Exception: pass
                ser = None
                if args.reconnect_delay <= 0:
                    break
                logging.info("Reconnecting in %.1f s…", args.reconnect_delay)  # noqa: RUF001
                time.sleep(args.reconnect_delay)

    except KeyboardInterrupt:
        logging.info("Stopping (frames=%d)", n_frames)
    finally:
        if ser is not None:
            try: ser.close()
            except Exception: pass
        if pcap:
            pcap.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
