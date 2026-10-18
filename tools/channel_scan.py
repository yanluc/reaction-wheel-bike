#!/usr/bin/python3
"""2.4 GHz channel occupancy scan with the base station's nRF24 (RPD > -64 dBm); pauses the bike link.

  ./channel_scan.py --sweeps 200
"""
import argparse
import glob
import struct
import time

import serial

import bikelink as bl

LEGAL_MAX_CH = 83      # 2483.5 MHz ISM upper edge
RATE_HALF_WIDTH = 1    # 2 Mbps occupies ~2 MHz -> also look at neighbours


def wifi_channels(ch):
    """Wi-Fi 2.4 GHz channels (20 MHz wide) overlapping nRF24 channel ch."""
    mhz = 2400 + ch
    return [n for n in range(1, 14) if abs(mhz - (2407 + 5 * n)) <= 10]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=None)
    ap.add_argument("--sweeps", type=int, default=200, help="1..255")
    ap.add_argument("--current", type=int, default=76, help="channel the bike link uses (marked)")
    args = ap.parse_args()

    port = args.port or sorted(glob.glob("/dev/serial/by-id/usb-Raspberry_Pi_*"))[0]
    ser = serial.Serial(port, 115200, timeout=0.05)
    ser.reset_input_buffer()
    sweeps = max(1, min(255, args.sweeps))
    ser.write(bl.encode_frame(struct.pack(bl.BASE_COMMAND_FMT, bl.BASE_COMMAND, bl.BASE_CMD_SCAN, sweeps)))
    print(f"scanning 126 channels x {sweeps} sweeps (~{126 * 0.21e-3 * sweeps:.1f} s)...")

    dec = bl.FrameDecoder()
    hits = {}
    deadline = time.monotonic() + 126 * 0.3e-3 * sweeps + 5
    while len(hits) < 126 and time.monotonic() < deadline:
        for p in dec.feed(ser.read(4096)):
            if p[0] == bl.SCAN_RESULT and len(p) == struct.calcsize(bl.SCAN_RESULT_FMT):
                _, first, count, n, data = struct.unpack(bl.SCAN_RESULT_FMT, p)
                for i in range(count):
                    hits[first + i] = data[i] / n
    if len(hits) < 126:
        raise SystemExit("no scan result (is the base flashed with scan support and its radio OK?)")

    print(f"\n ch   MHz  busy%  {'':40s} Wi-Fi overlap")
    for ch in range(126):
        pct = 100 * hits[ch]
        bar = "#" * min(40, round(pct * 40 / 100 * 4))  # x4 scale: low occupancy still visible
        notes = []
        if ch == args.current:
            notes.append("<- current link (bike traffic shows here)")
        if ch > LEGAL_MAX_CH:
            notes.append("outside ISM band, do not use")
        wifi = wifi_channels(ch)
        print(f"{ch:3d}  {2400 + ch:4d}  {pct:5.1f}  {bar:40s} {','.join(map(str, wifi)) or '-':8s} {' '.join(notes)}")

    def window_score(ch):
        return sum(hits[c] for c in range(ch - RATE_HALF_WIDTH, ch + RATE_HALF_WIDTH + 1))

    candidates = [c for c in range(1 + RATE_HALF_WIDTH, LEGAL_MAX_CH - RATE_HALF_WIDTH + 1) if c != args.current]
    best = sorted(candidates, key=lambda c: (window_score(c), len(wifi_channels(c))))[:5]
    print("\nquietest channels for 2 Mbps (sum of busy% over ch-1..ch+1):")
    for c in best:
        print(f"  ch {c:3d} = {2400 + c} MHz  score {100 * window_score(c):.1f}%  Wi-Fi overlap: {wifi_channels(c) or 'none'}")
    print(f"current ch {args.current}: score {100 * window_score(args.current):.1f}% (includes our own bike packets)")


if __name__ == "__main__":
    main()
