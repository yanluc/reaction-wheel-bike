#!/usr/bin/python3
"""Radio throughput benchmark (bike preset Release-Bench); --bike-port separates radio from USB loss.

  ./radio_bench.py --duration 30 --bike-port /dev/serial/by-id/usb-STMicroelectronics_*-if02
"""
import argparse
import glob
import struct
import threading
import time

import serial

import bikelink as bl

BENCH = bl.BENCH
BENCH_FMT = "<BBII22s"
MODES = {0: "ACK", 1: "STREAM", 2: "ACK-PIPE"}


def bike_log_reader(port, stop):
    ser = serial.Serial(port, 2000000, timeout=0.05)
    dec = bl.FrameDecoder()
    while not stop.is_set():
        for p in dec.feed(ser.read(4096)):
            if p[0] == bl.LOG:
                print(f"    [bike] {p[1:].decode(errors='replace')}", flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=None)
    ap.add_argument("--bike-port", default=None)
    ap.add_argument("--duration", type=float, default=20.0)
    args = ap.parse_args()

    port = args.port or sorted(glob.glob("/dev/serial/by-id/usb-Raspberry_Pi_*"))[0]
    ser = serial.Serial(port, 115200, timeout=0.01)
    ser.reset_input_buffer()
    stop = threading.Event()
    if args.bike_port:
        threading.Thread(target=bike_log_reader, args=(args.bike_port, stop), daemon=True).start()

    dec = bl.FrameDecoder()
    t0 = time.monotonic()
    next_report = t0 + 1.0
    count = {0: 0, 1: 0, 2: 0}
    lost = {0: 0, 1: 0, 2: 0}
    last_seq = {}
    usb_bytes = 0
    base_rx_prev = None
    base_rx_rate = None
    base_dropped = 0

    try:
        while time.monotonic() - t0 < args.duration:
            data = ser.read(65536)
            usb_bytes += len(data)
            for p in dec.feed(data):
                if p[0] == BENCH and len(p) == 32:
                    _, mode, seq, _, _ = struct.unpack(BENCH_FMT, p)
                    count[mode] = count.get(mode, 0) + 1
                    prev = last_seq.get(mode)
                    if prev is not None and seq > prev:
                        lost[mode] += seq - prev - 1
                    last_seq[mode] = seq  # seq restarts each phase -> a drop is a new phase, not loss
                elif p[0] == bl.BASE_STATUS:
                    st = bl.BaseStatus.parse(p)
                    if base_rx_prev is not None:
                        base_rx_rate = st.rx_packets - base_rx_prev
                    base_rx_prev = st.rx_packets
                    base_dropped = st.rx_dropped
            now = time.monotonic()
            if now >= next_report:
                next_report += 1.0
                parts = []
                for m in (0, 1, 2):
                    if count[m]:
                        n = count[m]
                        loss = 100.0 * lost[m] / (n + lost[m])
                        parts.append(f"{MODES[m]}: {n} pkt/s = {n * 32 / 1000:.1f} kB/s payload, lost {lost[m]} ({loss:.2f}%)")
                line = " | ".join(parts) or "no bench packets"
                line += f" | usb {usb_bytes / 1000:.0f} kB/s"
                if base_rx_rate is not None:
                    line += f" | base rx {base_rx_rate}/s dropped(total) {base_dropped}"
                if dec.crc_errors:
                    line += f" | crc_err {dec.crc_errors}"
                print(line, flush=True)
                count = {0: 0, 1: 0, 2: 0}
                lost = {0: 0, 1: 0, 2: 0}
                usb_bytes = 0
    except KeyboardInterrupt:
        pass
    stop.set()


if __name__ == "__main__":
    main()
