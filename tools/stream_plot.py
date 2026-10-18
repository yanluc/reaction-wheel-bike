#!/usr/bin/python3
"""Live plot of the bike's sample stream (preset Release-StreamTest) with XOR-FEC recovery and verification.

  ./stream_plot.py                 # window; press 'd' to toggle intentional packet drops
  ./stream_plot.py --headless 20   # no window, print statistics for 20 s
"""
import argparse
import collections
import glob
import random
import struct
import multiprocessing as mp
import queue
import time

import numpy as np
import serial

import bikelink as bl

from bikelink import (CMD_SET_TEST_DROPS, FEC_BLOCK, FEC_DEPTH, FEC_GROUP, SAMPLE_VALUES,
                      SAMPLES_PER_PACKET, STREAM_DATA, STREAM_HEADER, STREAM_PARITY)

RATE = 6700  # StreamTest::kSampleRateHz; overridden by --rate
NAMES = ["sine 1 Hz", "triangle 2 Hz", "square 0.5 Hz", "sawtooth 5 Hz"]


def expected(n: np.ndarray) -> np.ndarray:
    """Same formulas as StreamTest::makeSample (float32 like the MCU)."""
    f = np.float32
    ps = (n % RATE).astype(f) / f(RATE)
    pt = (n % (RATE // 2)).astype(f) / f(RATE // 2)
    pw = (n % (RATE // 5)).astype(f) / f(RATE // 5)
    out = np.empty((len(n), 4), dtype=np.int32)
    out[:, 0] = np.round(f(10000) * np.sin(f(6.28318530718) * ps))
    out[:, 1] = np.round(np.where(pt < f(0.5), f(-10000) + f(40000) * pt, f(30000) - f(40000) * pt))
    out[:, 2] = np.where((n % (RATE * 2)) < RATE, 8000, -8000)
    out[:, 3] = np.round(f(-10000) + f(20000) * pw)
    return out


class StreamDecoder:
    """Unwraps seq, rebuilds lost packets from interleaved XOR parity, emits ordered samples."""

    def __init__(self, on_samples):
        self.on_samples = on_samples
        self.reset()

    def reset(self):
        self.last_abs = None
        self.data = {}      # abs packet number -> payload (29 B)
        self.parity = {}    # abs first packet of group -> xor payload
        self.next_block = None
        self.stats = collections.Counter()

    def _unwrap(self, seq16):
        if self.last_abs is None:
            return seq16
        base = self.last_abs - (self.last_abs & 0xFFFF)
        cands = [base - 65536 + seq16, base + seq16, base + 65536 + seq16]
        return min(cands, key=lambda c: abs(c - self.last_abs))

    def feed(self, p: bytes):
        if p[0] not in (STREAM_DATA, STREAM_PARITY) or len(p) != 32:
            return
        seq16 = struct.unpack_from("<H", p, 1)[0]
        a = self._unwrap(seq16)
        if self.last_abs is not None and a < self.last_abs - 4 * FEC_BLOCK:
            self.stats["restarts"] += 1
            self.reset()
            a = seq16
        if p[0] == STREAM_DATA:
            self.data[a] = p[STREAM_HEADER:]
            self.stats["rx_data"] += 1
            self.last_abs = a if self.last_abs is None else max(self.last_abs, a)
        else:
            self.parity[a] = p[STREAM_HEADER:]
            self.stats["rx_parity"] += 1
        if self.next_block is None:
            self.next_block = a // FEC_BLOCK
        # A block is final once data two blocks later is arriving (parity comes at block end).
        while self.last_abs is not None and self.last_abs >= (self.next_block + 2) * FEC_BLOCK:
            self._finish(self.next_block)
            self.next_block += 1

    def _finish(self, block):
        first = block * FEC_BLOCK
        for g in range(FEC_DEPTH):
            members = [first + g + k * FEC_DEPTH for k in range(FEC_GROUP)]
            missing = [m for m in members if m not in self.data]
            if not missing:
                continue
            par = self.parity.get(first + g)
            if len(missing) == 1 and par is not None:
                acc = bytearray(par)
                for m in members:
                    if m != missing[0]:
                        for i, b in enumerate(self.data[m]):
                            acc[i] ^= b
                self.data[missing[0]] = bytes(acc)
                self.stats["recovered"] += 1
            else:
                self.stats["lost"] += len(missing)
        for pkt in range(first, first + FEC_BLOCK):
            payload = self.data.pop(pkt, None)
            n0 = pkt * SAMPLES_PER_PACKET
            if payload is None:
                self.on_samples(n0, None)
            else:
                n = SAMPLES_PER_PACKET * SAMPLE_VALUES
                vals = np.frombuffer(payload[:n * 2], dtype="<i2").reshape(SAMPLES_PER_PACKET, SAMPLE_VALUES)
                self.on_samples(n0, vals)
        for g in range(FEC_DEPTH):
            self.parity.pop(first + g, None)


def reader_process(port, drops, out_q, cmd_q, stop):
    ser = serial.Serial(port, 115200, timeout=0.01)
    ser.reset_input_buffer()
    chunks = []
    st = collections.Counter()

    def on_samples(n0, vals):
        if vals is None:
            chunks.append(np.full((SAMPLES_PER_PACKET, SAMPLE_VALUES), np.nan))
            st["gap_samples"] += SAMPLES_PER_PACKET
            return
        # firmware sample index wraps together with the 16-bit packet seq
        idx = np.arange(n0, n0 + SAMPLES_PER_PACKET, dtype=np.int64) % (65536 * SAMPLES_PER_PACKET)
        st["mismatch"] += int(np.count_nonzero(np.abs(vals.astype(np.int32) - expected(idx)) > 1))
        st["samples"] += SAMPLES_PER_PACKET
        chunks.append(vals.astype(float))

    dec = StreamDecoder(on_samples)
    fdec = bl.FrameDecoder()
    cmd_id = random.randint(1, 60000)  # the bike ignores a repeated id

    def send_drops(on):
        nonlocal cmd_id
        cmd_id = cmd_id % 0xFFFF + 1
        ser.write(bl.encode_frame(bl.command(cmd_id, CMD_SET_TEST_DROPS, 0, 0, 1 if on else 0)))
        st["drops_on"] = int(on)

    send_drops(drops)
    next_out = time.monotonic()
    while not stop.is_set():
        for p in fdec.feed(ser.read(65536)):
            kind = p[0]
            if kind in (STREAM_DATA, STREAM_PARITY):
                dec.feed(p)
            elif kind == bl.TELEMETRY:
                st["tlm"] += 1
            elif kind == bl.BASE_STATUS:
                b = bl.BaseStatus.parse(p)
                st["base_rx_total"] = b.rx_packets
                st["base_drop"] = b.rx_dropped
            elif kind == bl.LOG:
                print(f"[log] {p[1:].decode(errors='replace')}", flush=True)
        try:
            while True:
                send_drops(cmd_q.get_nowait())
        except queue.Empty:
            pass
        now = time.monotonic()
        if now >= next_out:
            next_out = now + 0.05
            stats = dict(st)
            stats.update({k: dec.stats.get(k, 0) for k in ("rx_data", "rx_parity", "recovered", "lost")})
            block = np.vstack(chunks) if chunks else np.empty((0, SAMPLE_VALUES))
            chunks.clear()
            try:
                out_q.put_nowait((block, stats))
            except queue.Full:
                pass  # the GUI is behind: skip this chunk for display, stats come with the next one


def format_stats(cur, prev, dt):
    d = {k: cur.get(k, 0) - prev.get(k, 0) for k in ("samples", "rx_data", "rx_parity", "tlm", "base_rx_total")}
    return (f"samples {d['samples'] / dt:6.0f}/s | packets data {d['rx_data'] / dt:5.0f}/s parity "
            f"{d['rx_parity'] / dt:4.0f}/s | FEC recovered {cur.get('recovered', 0)} lost {cur.get('lost', 0)} "
            f"(gap samples {cur.get('gap_samples', 0)}) | bad samples {cur.get('mismatch', 0)} | telemetry "
            f"{d['tlm'] / dt:.0f}/s | base rx {d['base_rx_total'] / dt:.0f}/s drop {cur.get('base_drop', 0)}"
            f" | drops {'ON' if cur.get('drops_on') else 'off'}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port")
    ap.add_argument("--window", type=float, default=2.0, help="seconds shown")
    ap.add_argument("--headless", type=float, default=0.0, help="no plot, print stats for N seconds")
    ap.add_argument("--drops", action="store_true", help="enable intentional drops at start")
    ap.add_argument("--points", type=int, default=2000, help="max points drawn per signal")
    ap.add_argument("--rate", type=int, default=RATE, help="sample rate of the firmware (BIKE_STREAM_RATE_HZ)")
    args = ap.parse_args()
    globals()["RATE"] = args.rate  # used by expected(); set before the reader process forks

    port = args.port or sorted(glob.glob("/dev/serial/by-id/usb-Raspberry_Pi_*"))[0]
    out_q, cmd_q, stop = mp.Queue(maxsize=100), mp.Queue(), mp.Event()
    # separate process: a GUI stall must not stall the USB reader (the base would drop data)
    proc = mp.Process(target=reader_process, args=(port, args.drops, out_q, cmd_q, stop), daemon=True)
    proc.start()

    if args.headless:
        prev, t_prev, cur = {}, time.monotonic(), {}
        t_end = t_prev + args.headless
        while time.monotonic() < t_end:
            try:
                _, cur = out_q.get(timeout=0.2)
            except queue.Empty:
                pass
            now = time.monotonic()
            if now - t_prev >= 1.0 and cur:
                print(format_stats(cur, prev, now - t_prev), flush=True)
                prev, t_prev = cur, now
        stop.set()
        proc.join(timeout=2)
        return

    import matplotlib
    for backend in ("QtAgg", "Qt5Agg", "TkAgg"):  # first GUI backend that is installed
        try:
            matplotlib.use(backend, force=True)
            import matplotlib.pyplot as plt
            plt.figure().clf()
            plt.close("all")
            break
        except (ImportError, RuntimeError):
            continue
    else:
        raise SystemExit("no GUI backend for matplotlib (install python3-pyqt5 or python3-tk)")
    from matplotlib.animation import FuncAnimation

    n_show = int(args.window * RATE)
    step = max(1, -(-n_show // args.points))  # decimation for drawing only
    ring = np.full((n_show, SAMPLE_VALUES), np.nan)
    t_axis = np.arange(-n_show, 0)[::step] / RATE

    fig, axes = plt.subplots(SAMPLE_VALUES, 1, sharex=True, figsize=(11, 8))
    lines = []
    for ax, name in zip(axes, NAMES):
        (ln,) = ax.plot(t_axis, np.full(len(t_axis), np.nan), lw=0.8)
        ax.set_ylim(-11000, 11000)
        ax.set_xlim(-args.window, 0)
        ax.set_ylabel(name, fontsize=8)
        ax.grid(alpha=0.3)
        lines.append(ln)
    axes[-1].set_xlabel("time [s] (0 = newest sample)")
    title = fig.suptitle("waiting for data...", fontsize=9)
    st = {"prev": {}, "cur": {}, "t": time.monotonic(), "drops": args.drops}

    def on_key(ev):
        if ev.key == "d":
            st["drops"] = not st["drops"]
            cmd_q.put(st["drops"])

    fig.canvas.mpl_connect("key_press_event", on_key)

    def update(_):
        nonlocal ring
        blocks = []
        try:
            while True:
                block, st["cur"] = out_q.get_nowait()
                blocks.append(block)
        except queue.Empty:
            pass
        if blocks:
            new = np.vstack(blocks)[-n_show:]
            ring = np.vstack((ring[len(new):], new))
            shown = ring[::step]
            for i, ln in enumerate(lines):
                ln.set_ydata(shown[:, i])
        now = time.monotonic()
        if now - st["t"] >= 1.0 and st["cur"]:
            line = format_stats(st["cur"], st["prev"], now - st["t"])
            st["prev"], st["t"] = st["cur"], now
            print(line, flush=True)
            title.set_text(line.replace(" | ", "\n", 2) + "\n'd' = toggle intentional drops")
        return lines

    _anim = FuncAnimation(fig, update, interval=100, blit=False, cache_frame_data=False)  # keep a reference
    plt.tight_layout(rect=(0, 0, 1, 0.9))
    plt.show()
    stop.set()
    proc.join(timeout=2)


if __name__ == "__main__":
    main()
