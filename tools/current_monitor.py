#!/usr/bin/python3
"""Live bike current [mA] from telemetry, plotted with 1 s mean/std. Keys: z = tare, c = clear tare.

  ./current_monitor.py
  ./current_monitor.py --headless 10
"""
import argparse
import collections
import glob
import struct
import time

import numpy as np
import serial

import bikelink as bl


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--window", type=float, default=10.0, help="seconds shown")
    ap.add_argument("--headless", type=float, default=0.0, help="no plot, print 1 s stats for N seconds")
    args = ap.parse_args()

    port = args.port or sorted(glob.glob("/dev/serial/by-id/usb-Raspberry_Pi_*"))[0]
    ser = serial.Serial(port, args.baud, timeout=0)
    ser.reset_input_buffer()
    dec = bl.FrameDecoder()
    hist = collections.deque()  # (time, mA)
    state = {"tare": 0.0}

    def pump():
        now = time.monotonic()
        for p in dec.feed(ser.read(65536)):
            if p[0] == bl.TELEMETRY and len(p) == struct.calcsize(bl.TELEMETRY_FMT):
                hist.append((now, bl.Telemetry.parse(p).current * 1000.0))
            elif p[0] == bl.LOG:
                print(f"[log] {p[1:].decode(errors='replace')}", flush=True)
        while hist and hist[0][0] < now - args.window:
            hist.popleft()

    def last_second():
        now = time.monotonic()
        vals = np.array([v for t, v in hist if t >= now - 1.0]) - state["tare"]
        return vals

    def summary():
        v = last_second()
        if len(v) == 0:
            return "no telemetry (bike on, HW firmware, base connected?)"
        return (f"I = {v[-1]:8.0f} mA | 1 s mean {v.mean():8.1f} mA  std {v.std():6.1f} mA  "
                f"min {v.min():.0f} max {v.max():.0f} | {len(v)} samples/s | tare {state['tare']:.0f} mA")

    if args.headless:
        t_end = time.monotonic() + args.headless
        next_print = time.monotonic() + 1.0
        while time.monotonic() < t_end:
            pump()
            if time.monotonic() >= next_print:
                next_print += 1.0
                print(summary(), flush=True)
            time.sleep(0.01)
        return

    import matplotlib
    for backend in ("QtAgg", "Qt5Agg", "TkAgg"):
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

    fig, ax = plt.subplots(figsize=(10, 5))
    (line,) = ax.plot([], [], lw=1)
    ax.set_xlim(-args.window, 0)
    ax.set_xlabel("time [s] (0 = now)")
    ax.set_ylabel("current [mA]")
    ax.grid(alpha=0.3)
    title = fig.suptitle("waiting for telemetry...", fontsize=11)

    def on_key(ev):
        if ev.key == "z":
            v = last_second() + state["tare"]
            if len(v):
                state["tare"] = float(v.mean())
        elif ev.key == "c":
            state["tare"] = 0.0

    fig.canvas.mpl_connect("key_press_event", on_key)

    def update(_):
        pump()
        if hist:
            now = time.monotonic()
            t = np.array([h[0] for h in hist]) - now
            v = np.array([h[1] for h in hist]) - state["tare"]
            line.set_data(t, v)
            lo, hi = v.min(), v.max()
            pad = max(50.0, 0.1 * (hi - lo))
            ax.set_ylim(lo - pad, hi + pad)
        title.set_text(summary() + "\nz = tare, c = clear tare")
        return (line,)

    _anim = FuncAnimation(fig, update, interval=100, blit=False, cache_frame_data=False)  # keep a reference
    plt.tight_layout(rect=(0, 0, 1, 0.92))
    plt.show()


if __name__ == "__main__":
    main()
