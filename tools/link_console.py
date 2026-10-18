#!/usr/bin/python3
"""Bike link console (radio base or STM32 cable): logs, 1 s stats, ping RTT, one-shot commands.

  ./link_console.py --cmd arm
  ./link_console.py --port /dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_*-if02 --baud 2000000
"""
import argparse
import glob
import random
import struct
import sys
import time

import serial

import bikelink as bl

COMMANDS = {"arm": bl.CMD_ARM, "disarm": bl.CMD_DISARM, "ping": bl.CMD_PING}


def find_base_port():
    ports = sorted(glob.glob("/dev/serial/by-id/usb-Raspberry_Pi_*"))
    return ports[0] if ports else None


def flags_str(f):
    names = [("ARMED", bl.FLAG_ARMED), ("IMU", bl.FLAG_IMU_OK), ("LINK", bl.FLAG_RADIO_LINK),
             ("SIM", bl.FLAG_SIM), ("FAILSAFE", bl.FLAG_FAILSAFE)]
    return "|".join(n for n, m in names if f & m) or "-"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="default: RP2040 base station from /dev/serial/by-id")
    ap.add_argument("--baud", type=int, default=115200, help="ignored by USB CDC; 2000000 for the STM32 VCP")
    ap.add_argument("--ping-hz", type=float, default=10.0)
    ap.add_argument("--cmd", choices=sorted(COMMANDS) + ["controller", "param"])
    ap.add_argument("--arg", type=float, default=0.0, help="controller index / param value")
    ap.add_argument("--param", type=int, default=0, help="param index for --cmd param")
    ap.add_argument("--duration", type=float, default=0.0, help="seconds, 0 = until Ctrl+C")
    args = ap.parse_args()

    port = args.port or find_base_port()
    if not port:
        sys.exit("no base station found in /dev/serial/by-id (is the RP2040 flashed and plugged in?)")
    ser = serial.Serial(port, args.baud, timeout=0.01)
    ser.reset_input_buffer()
    print(f"connected: {port}")

    dec = bl.FrameDecoder()
    t0 = time.monotonic()
    now_ms = lambda: int((time.monotonic() - t0) * 1000)
    cmd_id = random.randint(1, 0xFFFF)  # avoid colliding with the last id the bike saw before a restart
    pending = {}                         # cmd id -> send time [ms]
    rtts = []
    tlm_count = tlm_lost = 0
    last_seq = None
    last_tlm = None
    base = None
    next_ping = time.monotonic()
    next_report = time.monotonic() + 1.0

    def send(cmd, param=0, arg=0):
        nonlocal cmd_id
        cmd_id = cmd_id % 0xFFFF + 1
        stamp = now_ms()
        ser.write(bl.encode_frame(bl.command(cmd_id, cmd, stamp, param, arg)))
        pending[cmd_id] = stamp

    if args.cmd:
        if args.cmd == "controller":
            send(bl.CMD_SET_CONTROLLER, arg=int(args.arg))
        elif args.cmd == "param":
            send(bl.CMD_SET_PARAM, param=args.param, arg=int(args.arg * 1000))
        else:
            send(COMMANDS[args.cmd])

    try:
        while not args.duration or time.monotonic() - t0 < args.duration:
            for p in dec.feed(ser.read(4096)):
                kind = p[0]
                if kind == bl.TELEMETRY and len(p) == struct.calcsize(bl.TELEMETRY_FMT):
                    t = bl.Telemetry.parse(p)
                    tlm_count += 1
                    if last_seq is not None:
                        gap = (t.seq - last_seq - 1) & 0xFFFF
                        if gap > 1000:
                            print("[bike restarted]")
                        else:
                            tlm_lost += gap
                    last_seq = t.seq
                    last_tlm = t
                    sent = pending.pop(t.last_cmd_id, None)
                    if sent is not None:
                        rtts.append(now_ms() - sent)
                elif kind == bl.BASE_STATUS and len(p) == struct.calcsize(bl.BASE_STATUS_FMT):
                    base = bl.BaseStatus.parse(p)
                elif kind == bl.LOG:
                    print(f"[log] {p[1:].decode(errors='replace')}")

            now = time.monotonic()
            if args.ping_hz > 0 and now >= next_ping:
                next_ping = now + 1.0 / args.ping_hz
                send(bl.CMD_PING)
                for k in [k for k, v in pending.items() if now_ms() - v > 2000]:
                    del pending[k]

            if now >= next_report:
                next_report += 1.0
                line = f"tlm {tlm_count:3d}/s lost {tlm_lost}"
                if rtts:
                    line += f" | rtt ms min/avg/max {min(rtts)}/{sum(rtts) / len(rtts):.1f}/{max(rtts)}"
                else:
                    line += " | rtt -"
                if last_tlm:
                    line += (f" | bike {flags_str(last_tlm.flags)} ctrl={last_tlm.controller}"
                             f" roll={last_tlm.roll:+.3f}"
                             f" txfail={last_tlm.tx_failures}")
                if base:
                    line += f" | base radio={'OK' if base.radio_ok else 'DOWN'} rx={base.rx_packets}"
                if dec.crc_errors:
                    line += f" | crc_err={dec.crc_errors}"
                print(line, flush=True)
                tlm_count = 0
                rtts.clear()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
