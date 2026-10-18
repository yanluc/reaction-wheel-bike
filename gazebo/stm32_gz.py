#!/usr/bin/env python3
"""STM32 <-> Gazebo bridge without ROS: MSG_CMD frames -> cmd_vel Twist, Odometry -> MSG_ODOM frames.

Frame: 0xAA 0x55 | type | len | payload | crc8(poly 0x07) over type..payload
Needs gz bindings (source /opt/ros/jazzy/setup.bash) and system Python: /usr/bin/python3 stm32_gz.py
"""
import argparse
import math
import struct
import threading
import time

import serial
from gz.msgs10.odometry_pb2 import Odometry
from gz.msgs10.twist_pb2 import Twist
from gz.transport13 import Node

SYNC = b"\xAA\x55"
MSG_CMD = 0x01
MSG_ODOM = 0x02
MODE_NAMES = ["STOP", "FORWARD", "CIRCLE", "RETURN_HOME"]


def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def build_frame(msg_type: int, payload: bytes) -> bytes:
    body = bytes([msg_type, len(payload)]) + payload
    return SYNC + body + bytes([crc8(body)])


class FrameParser:
    def __init__(self):
        self.buf = bytearray()
        self.crc_errors = 0

    def feed(self, data: bytes):
        self.buf += data
        frames = []
        while True:
            i = self.buf.find(SYNC)
            if i < 0:
                del self.buf[:-1]  # keep a possible trailing 0xAA
                return frames
            del self.buf[:i]
            if len(self.buf) < 4:
                return frames
            n = self.buf[3]
            if n > 16:
                del self.buf[:2]
                continue
            if len(self.buf) < 5 + n:
                return frames
            body = bytes(self.buf[2:4 + n])
            if crc8(body) == self.buf[4 + n]:
                frames.append((body[0], body[2:]))
                del self.buf[:5 + n]
            else:
                self.crc_errors += 1
                del self.buf[:2]


class Bridge:
    def __init__(self, args):
        self.ser = serial.Serial(args.port, args.baud, timeout=0.05)
        self.ser.reset_input_buffer()
        self.tx_lock = threading.Lock()
        self.odom_period = 1.0 / args.odom_rate
        self.cmd_timeout = args.cmd_timeout
        print(f"Opened {args.port} @ {args.baud} baud", flush=True)

        self.node = Node()
        self.pub = self.node.advertise(f"/model/{args.model}/cmd_vel", Twist)
        if not self.node.subscribe(Odometry, f"/model/{args.model}/odometry", self.on_odom):
            raise RuntimeError("odometry subscribe failed")

        self.parser = FrameParser()
        self.last_cmd_time = 0.0
        self.last_odom_tx = 0.0
        self.stopped = True
        self.mode = None
        self.rx_frames = 0
        self.last_seq = None
        self.lost = 0

    def on_odom(self, msg: Odometry):
        # gz-transport thread, hence tx_lock
        now = time.monotonic()
        if now - self.last_odom_tx < self.odom_period:
            return
        self.last_odom_tx = now
        p = msg.pose.position
        q = msg.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        payload = struct.pack(
            "<iihh",
            int(round(p.x * 1000)),
            int(round(p.y * 1000)),
            int(round(yaw * 1000)),
            max(-32768, min(32767, int(round(msg.twist.linear.x * 1000)))),
        )
        with self.tx_lock:
            self.ser.write(build_frame(MSG_ODOM, payload))

    def on_cmd(self, vx_mm_s, wz_mrad_s, mode, seq):
        self.rx_frames += 1
        if self.last_seq is not None:
            self.lost += (seq - self.last_seq - 1) & 0xFF
        self.last_seq = seq
        if mode != self.mode:
            self.mode = mode
            print(f"STM32 mode -> {MODE_NAMES[mode] if mode < len(MODE_NAMES) else mode}", flush=True)

        msg = Twist()
        msg.linear.x = vx_mm_s / 1000.0
        msg.angular.z = wz_mrad_s / 1000.0
        self.pub.publish(msg)
        self.last_cmd_time = time.monotonic()
        self.stopped = False

    def run(self):
        next_report = time.monotonic() + 5.0
        while True:
            data = self.ser.read(self.ser.in_waiting or 1)
            for msg_type, payload in self.parser.feed(data):
                if msg_type == MSG_CMD and len(payload) == 6:
                    self.on_cmd(*struct.unpack("<hhBB", payload))

            now = time.monotonic()
            if not self.stopped and now - self.last_cmd_time > self.cmd_timeout:
                self.pub.publish(Twist())
                self.stopped = True
                print("No commands from STM32 -> robot stopped", flush=True)
            if now >= next_report:
                print(f"rx {self.rx_frames / 5.0:.1f} frames/s, lost {self.lost}, "
                      f"crc errors {self.parser.crc_errors}", flush=True)
                self.rx_frames = 0
                next_report += 5.0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=2000000)
    ap.add_argument("--model", default="vehicle_blue")
    ap.add_argument("--cmd-timeout", type=float, default=0.3)
    ap.add_argument("--odom-rate", type=float, default=50.0)
    bridge = Bridge(ap.parse_args())
    try:
        bridge.run()
    except KeyboardInterrupt:
        pass
    finally:
        bridge.pub.publish(Twist())
        bridge.ser.close()


if __name__ == "__main__":
    main()
