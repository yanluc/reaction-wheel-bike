"""Laptop side of the bike protocol. Mirrors common/include/proto/{Messages,Frame}.hpp - keep in sync.

Frame: 0xAA 0x55 | len | payload | crc8(len + payload), crc8 poly 0x07 init 0.
"""
import struct
from dataclasses import dataclass

SYNC = b"\xAA\x55"
MAX_PAYLOAD = 128

TELEMETRY = 0x01
COMMAND = 0x02
LOG = 0x03
BASE_STATUS = 0x10
BASE_COMMAND = 0x11
SCAN_RESULT = 0x12
SIM_SENSORS = 0x20
SIM_ACTUATORS = 0x21

STREAM_DATA = 0x05
STREAM_PARITY = 0x06
BENCH = 0x7E

CMD_NOP, CMD_PING, CMD_ARM, CMD_DISARM, CMD_SET_CONTROLLER, CMD_SET_PARAM, CMD_SET_TEST_DROPS, CMD_CALIBRATE = range(8)

# Sample stream (proto::StreamSample, stream/XorFec.hpp)
SAMPLE_FMT = "<4h"
SAMPLE_VALUES = 4
SAMPLES_PER_PACKET = 3
STREAM_HEADER = 3
FEC_DEPTH = 8
FEC_GROUP = 8
FEC_BLOCK = FEC_DEPTH * FEC_GROUP

FLAG_ARMED, FLAG_IMU_OK, FLAG_RADIO_LINK, FLAG_SIM, FLAG_FAILSAFE, FLAG_EST_READY = (1 << i for i in range(6))

TELEMETRY_FMT = "<BHIBBhhhhHHHIH"
COMMAND_FMT = "<BHBIBi"
BASE_STATUS_FMT = "<BBIIIII"
BASE_COMMAND_FMT = "<BBH"
SCAN_RESULT_FMT = "<BBBH64s"
BASE_CMD_SCAN = 1
SIM_SENSORS_FMT = "<BI10f"
SIM_ACTUATORS_FMT = "<BfB"


def crc8(data: bytes | bytearray, crc: int = 0) -> int:
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def encode_frame(payload: bytes) -> bytes:
    if not 0 < len(payload) <= MAX_PAYLOAD:
        raise ValueError("bad payload length")
    body = bytes([len(payload)]) + payload
    return SYNC + body + bytes([crc8(body)])


class FrameDecoder:
    def __init__(self):
        self.buf = bytearray()
        self.crc_errors = 0

    def feed(self, data: bytes):
        self.buf += data
        out = []
        while True:
            i = self.buf.find(SYNC)
            if i < 0:
                del self.buf[:-1]
                return out
            del self.buf[:i]
            if len(self.buf) < 3:
                return out
            n = self.buf[2]
            if n == 0 or n > MAX_PAYLOAD:
                del self.buf[:2]
                continue
            if len(self.buf) < 4 + n:
                return out
            if crc8(self.buf[2:3 + n]) == self.buf[3 + n]:
                out.append(bytes(self.buf[3:3 + n]))
                del self.buf[:4 + n]
            else:
                self.crc_errors += 1
                del self.buf[:2]


@dataclass
class Telemetry:
    seq: int
    uptime_ms: int
    controller: int
    flags: int
    roll: float
    roll_rate: float
    motor: float
    current: float
    dist_left: float
    dist_right: float
    last_cmd_id: int
    last_cmd_stamp: int
    tx_failures: int

    @classmethod
    def parse(cls, p: bytes):
        f = struct.unpack(TELEMETRY_FMT, p)
        return cls(f[1], f[2], f[3], f[4], f[5] / 1000, f[6] / 1000, f[7] / 1000, f[8] / 1000,
                   f[9] / 1000, f[10] / 1000, f[11], f[12], f[13])


@dataclass
class BaseStatus:
    radio_ok: bool
    rx_packets: int
    acks_loaded: int
    usb_frames: int
    uptime_ms: int
    rx_dropped: int

    @classmethod
    def parse(cls, p: bytes):
        f = struct.unpack(BASE_STATUS_FMT, p)
        return cls(bool(f[1]), f[2], f[3], f[4], f[5], f[6])


def command(cmd_id: int, cmd: int, stamp_ms: int, param: int = 0, arg: int = 0) -> bytes:
    return struct.pack(COMMAND_FMT, COMMAND, cmd_id & 0xFFFF, cmd, stamp_ms & 0xFFFFFFFF, param, arg)
