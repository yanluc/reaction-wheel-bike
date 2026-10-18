#pragma once
// Messages shared by bike, base and tools/bikelink.py (keep layouts in sync). First byte is MsgType;
// one message = one nRF24 payload (<= 32 B) or one Frame on USB/UART. Packed, little-endian.

#include <cstdint>

namespace proto {

enum class MsgType : uint8_t {
    Telemetry = 0x01,     // bike -> base (radio payload) -> laptop
    Command = 0x02,       // laptop -> base -> bike (radio ACK payload)
    Log = 0x03,           // text, serial links only
    StreamData = 0x05,    // bike -> base, NOACK, samples (see StreamDataPacket)
    StreamParity = 0x06,  // bike -> base, NOACK, XOR parity of one FEC group
    Bench = 0x7E,         // radio throughput test (BIKE_RADIO_BENCH)
    BaseStatus = 0x10,    // base -> laptop
    BaseCommand = 0x11,   // laptop -> base (not forwarded to the bike)
    ScanResult = 0x12,    // base -> laptop
    SimSensors = 0x20,    // simulator -> bike (UART, SIM platform)
    SimActuators = 0x21,  // bike -> simulator (UART, SIM platform)
};

enum class CommandId : uint8_t {
    Nop = 0,      // heartbeat from the base when the laptop has nothing to send
    Ping = 1,     // echoed back in Telemetry (RTT measurement)
    Arm = 2,
    Disarm = 3,
    SetController = 4,  // arg = controller index
    SetParam = 5,       // param = index, arg = value * 1000
    SetTestDrops = 6,   // stream test: arg != 0 -> bike drops packets on purpose (FEC demo)
};

enum TelemetryFlags : uint8_t {
    kFlagArmed = 1 << 0,
    kFlagImuOk = 1 << 1,
    kFlagRadioLink = 1 << 2,  // bike sees ACKs from the base
    kFlagSimPlatform = 1 << 3,
    kFlagFailsafe = 1 << 4,
};

#pragma pack(push, 1)

struct Telemetry {
    MsgType type = MsgType::Telemetry;
    uint16_t seq;
    uint32_t uptimeMs;
    uint8_t controller;
    uint8_t flags;
    int16_t rollMrad;
    int16_t rollRateMradS;
    int16_t motorPermille;  // -1000..1000
    int16_t currentMa;
    uint16_t distLeftMm;
    uint16_t distRightMm;
    uint16_t lastCmdId;      // echo of the last received Command.id
    uint32_t lastCmdStamp;   // echo of Command.stamp (laptop clock) -> RTT
    uint16_t txFailures;     // ACKed packets that hit MAX_RT since boot
};
static_assert(sizeof(Telemetry) <= 32, "Telemetry must fit one nRF24 payload");

struct Command {
    MsgType type = MsgType::Command;
    uint16_t id;     // 0 = no new command (heartbeat)
    CommandId cmd;
    uint32_t stamp;  // laptop timestamp [ms], echoed back
    uint8_t param;
    int32_t arg;
};
static_assert(sizeof(Command) <= 32, "Command must fit one nRF24 ACK payload");

struct BaseStatus {
    MsgType type = MsgType::BaseStatus;
    uint8_t radioOk;
    uint32_t rxPackets;
    uint32_t acksLoaded;
    uint32_t usbFrames;
    uint32_t uptimeMs;
    uint32_t rxDropped;  // received but dropped before USB (inter-core queue full)
};

enum class BaseCommandId : uint8_t { Scan = 1 };

struct BaseCommand {
    MsgType type = MsgType::BaseCommand;
    BaseCommandId cmd;
    uint16_t sweeps;  // Scan: passes over all channels
};

// Scan results are sent in chunks (one frame holds max 128 B).
struct ScanResult {
    MsgType type = MsgType::ScanResult;
    uint8_t firstChannel;
    uint8_t count;
    uint16_t sweeps;
    uint8_t hits[64];  // hits[i] = sweeps with carrier on firstChannel + i (saturates at 255)
};

// NOACK sample stream with XOR parity (stream/XorFec.hpp); keep in sync with tools/bikelink.py.
struct StreamSample {
    int16_t v[4];
};

constexpr uint8_t kStreamHeader = 3;  // type + seq
constexpr uint8_t kStreamPayload = 32 - kStreamHeader;
constexpr uint8_t kSamplesPerPacket = kStreamPayload / sizeof(StreamSample);

// Always a full 32 B so parity XORs fixed blocks. Sample index of samples[i] = seq * kSamplesPerPacket + i.
struct StreamDataPacket {
    MsgType type = MsgType::StreamData;
    uint16_t seq;
    StreamSample samples[kSamplesPerPacket];
    uint8_t pad[kStreamPayload - kSamplesPerPacket * sizeof(StreamSample)];
};
static_assert(sizeof(StreamDataPacket) == 32, "stream packets are full nRF24 payloads");

// Parity packet: payload = XOR of bytes [3..31] of the kFecGroup data packets with
// seq = firstSeq + k * kFecDepth, k = 0..kFecGroup-1.
struct StreamParityPacket {
    MsgType type = MsgType::StreamParity;
    uint16_t firstSeq;
    uint8_t xorPayload[kStreamPayload];
};
static_assert(sizeof(StreamParityPacket) == 32, "stream packets are full nRF24 payloads");

struct BenchPacket {
    MsgType type = MsgType::Bench;
    uint8_t mode;     // BenchMode
    uint32_t seq;     // per mode, restarts at 0 for every phase
    uint32_t timeUs;
    uint8_t fill[22];
};
static_assert(sizeof(BenchPacket) == 32, "BenchPacket is one full nRF24 payload");

enum class BenchMode : uint8_t { Ack = 0, NoAckStream = 1, AckPipelined = 2 };

struct SimSensors {
    MsgType type = MsgType::SimSensors;
    uint32_t timeUs;
    float accel[3];  // m/s^2, body frame
    float gyro[3];   // rad/s
    float distLeftM;
    float distRightM;
    float currentA;
    float wheelSpeedRadS;  // reaction wheel
};

struct SimActuators {
    MsgType type = MsgType::SimActuators;
    float motorCommand;  // -1..1
    uint8_t enabled;
};

#pragma pack(pop)

}  // namespace proto
