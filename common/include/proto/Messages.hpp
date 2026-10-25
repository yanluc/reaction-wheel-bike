#pragma once
// Messages shared by bike, base and tools/bikelink.py (keep layouts in sync). First byte is MsgType;
// one message = one nRF24 payload (<= 32 B) or one Frame on USB/UART. Packed, little-endian.

#include <cstdint>

namespace proto {

enum class MsgType : uint8_t {
    Telemetry = 0x01,     // bike -> base (radio payload) -> laptop
    Command = 0x02,       // laptop -> base -> bike (radio ACK payload)
    Log = 0x03,           // text, serial links only
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
    // 6: unused (was SetTestDrops)
    Calibrate = 7,      // disarm, restart the estimator calibration (bike must stand still)
};

enum TelemetryFlags : uint8_t {
    kFlagArmed = 1 << 0,
    kFlagImuOk = 1 << 1,
    kFlagRadioLink = 1 << 2,  // bike sees ACKs from the base
    kFlagSimPlatform = 1 << 3,
    kFlagFailsafe = 1 << 4,
    kFlagEstimatorReady = 1 << 5,  // startup calibration done
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
