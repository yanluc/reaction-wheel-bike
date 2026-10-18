#pragma once
// Byte-stream framing for USB CDC / UART links:
//   0xAA 0x55 | len | payload[len] | crc8(len, payload)     crc8: poly 0x07, init 0
// The payload is one message from Messages.hpp (first byte = MsgType).

#include <cstddef>
#include <cstdint>

namespace proto {

constexpr uint8_t kSync1 = 0xAA;
constexpr uint8_t kSync2 = 0x55;
constexpr size_t kMaxFramePayload = 128;  // serial links only; radio payloads are <= 32 B
constexpr size_t kFrameOverhead = 4;

uint8_t crc8(const uint8_t* data, size_t len, uint8_t crc = 0);

// Returns frame size, 0 on error.
size_t encodeFrame(const void* payload, size_t len, uint8_t* out, size_t outCapacity);

class FrameDecoder {
public:
    bool feed(uint8_t byte);
    const uint8_t* payload() const { return buf_; }
    size_t size() const { return len_; }
    uint32_t crcErrors() const { return crcErrors_; }

private:
    enum class State : uint8_t { Sync1, Sync2, Len, Data, Crc };
    State state_ = State::Sync1;
    uint8_t buf_[kMaxFramePayload] = {};
    uint8_t len_ = 0;
    uint8_t idx_ = 0;
    uint32_t crcErrors_ = 0;
};

}  // namespace proto
