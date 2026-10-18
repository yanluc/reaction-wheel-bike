#include "proto/Frame.hpp"

#include <cstring>

namespace proto {

uint8_t crc8(const uint8_t* data, size_t len, uint8_t crc) {
    while (len--) {
        crc ^= *data++;
        for (int i = 0; i < 8; ++i) {
            crc = (crc & 0x80u) ? static_cast<uint8_t>((crc << 1) ^ 0x07u) : static_cast<uint8_t>(crc << 1);
        }
    }
    return crc;
}

size_t encodeFrame(const void* payload, size_t len, uint8_t* out, size_t outCapacity) {
    if (len == 0 || len > kMaxFramePayload || outCapacity < len + kFrameOverhead) {
        return 0;
    }
    out[0] = kSync1;
    out[1] = kSync2;
    out[2] = static_cast<uint8_t>(len);
    std::memcpy(&out[3], payload, len);
    out[3 + len] = crc8(&out[2], len + 1);
    return len + kFrameOverhead;
}

bool FrameDecoder::feed(uint8_t b) {
    switch (state_) {
        case State::Sync1:
            if (b == kSync1) state_ = State::Sync2;
            break;
        case State::Sync2:
            state_ = (b == kSync2) ? State::Len : (b == kSync1 ? State::Sync2 : State::Sync1);
            break;
        case State::Len:
            if (b == 0 || b > kMaxFramePayload) {
                state_ = State::Sync1;
            } else {
                len_ = b;
                idx_ = 0;
                state_ = State::Data;
            }
            break;
        case State::Data:
            buf_[idx_++] = b;
            if (idx_ >= len_) state_ = State::Crc;
            break;
        case State::Crc: {
            state_ = State::Sync1;
            if (crc8(buf_, len_, crc8(&len_, 1)) == b) {
                return true;
            }
            ++crcErrors_;
            break;
        }
    }
    return false;
}

}  // namespace proto
