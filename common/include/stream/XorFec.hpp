#pragma once
// Interleaved XOR parity encoder for the NOACK stream (decoder is on the laptop). Group = seq mod
// kFecDepth; parity follows its kFecGroup-th member. One loss per group is recoverable, so any burst
// of <= kFecDepth packets is recovered.

#include <cstdint>
#include <cstring>

#include "proto/Messages.hpp"

namespace stream {

constexpr uint16_t kFecDepth = 8;  // interleaving: recoverable burst length (packets)
constexpr uint16_t kFecGroup = 8;  // data packets per parity packet
constexpr uint16_t kFecBlock = kFecDepth * kFecGroup;
static_assert((65536u % kFecBlock) == 0, "blocks must tile the 16-bit seq space");

class XorFecEncoder {
public:
    // pkt: 32 B data packet as 8 aligned words. Returns true when its group is complete; parity then
    // holds the parity packet to send.
    bool add(uint16_t seq, const uint32_t* pkt, uint32_t* parity) {
        const uint16_t group = seq % kFecDepth;
        uint32_t* acc = acc_[group];
        for (int i = 0; i < 8; ++i) {
            acc[i] ^= pkt[i];
        }
        if ((seq % kFecBlock) / kFecDepth != kFecGroup - 1) {
            return false;
        }
        for (int i = 0; i < 8; ++i) {
            parity[i] = acc[i];
            acc[i] = 0;
        }
        auto* p = reinterpret_cast<uint8_t*>(parity);
        const uint16_t first = static_cast<uint16_t>(seq - (kFecGroup - 1) * kFecDepth);
        p[0] = static_cast<uint8_t>(proto::MsgType::StreamParity);
        p[1] = static_cast<uint8_t>(first);
        p[2] = static_cast<uint8_t>(first >> 8);
        return true;
    }

    void reset() { std::memset(acc_, 0, sizeof(acc_)); }

private:
    uint32_t acc_[kFecDepth][8] = {};
};

}  // namespace stream
