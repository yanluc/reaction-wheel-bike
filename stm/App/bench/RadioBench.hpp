#pragma once
// Radio throughput benchmark (-DBIKE_RADIO_BENCH=ON); arrivals are counted by tools/radio_bench.py.
// AckPipelined: on MAX_RT only the flag is cleared, so the chip resends the same packet.

#include <cstdint>

#include "hal/Hal.hpp"
#include "nrf24/Nrf24.hpp"
#include "platform/stm32/SerialLink.hpp"
#include "proto/Messages.hpp"

namespace bike {

class RadioBench {
public:
    RadioBench(nrf24::Nrf24& radio, hal::IClock& clock, stm32::SerialLink& serial)
        : radio_(radio), clock_(clock), serial_(serial) {}

    [[noreturn]] void run(const nrf24::Config& cfg, uint32_t phaseMs = 5000);

private:
    void runAckPhase(uint32_t untilMs);
    void runStreamPhase(uint32_t untilMs);
    void runAckPipelinedPhase(uint32_t untilMs);
    void fill(proto::BenchPacket& p, proto::BenchMode mode);

    nrf24::Nrf24& radio_;
    hal::IClock& clock_;
    stm32::SerialLink& serial_;
    uint32_t seq_ = 0;
};

}  // namespace bike
