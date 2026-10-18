#pragma once
// Stream test (-DBIKE_STREAM_TEST=ON): periodic signals through RadioStream, verified by
// tools/stream_plot.py. SetTestDrops (arg 1/0) toggles intentional packet drops.

#include <cstdint>

#include "comms/RadioStream.hpp"
#include "hal/Hal.hpp"
#include "platform/stm32/SerialLink.hpp"
#include "platform/stm32/Stm32Hal.hpp"

namespace bike {

#ifndef BIKE_STREAM_RATE_HZ
#define BIKE_STREAM_RATE_HZ 6700
#endif

class StreamTest final : public ICommandSink {
public:
    // Signal periods must be whole samples.
    static constexpr uint32_t kSampleRateHz = BIKE_STREAM_RATE_HZ;
    static_assert(kSampleRateHz % 10 == 0, "rate must be a multiple of 10 Hz");

    StreamTest(nrf24::Nrf24& radio, hal::IClock& clock, stm32::SerialLink& serial, stm32::Pin& ledLink,
               LinkMode mode)
        : radio_(radio, clock, *this, mode), clock_(clock), serial_(serial), ledLink_(ledLink) {}

    [[noreturn]] void run(const nrf24::Config& cfg);
    void onCommand(const proto::Command& cmd) override;

    static proto::StreamSample makeSample(uint32_t n);  // same formulas as tools/stream_plot.py

private:
    RadioStream radio_;
    hal::IClock& clock_;
    stm32::SerialLink& serial_;
    stm32::Pin& ledLink_;
    uint16_t lastCmdId_ = 0;
    uint32_t lastCmdStamp_ = 0;
    bool drops_ = false;
};

}  // namespace bike
