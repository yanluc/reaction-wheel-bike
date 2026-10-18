#include "bench/StreamTest.hpp"

#include <cmath>

namespace bike {
namespace {

constexpr uint32_t kSinePeriod = StreamTest::kSampleRateHz / 1;
constexpr uint32_t kTrianglePeriod = StreamTest::kSampleRateHz / 2;
constexpr uint32_t kSquarePeriod = StreamTest::kSampleRateHz * 2;
constexpr uint32_t kSawPeriod = StreamTest::kSampleRateHz / 5;
constexpr float kTwoPi = 6.28318530718f;

}  // namespace

proto::StreamSample StreamTest::makeSample(uint32_t n) {
    proto::StreamSample s;
    const float ps = static_cast<float>(n % kSinePeriod) / kSinePeriod;
    const float pt = static_cast<float>(n % kTrianglePeriod) / kTrianglePeriod;
    const float pw = static_cast<float>(n % kSawPeriod) / kSawPeriod;
    s.v[0] = static_cast<int16_t>(std::lround(10000.0f * std::sin(kTwoPi * ps)));
    s.v[1] = static_cast<int16_t>(std::lround(pt < 0.5f ? -10000.0f + 40000.0f * pt : 30000.0f - 40000.0f * pt));
    s.v[2] = static_cast<int16_t>((n % kSquarePeriod) < kSquarePeriod / 2 ? 8000 : -8000);
    s.v[3] = static_cast<int16_t>(std::lround(-10000.0f + 20000.0f * pw));
    return s;
}

void StreamTest::onCommand(const proto::Command& cmd) {
    lastCmdId_ = cmd.id;
    lastCmdStamp_ = cmd.stamp;
    if (cmd.cmd == proto::CommandId::SetTestDrops) {
        drops_ = cmd.arg != 0;
        radio_.setTestDrops(drops_);
        serial_.log("test drops %s", drops_ ? "ON" : "OFF");
    }
}

void StreamTest::run(const nrf24::Config& cfg) {
    serial_.start();
    const bool ok = radio_.init(cfg);
    serial_.log("STREAM TEST: nRF24 %s, ch=%u, mode=%s, %lu Hz x %u values, %u samples/packet",
                ok ? "OK" : "NOT RESPONDING", cfg.channel, radio_.mode() == LinkMode::Ack ? "ACK" : "NOACK+FEC",
                static_cast<unsigned long>(kSampleRateHz),
                static_cast<unsigned>(sizeof(proto::StreamSample) / sizeof(int16_t)),
                static_cast<unsigned>(proto::kSamplesPerPacket));

    const uint32_t startUs = clock_.micros();
    uint32_t n = 0;
    uint32_t nextTelemetryUs = startUs;
    uint32_t nextStatsUs = startUs + 1000000;
    uint16_t tlmSeq = 0;
    uint32_t genCycles = 0;
    uint32_t radioCycles = 0;
    uint32_t ticks = 0;
    RadioStream::Stats prev;

    while (true) {
        const uint32_t now = clock_.micros();

        const uint64_t due = static_cast<uint64_t>(now - startUs) * kSampleRateHz / 1000000u;
        if (n < due) {
            const uint32_t a = DWT->CYCCNT;
            // Modulo the 16-bit seq cycle: the receiver rebuilds the index from seq alone.
            const proto::StreamSample smp = makeSample(n++ % (65536u * proto::kSamplesPerPacket));
            // Barrier: keep sin()/lround() from sinking past the cycle-counter read.
            asm volatile("" : : "r"(&smp) : "memory");
            const uint32_t b = DWT->CYCCNT;
            radio_.pushSample(smp);
            radio_.poll();
            radioCycles += DWT->CYCCNT - b;
            genCycles += b - a;
            ++ticks;
        }
        serial_.poll();

        if (static_cast<int32_t>(now - nextTelemetryUs) >= 0) {
            nextTelemetryUs += 10000;  // ACKed: carries commands back
            proto::Telemetry t{};
            t.type = proto::MsgType::Telemetry;
            t.seq = tlmSeq;
            t.uptimeMs = clock_.millis();
            t.flags = (radio_.linkUp() ? proto::kFlagRadioLink : 0);
            t.lastCmdId = lastCmdId_;
            t.lastCmdStamp = lastCmdStamp_;
            t.txFailures = static_cast<uint16_t>(radio_.stats().telemetryFailed);
            if (radio_.sendTelemetry(t)) {
                ++tlmSeq;
            }
            ledLink_.write(radio_.linkUp());
        }

        if (static_cast<int32_t>(now - nextStatsUs) >= 0) {
            nextStatsUs += 1000000;
            const RadioStream::Stats& s = radio_.stats();
            serial_.log("data=%lu/s parity=%lu/s acks=%lu fail=%lu overflow=%lu testdrop=%lu cmds=%lu maxrt=%lu",
                        static_cast<unsigned long>(s.dataPackets - prev.dataPackets),
                        static_cast<unsigned long>(s.parityPackets - prev.parityPackets),
                        static_cast<unsigned long>(s.telemetryAcked - prev.telemetryAcked),
                        static_cast<unsigned long>(s.telemetryFailed - prev.telemetryFailed),
                        static_cast<unsigned long>(s.queueOverflows), static_cast<unsigned long>(s.testDrops),
                        static_cast<unsigned long>(s.commands), static_cast<unsigned long>(s.retriesExhausted));
            const uint32_t cpm = SystemCoreClock / 1000000u;
            const uint32_t radioUs = radioCycles / cpm;
            serial_.log("cpu: radio=%lu us/s (%lu.%02lu%%), sample gen=%lu us/s, ticks=%lu/s",
                        static_cast<unsigned long>(radioUs), static_cast<unsigned long>(radioUs / 10000),
                        static_cast<unsigned long>((radioUs / 100) % 100),
                        static_cast<unsigned long>(genCycles / cpm), static_cast<unsigned long>(ticks));
            radioCycles = 0;
            genCycles = 0;
            ticks = 0;
            prev = s;
        }
    }
}

}  // namespace bike
