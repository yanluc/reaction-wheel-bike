#include "bench/RadioBench.hpp"

#include <cstring>

namespace bike {

void RadioBench::fill(proto::BenchPacket& p, proto::BenchMode mode) {
    p.type = proto::MsgType::Bench;
    p.mode = static_cast<uint8_t>(mode);
    p.seq = seq_++;
    p.timeUs = clock_.micros();
}

void RadioBench::runAckPhase(uint32_t untilMs) {
    radio_.setTxMode();
    proto::BenchPacket p{};
    uint8_t ack[nrf24::Nrf24::kMaxPayload];
    uint8_t ackLen = 0;
    uint32_t ok = 0, fail = 0, retries = 0;
    uint32_t secStart = clock_.millis();

    while (static_cast<int32_t>(clock_.millis() - untilMs) < 0) {
        serial_.poll();
        fill(p, proto::BenchMode::Ack);
        if (!radio_.startTransmit(reinterpret_cast<const uint8_t*>(&p), sizeof(p))) {
            continue;
        }
        nrf24::TxResult r;
        while ((r = radio_.pollTransmit(ack, ackLen)) == nrf24::TxResult::Pending) {
        }
        if (r == nrf24::TxResult::Ok) {
            ++ok;
            retries += radio_.lastRetries();
        } else {
            ++fail;
        }
        if (clock_.millis() - secStart >= 1000) {
            secStart += 1000;
            serial_.log("bench ACK: sent ok=%lu/s fail=%lu/s retries=%lu | STATUS=%02x FIFO=%02x OBS=%02x",
                        static_cast<unsigned long>(ok), static_cast<unsigned long>(fail),
                        static_cast<unsigned long>(retries), radio_.status(), radio_.fifoStatus(),
                        radio_.observeTx());
            ok = fail = retries = 0;
        }
    }
}

void RadioBench::runStreamPhase(uint32_t untilMs) {
    radio_.setTxMode();
    radio_.setStreaming(true);
    proto::BenchPacket p{};
    fill(p, proto::BenchMode::NoAckStream);
    uint32_t queued = 0;
    uint32_t secStart = clock_.millis();

    while (static_cast<int32_t>(clock_.millis() - untilMs) < 0) {
        serial_.poll();
        if (radio_.queueNoAck(reinterpret_cast<const uint8_t*>(&p), sizeof(p))) {
            ++queued;
            fill(p, proto::BenchMode::NoAckStream);
        }
        if (clock_.millis() - secStart >= 1000) {
            secStart += 1000;
            serial_.log("bench STREAM: queued=%lu/s", static_cast<unsigned long>(queued));
            queued = 0;
        }
    }
    radio_.setStreaming(false);
    radio_.flushTx();
}

void RadioBench::runAckPipelinedPhase(uint32_t untilMs) {
    radio_.setTxMode();
    radio_.setStreaming(true);
    alignas(4) uint8_t buf[1 + sizeof(proto::BenchPacket)];
    proto::BenchPacket p{};
    fill(p, proto::BenchMode::AckPipelined);
    uint32_t queued = 0, maxRt = 0;
    uint32_t secStart = clock_.millis();

    while (static_cast<int32_t>(clock_.millis() - untilMs) < 0) {
        serial_.poll();
        std::memcpy(&buf[1], &p, sizeof(p));
        if (radio_.startQueueInPlace(buf, sizeof(p), true)) {
            while (radio_.pollAsync() == nrf24::AsyncResult::Busy) {
            }
            ++queued;
            fill(p, proto::BenchMode::AckPipelined);
        }
        if (radio_.irqPending()) {
            const uint8_t st = radio_.status();
            if (st & nrf24::Nrf24::kStatusMaxRt) ++maxRt;
            uint8_t ack[nrf24::Nrf24::kMaxPayload];
            uint8_t ackLen = 0;
            radio_.drainRx(ack, ackLen);  // a full RX FIFO would block every ACK with payload
            radio_.clearFlags(st & (nrf24::Nrf24::kStatusMaxRt | nrf24::Nrf24::kStatusRxDr));
        }
        if (clock_.millis() - secStart >= 1000) {
            secStart += 1000;
            serial_.log("bench ACK-PIPE: queued=%lu/s max_rt=%lu/s", static_cast<unsigned long>(queued),
                        static_cast<unsigned long>(maxRt));
            queued = maxRt = 0;
        }
    }
    radio_.setStreaming(false);
    radio_.flushTx();
}

void RadioBench::run(const nrf24::Config& cfg, uint32_t phaseMs) {
    serial_.start();
    const bool ok = radio_.init(cfg);
    serial_.log("RADIO BENCH: nRF24 %s, %s, phases %lu ms", ok ? "OK" : "NOT RESPONDING",
                cfg.dataRate == nrf24::DataRate::Mbps2 ? "2 Mbps" : "1 Mbps", static_cast<unsigned long>(phaseMs));
    while (true) {
        seq_ = 0;
        runAckPhase(clock_.millis() + phaseMs);
        seq_ = 0;
        runStreamPhase(clock_.millis() + phaseMs);
        seq_ = 0;
        runAckPipelinedPhase(clock_.millis() + phaseMs);
    }
}

}  // namespace bike
