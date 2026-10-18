#include "comms/RadioStream.hpp"

#include <cstring>

namespace bike {
namespace {
constexpr uint32_t kTelemetryTimeoutMs = 20;  // ACK lost without RX_DR/MAX_RT (e.g. empty ACK)
constexpr uint8_t kRxDr = nrf24::Nrf24::kStatusRxDr;
constexpr uint8_t kTxDs = nrf24::Nrf24::kStatusTxDs;
constexpr uint8_t kMaxRt = nrf24::Nrf24::kStatusMaxRt;
}  // namespace

bool RadioStream::init(const nrf24::Config& cfg) {
    ready_ = radio_.init(cfg);
    if (ready_) {
        radio_.setTxMode();
        radio_.setStreaming(true);  // CE high: FIFO entries go out back to back
    }
    return ready_;
}

bool RadioStream::linkUp() const {
    return everAcked_ && (clock_.millis() - lastAckMs_) < linkTimeoutMs_;
}

bool RadioStream::shouldDropForTest(uint16_t seq) const {
    if (!testDrops_) return false;
    if (seq % 997 == 500) return true;                          // isolated losses
    const uint16_t a = seq % 4096;
    if (a >= 2000 && a < 2000 + stream::kFecDepth) return true;  // burst = depth -> recoverable
    const uint16_t b = seq % 16384;
    return b >= 9000 && b < 9000 + stream::kFecDepth + 2;        // burst > depth -> 2 lost
}

HAL_RAM_FUNC RadioStream::Slot* RadioStream::reserveSlot() {
    return ((head_ + 1) % kQueueLen == tail_) ? nullptr : &slots_[head_];
}

HAL_RAM_FUNC void RadioStream::pushSample(const proto::StreamSample& s) {
    if (fill_ == 0) {
        filling_ = reserveSlot();
        if (filling_ == nullptr) {
            ++stats_.queueOverflows;
            filling_ = &scratch_;  // keep the stream (and FEC) consistent; this packet is dropped
        }
    }
    std::memcpy(filling_->bytes() + proto::kStreamHeader + fill_ * sizeof(s), &s, sizeof(s));
    if (++fill_ == proto::kSamplesPerPacket) {
        finishPacket();
    }
}

HAL_RAM_FUNC void RadioStream::finishPacket() {
    fill_ = 0;
    filling_->len = 32;
    uint8_t* b = filling_->bytes();
    const uint16_t seq = seq_++;
    b[0] = static_cast<uint8_t>(proto::MsgType::StreamData);
    b[1] = static_cast<uint8_t>(seq);
    b[2] = static_cast<uint8_t>(seq >> 8);

    // Parity also covers test-dropped packets.
    uint32_t parity[8];
    const bool parityReady = mode_ == LinkMode::NoAckFec && fec_.add(seq, filling_->words, parity);

    if (filling_ != &scratch_) {
        if (shouldDropForTest(seq)) {
            ++stats_.testDrops;
        } else {
            head_ = (head_ + 1) % kQueueLen;
            ++stats_.dataPackets;
        }
    }
    if (parityReady) {
        Slot* p = reserveSlot();
        if (p == nullptr) {
            ++stats_.queueOverflows;
        } else {
            for (int i = 0; i < 8; ++i) p->words[i] = parity[i];
            p->len = 32;
            head_ = (head_ + 1) % kQueueLen;
            ++stats_.parityPackets;
        }
    }
}

bool RadioStream::sendTelemetry(const proto::Telemetry& t) {
    if (!ready_) {
        return false;
    }
    if (telemetryPending_ || telemetryQueued_) {
        return false;
    }
    std::memcpy(telemetry_.bytes(), &t, sizeof(t));
    telemetryPending_ = true;
    return true;
}

void RadioStream::serviceFlags(uint8_t status) {
    {
        // Drain the RX FIFO on every event: a full RX FIFO makes every ACK payload fail (MAX_RT forever).
        uint8_t buf[nrf24::Nrf24::kMaxPayload];
        uint8_t len;
        while ((len = radio_.readPayload(buf)) != 0) {
            telemetryQueued_ = false;
            ++stats_.telemetryAcked;
            everAcked_ = true;
            lastAckMs_ = clock_.millis();
            if (len == sizeof(proto::Command) && buf[0] == static_cast<uint8_t>(proto::MsgType::Command)) {
                proto::Command cmd;
                std::memcpy(&cmd, buf, sizeof(cmd));
                if (cmd.id != 0 && cmd.id != lastCmdId_) {  // 0 = heartbeat
                    lastCmdId_ = cmd.id;
                    ++stats_.commands;
                    sink_.onCommand(cmd);
                }
            }
        }
    }
    if (status & kMaxRt) {
        if (mode_ == LinkMode::Ack) {
            // Clearing MAX_RT makes the chip resend the same packet: nothing is lost.
            ++stats_.retriesExhausted;
        } else {
            // Unacked telemetry would be retried forever: flush (also drops <= 2 stream packets).
            radio_.flushTx();
            telemetryQueued_ = false;
            ++stats_.telemetryFailed;
        }
    }
    radio_.clearFlags(static_cast<uint8_t>((status & (kRxDr | kMaxRt)) | kTxDs));
}

HAL_RAM_FUNC void RadioStream::service() {
    if (!ready_) {
        return;
    }

    if (inFlight_) {
        const nrf24::AsyncResult r = radio_.pollAsync();
        if (r == nrf24::AsyncResult::Busy) {
            return;  // DMA still running; nothing else may use the SPI bus
        }
        if (inFlightTelemetry_) {
            telemetryPending_ = false;
            // NoAckFec: wait for its ACK payload; in Ack mode the chip handles ACKs itself.
            telemetryQueued_ = mode_ == LinkMode::NoAckFec;
            telemetryQueuedMs_ = clock_.millis();
            ++stats_.telemetrySent;
        } else {
            tail_ = (tail_ + 1) % kQueueLen;
        }
        inFlight_ = false;
    }

    // IRQ pin: RX_DR or MAX_RT pending (TX_DS masked).
    if (radio_.irqPending()) {
        serviceFlags(radio_.status());
    }
    if (telemetryQueued_ && clock_.millis() - telemetryQueuedMs_ > kTelemetryTimeoutMs) {
        telemetryQueued_ = false;
    }

    if (backoffPolls_ != 0) {
        --backoffPolls_;
        return;
    }

    if (telemetryPending_) {
        if (radio_.startQueueInPlace(&telemetry_.cmd, sizeof(proto::Telemetry), true)) {
            inFlight_ = true;
            inFlightTelemetry_ = true;
        } else {
            backoffPolls_ = 1;  // TX FIFO full
        }
        return;
    }
    if (head_ != tail_) {
        Slot& slot = slots_[tail_];
        if (radio_.startQueueInPlace(&slot.cmd, slot.len, mode_ == LinkMode::Ack)) {
            inFlight_ = true;
            inFlightTelemetry_ = false;
        } else {
            backoffPolls_ = 1;  // TX FIFO full
        }
    }
}

}  // namespace bike
