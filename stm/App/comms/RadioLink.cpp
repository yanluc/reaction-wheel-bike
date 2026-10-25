#include "comms/RadioLink.hpp"

#include <cstring>

namespace bike {
namespace {
constexpr uint32_t kAckTimeoutMs = 20;  // ACK without payload (no RX_DR) or lost IRQ
constexpr uint8_t kRxDr = nrf24::Nrf24::kStatusRxDr;
constexpr uint8_t kTxDs = nrf24::Nrf24::kStatusTxDs;
constexpr uint8_t kMaxRt = nrf24::Nrf24::kStatusMaxRt;
}  // namespace

bool RadioLink::init(const nrf24::Config& cfg) {
    ready_ = radio_.init(cfg);
    if (ready_) {
        radio_.setTxMode();
        radio_.setStreaming(true);  // CE high: a queued packet goes out at once
    }
    return ready_;
}

bool RadioLink::linkUp() const {
    return everAcked_ && (clock_.millis() - lastAckMs_) < linkTimeoutMs_;
}

bool RadioLink::sendTelemetry(const proto::Telemetry& t) {
    if (!ready_ || pending_ || inFlight_ || queued_) {
        return false;
    }
    std::memcpy(tx_.words, &t, sizeof(t));
    pending_ = true;
    return true;
}

void RadioLink::serviceFlags(uint8_t status) {
    // Drain the RX FIFO on every event: a full RX FIFO makes every ACK payload fail (MAX_RT forever).
    uint8_t buf[nrf24::Nrf24::kMaxPayload];
    uint8_t len;
    while ((len = radio_.readPayload(buf)) != 0) {
        queued_ = false;
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
    if (status & kMaxRt) {
        // Clearing MAX_RT alone would resend the stale packet forever: drop it.
        radio_.flushTx();
        queued_ = false;
        ++stats_.telemetryFailed;
    }
    radio_.clearFlags(static_cast<uint8_t>((status & (kRxDr | kMaxRt)) | kTxDs));
}

void RadioLink::service() {
    if (!ready_) {
        return;
    }
    if (inFlight_) {
        if (radio_.pollAsync() == nrf24::AsyncResult::Busy) {
            return;  // DMA still running; nothing else may use the SPI bus
        }
        inFlight_ = false;
        queued_ = true;
        queuedMs_ = clock_.millis();
        ++stats_.telemetrySent;
    }

    // IRQ pin: RX_DR or MAX_RT pending (TX_DS masked).
    if (radio_.irqPending()) {
        serviceFlags(radio_.status());
    }
    if (queued_ && clock_.millis() - queuedMs_ > kAckTimeoutMs) {
        queued_ = false;
    }

    if (backoff_) {
        backoff_ = false;
    } else if (pending_) {
        if (radio_.startQueueInPlace(&tx_.cmd, sizeof(proto::Telemetry), true)) {
            pending_ = false;
            inFlight_ = true;
        } else {
            backoff_ = true;  // TX FIFO full
        }
    }
}

}  // namespace bike
