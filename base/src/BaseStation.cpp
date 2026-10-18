#include "BaseStation.hpp"

#include <cstring>

#include "pico/stdio.h"
#include "pico/time.h"

namespace base {
namespace {

constexpr uint32_t kHeartbeatPeriodUs = 5000;

proto::Command heartbeat() {
    proto::Command c{};
    c.type = proto::MsgType::Command;
    c.id = 0;
    c.cmd = proto::CommandId::Nop;
    return c;
}

}  // namespace

void SharedState::init() {
    queue_init(&rxQueue, sizeof(RadioPacket), 512);  // ~100 ms of packets at 5 k/s
    queue_init(&cmdQueue, sizeof(proto::Command), 8);
}

void RadioCore::init(const nrf24::Config& cfg) {
    cfg_ = cfg;
    const bool ok = radio_.init(cfg_);
    shared_.radioOk = ok;
    if (ok) {
        radio_.startListening();
        loadAck(heartbeat());
    }
}

void RadioCore::loadAck(const proto::Command& cmd) {
    radio_.writeAckPayload(0, reinterpret_cast<const uint8_t*>(&cmd), sizeof(cmd));
    ++shared_.acksLoaded;
}

void RadioCore::poll() {
    if (!shared_.radioOk) {
        const uint32_t now = clock_.millis();
        if (now - lastRetryMs_ > 1000) {  // module plugged in later / brown-out
            lastRetryMs_ = now;
            init(cfg_);
        }
        return;
    }

    if (const uint16_t sweeps = shared_.scanRequest.exchange(0)) {
        runScan(sweeps);
    }

    // Re-check the FIFO after clearing RX_DR: a packet arriving in between raises no new IRQ.
    if (radio_.irqPending()) {
        RadioPacket pkt;
        auto forward = [&] {
            ++shared_.rxPackets;
            if (!queue_try_add(&shared_.rxQueue, &pkt)) {
                ++shared_.rxDropped;
            }
        };
        while (true) {
            while ((pkt.len = radio_.readPayload(pkt.data)) != 0) {
                forward();
            }
            radio_.clearFlags(nrf24::Nrf24::kStatusRxDr);
            if ((pkt.len = radio_.readPayload(pkt.data)) == 0) {
                break;
            }
            forward();
        }
    }

    proto::Command cmd;
    if (queue_try_remove(&shared_.cmdQueue, &cmd)) {
        radio_.flushTx();  // supersede the queued heartbeat / an older command
        loadAck(cmd);
    } else if (clock_.micros() - lastHeartbeatUs_ >= kHeartbeatPeriodUs && radio_.txFifoEmpty()) {
        // Each ACK payload is consumed by one packet; the rest stay empty (shorter on air, bike not woken).
        loadAck(heartbeat());
        lastHeartbeatUs_ = clock_.micros();
    }
}

void RadioCore::runScan(uint16_t sweeps) {
    // Bike link is down meanwhile (~126 ch * 0.2 ms * sweeps).
    for (uint8_t ch = 0; ch < SharedState::kScanChannels; ++ch) {
        shared_.scanHits[ch] = 0;
    }
    for (uint16_t s = 0; s < sweeps; ++s) {
        for (uint8_t ch = 0; ch < SharedState::kScanChannels; ++ch) {
            if (radio_.carrierDetect(ch) && shared_.scanHits[ch] < 255) {
                ++shared_.scanHits[ch];
            }
        }
    }
    shared_.scanSweeps = sweeps;
    shared_.scanReady.store(true, std::memory_order_release);

    radio_.setChannel(cfg_.channel);
    radio_.flushRx();
    radio_.flushTx();
    radio_.startListening();
    loadAck(heartbeat());
}

void UsbCore::sendScanResult() {
    for (uint8_t first = 0; first < SharedState::kScanChannels; first += 64) {
        proto::ScanResult r{};
        r.type = proto::MsgType::ScanResult;
        r.firstChannel = first;
        r.count = static_cast<uint8_t>(SharedState::kScanChannels - first < 64 ? SharedState::kScanChannels - first : 64);
        r.sweeps = shared_.scanSweeps;
        std::memcpy(r.hits, &shared_.scanHits[first], r.count);
        send(&r, sizeof(r));
    }
    flush();
}

void UsbCore::send(const void* msg, size_t len) {
    if (len_ + len + proto::kFrameOverhead > kBatch) {
        flush();
    }
    len_ += proto::encodeFrame(msg, len, &buf_[len_], kBatch - len_);
}

void UsbCore::flush() {
    if (len_ != 0) {
        stdio_put_string(reinterpret_cast<const char*>(buf_), static_cast<int>(len_), false, false);
        len_ = 0;
    }
    lastFlushUs_ = clock_.micros();
}

void UsbCore::log(const char* text) {
    char msg[proto::kMaxFramePayload];
    msg[0] = static_cast<char>(proto::MsgType::Log);
    size_t n = std::strlen(text);
    if (n > sizeof(msg) - 1) n = sizeof(msg) - 1;
    std::memcpy(&msg[1], text, n);
    send(msg, n + 1);
}

void UsbCore::pollUsbIn() {
    char in[64];
    const int n = stdio_get_until(in, sizeof(in), make_timeout_time_us(0));
    for (int i = 0; i < n; ++i) {
        if (!decoder_.feed(static_cast<uint8_t>(in[i]))) {
            continue;
        }
        ++status_.usbFrames;
        const uint8_t* msg = decoder_.payload();
        if (decoder_.size() == sizeof(proto::Command) && msg[0] == static_cast<uint8_t>(proto::MsgType::Command)) {
            proto::Command cmd;
            std::memcpy(&cmd, msg, sizeof(cmd));
            queue_try_add(&shared_.cmdQueue, &cmd);
        } else if (decoder_.size() == sizeof(proto::BaseCommand) &&
                   msg[0] == static_cast<uint8_t>(proto::MsgType::BaseCommand)) {
            proto::BaseCommand bc;
            std::memcpy(&bc, msg, sizeof(bc));
            if (bc.cmd == proto::BaseCommandId::Scan) {
                const uint16_t sweeps = bc.sweeps == 0 ? 1 : (bc.sweeps > 255 ? 255 : bc.sweeps);
                shared_.scanRequest = sweeps;
                log("channel scan started");
            }
        }
    }
}

void UsbCore::poll() {
    RadioPacket pkt;
    while (queue_try_remove(&shared_.rxQueue, &pkt)) {
        send(pkt.data, pkt.len);
    }
    if (len_ > kBatch / 2 || (len_ != 0 && clock_.micros() - lastFlushUs_ > 2000)) {
        flush();
    }

    pollUsbIn();

    if (shared_.scanReady.load(std::memory_order_acquire)) {
        shared_.scanReady = false;
        sendScanResult();
    }

    const bool ok = shared_.radioOk;
    if (ok != lastRadioOk_) {
        lastRadioOk_ = ok;
        log(ok ? "nRF24 OK (PRX)" : "nRF24 NOT RESPONDING - check wiring/3V3 supply");
    }

    const uint32_t now = clock_.millis();
    if (now - lastStatusMs_ >= 1000) {
        lastStatusMs_ = now;
        status_.type = proto::MsgType::BaseStatus;
        status_.radioOk = ok ? 1 : 0;
        status_.rxPackets = shared_.rxPackets;
        status_.acksLoaded = shared_.acksLoaded;
        status_.rxDropped = shared_.rxDropped;
        status_.uptimeMs = now;
        send(&status_, sizeof(status_));
    }
}

}  // namespace base
