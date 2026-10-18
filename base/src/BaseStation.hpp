#pragma once
// nRF24 PRX <-> USB CDC bridge. core0 only services the radio, core1 does USB, so a slow USB write
// never stalls reception (the 3-deep RX FIFO fills in ~0.5 ms at 2 Mbps). Laptop Commands become
// ACK payloads; otherwise a heartbeat (id 0) stays loaded so the bike always gets an answer.

#include <atomic>
#include <cstdint>

#include "hal/Hal.hpp"
#include "nrf24/Nrf24.hpp"
#include "pico/util/queue.h"
#include "proto/Frame.hpp"
#include "proto/Messages.hpp"

namespace base {

struct RadioPacket {
    uint8_t len;
    uint8_t data[nrf24::Nrf24::kMaxPayload];
};

struct SharedState {
    queue_t rxQueue;   // RadioPacket, core0 -> core1
    queue_t cmdQueue;  // proto::Command, core1 -> core0
    std::atomic<uint32_t> rxPackets{0};
    std::atomic<uint32_t> rxDropped{0};  // rxQueue full (USB not keeping up)
    std::atomic<uint32_t> acksLoaded{0};
    std::atomic<bool> radioOk{false};

    // Channel scan: core1 sets scanRequest (sweeps), core0 fills scanHits and sets scanReady.
    static constexpr uint8_t kScanChannels = 126;  // RF_CH 0..125 = 2400..2525 MHz
    std::atomic<uint16_t> scanRequest{0};
    std::atomic<bool> scanReady{false};
    uint8_t scanHits[kScanChannels] = {};
    uint16_t scanSweeps = 0;

    void init();
};

class RadioCore {
public:
    RadioCore(nrf24::Nrf24& radio, hal::IClock& clock, SharedState& shared)
        : radio_(radio), clock_(clock), shared_(shared) {}

    void init(const nrf24::Config& cfg);
    void poll();

private:
    void loadAck(const proto::Command& cmd);
    void runScan(uint16_t sweeps);

    nrf24::Nrf24& radio_;
    hal::IClock& clock_;
    SharedState& shared_;
    nrf24::Config cfg_{};
    uint32_t lastRetryMs_ = 0;
    uint32_t lastHeartbeatUs_ = 0;
};

class UsbCore {
public:
    UsbCore(hal::IClock& clock, SharedState& shared) : clock_(clock), shared_(shared) {}

    void poll();

private:
    void pollUsbIn();
    void sendScanResult();
    void send(const void* msg, size_t len);
    void flush();
    void log(const char* text);

    hal::IClock& clock_;
    SharedState& shared_;
    proto::FrameDecoder decoder_;
    proto::BaseStatus status_{};
    uint32_t lastStatusMs_ = 0;
    bool lastRadioOk_ = false;
    // USB CDC is far more efficient with large writes than with one call per frame.
    static constexpr size_t kBatch = 2048;
    uint8_t buf_[kBatch] = {};
    size_t len_ = 0;
    uint32_t lastFlushUs_ = 0;
};

}  // namespace base
