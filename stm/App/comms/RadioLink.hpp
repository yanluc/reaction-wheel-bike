#pragma once
// Bike side of the nRF24 link (PTX, CE high). Telemetry is ACKed by the chip; the base answers it with
// an ACK payload carrying the next Command (or a heartbeat). MAX_RT drops the packet: the next
// telemetry is fresher than a resend.

#include <cstdint>

#include "hal/Hal.hpp"
#include "nrf24/Nrf24.hpp"
#include "proto/Messages.hpp"

namespace bike {

class ICommandSink {
public:
    virtual void onCommand(const proto::Command& cmd) = 0;

protected:
    ~ICommandSink() = default;
};

class RadioLink {
public:
    struct Stats {
        uint32_t telemetrySent = 0;
        uint32_t telemetryAcked = 0;   // ACK payload received
        uint32_t telemetryFailed = 0;  // MAX_RT
        uint32_t commands = 0;
    };

    RadioLink(nrf24::Nrf24& radio, hal::IClock& clock, ICommandSink& sink, uint32_t linkTimeoutMs = 300)
        : radio_(radio), clock_(clock), sink_(sink), linkTimeoutMs_(linkTimeoutMs) {}

    bool init(const nrf24::Config& cfg);
    // Call once per main-loop iteration.
    void poll() {
        if (inFlight_ || pending_ || queued_) {
            service();
        }
    }

    bool sendTelemetry(const proto::Telemetry& t);  // false while the previous one is not done

    bool linkUp() const;
    const Stats& stats() const { return stats_; }

private:
    // DMA source: nRF24 command byte directly followed by the word-aligned payload.
    struct alignas(4) TxBuffer {
        uint8_t pad[3];
        uint8_t cmd;
        uint32_t words[8];
    };
    static_assert(sizeof(TxBuffer) == 36, "cmd must directly precede the payload");

    void service();
    void serviceFlags(uint8_t status);

    nrf24::Nrf24& radio_;
    hal::IClock& clock_;
    ICommandSink& sink_;
    uint32_t linkTimeoutMs_;
    bool ready_ = false;

    TxBuffer tx_ = {};
    bool pending_ = false;   // waiting to be written to the TX FIFO
    bool inFlight_ = false;  // DMA write to the TX FIFO running
    bool queued_ = false;    // in the TX FIFO, waiting for its ACK payload
    uint32_t queuedMs_ = 0;
    bool backoff_ = false;   // TX FIFO was full: skip one poll

    bool everAcked_ = false;
    uint32_t lastAckMs_ = 0;
    uint16_t lastCmdId_ = 0;
    Stats stats_;
};

}  // namespace bike
