#pragma once
// Bike side of the nRF24 link (PTX, CE high). The base answers each ACKed telemetry packet with an
// ACK payload carrying the next Command (or a heartbeat).

#include <cstdint>

#include "hal/Hal.hpp"
#include "nrf24/Nrf24.hpp"
#include "proto/Messages.hpp"
#include "stream/XorFec.hpp"

namespace bike {

class ICommandSink {
public:
    virtual void onCommand(const proto::Command& cmd) = 0;

protected:
    ~ICommandSink() = default;
};

// NoAckFec: stream without ACK + XOR parity; MAX_RT on telemetry flushes the TX FIFO.
// Ack: every packet ACKed; on MAX_RT the chip resends the same packet.
enum class LinkMode : uint8_t { NoAckFec, Ack };

class RadioStream {
public:
    struct Stats {
        uint32_t dataPackets = 0;
        uint32_t parityPackets = 0;
        uint32_t queueOverflows = 0;
        uint32_t testDrops = 0;
        uint32_t telemetrySent = 0;
        uint32_t telemetryAcked = 0;
        uint32_t telemetryFailed = 0;  // NoAckFec: MAX_RT
        uint32_t retriesExhausted = 0; // Ack: MAX_RT (packet resent)
        uint32_t commands = 0;
    };

    RadioStream(nrf24::Nrf24& radio, hal::IClock& clock, ICommandSink& sink, LinkMode mode = LinkMode::NoAckFec,
                uint32_t linkTimeoutMs = 300)
        : radio_(radio), clock_(clock), sink_(sink), mode_(mode), linkTimeoutMs_(linkTimeoutMs) {}

    LinkMode mode() const { return mode_; }

    bool init(const nrf24::Config& cfg);
    // Call once per control-loop tick.
    void poll() {
        if (inFlight_ || head_ != tail_ || telemetryPending_ || telemetryQueued_) {
            service();
        }
    }

    void pushSample(const proto::StreamSample& s);
    bool sendTelemetry(const proto::Telemetry& t);  // false while one is pending

    void setTestDrops(bool on) { testDrops_ = on; }

    bool ready() const { return ready_; }
    bool linkUp() const;
    const Stats& stats() const { return stats_; }

private:
    // DMA source: nRF24 command byte directly followed by the word-aligned payload.
    struct alignas(4) Slot {
        uint8_t len;
        uint8_t pad[2];
        uint8_t cmd;
        uint32_t words[8];
        uint8_t* bytes() { return reinterpret_cast<uint8_t*>(words); }
    };
    static_assert(sizeof(Slot) == 36, "cmd must directly precede the payload");

    void service();
    void finishPacket();
    Slot* reserveSlot();
    void serviceFlags(uint8_t status);
    bool shouldDropForTest(uint16_t seq) const;

    static constexpr size_t kQueueLen = 32;

    nrf24::Nrf24& radio_;
    hal::IClock& clock_;
    ICommandSink& sink_;
    LinkMode mode_;
    uint32_t linkTimeoutMs_;
    bool ready_ = false;
    bool testDrops_ = false;

    stream::XorFecEncoder fec_;
    Slot slots_[kQueueLen] = {};
    Slot scratch_ = {};
    Slot* filling_ = nullptr;
    size_t head_ = 0;
    size_t tail_ = 0;
    uint8_t fill_ = 0;
    uint16_t seq_ = 0;

    bool inFlight_ = false;
    bool inFlightTelemetry_ = false;
    uint8_t backoffPolls_ = 0;

    Slot telemetry_ = {};
    bool telemetryPending_ = false;
    bool telemetryQueued_ = false;
    uint32_t telemetryQueuedMs_ = 0;
    bool everAcked_ = false;
    uint32_t lastAckMs_ = 0;
    uint16_t lastCmdId_ = 0;
    Stats stats_;
};

}  // namespace bike
