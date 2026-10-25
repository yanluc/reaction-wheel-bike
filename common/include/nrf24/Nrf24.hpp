#pragma once
// Portable nRF24L01+ driver. PTX: startTransmit() + pollTransmit(); PRX: startListening(),
// readPayload() until 0, writeAckPayload(); stream: setStreaming(true) + startQueueInPlace()/pollAsync().

#include <cstdint>

#include "hal/Hal.hpp"

namespace nrf24 {

enum class DataRate : uint8_t { Kbps250, Mbps1, Mbps2 };
enum class PaLevel : uint8_t { Min = 0, Low = 1, High = 2, Max = 3 };  // -18, -12, -6, 0 dBm

struct Config {
    uint8_t channel = 20;  // 2400 + ch MHz, keep <= 83
    DataRate dataRate = DataRate::Mbps2;
    PaLevel paLevel = PaLevel::Low;
    uint8_t address[5] = {'B', 'I', 'K', 'E', '1'};
    uint8_t retryDelay = 1;   // ARD: (n + 1) * 250 us; ACK payloads at 2 Mbps need >= 250 us
    uint8_t retryCount = 3;   // ARC: 0..15
};

enum class TxResult : uint8_t { Idle, Pending, Ok, Failed };
enum class AsyncResult : uint8_t { Idle, Busy, Done };

class Nrf24 {
public:
    static constexpr uint8_t kMaxPayload = 32;

    // irq: optional, active low. TX_DS is masked, so it only signals RX_DR and MAX_RT.
    Nrf24(hal::ISpi& spi, hal::IOutputPin& ce, hal::IOutputPin& csn, hal::IClock& clock,
          hal::IInputPin* irq = nullptr)
        : spi_(spi), ce_(ce), csn_(csn), clock_(clock), irq_(irq) {}

    bool irqPending();

    // Powers up in standby. Returns false if the chip does not answer (register read-back mismatch).
    bool init(const Config& cfg);

    void setTxMode();
    bool startTransmit(const uint8_t* data, uint8_t len);
    // Ok/Failed are reported once. All ACK payloads are drained; ackBuf holds the newest.
    TxResult pollTransmit(uint8_t* ackBuf, uint8_t& ackLen);
    uint8_t lastRetries() const { return lastRetries_; }

    // CE stays high while streaming, so the chip sends FIFO entries back to back.
    void setStreaming(bool on);

    // Zero-copy async write: buf[0] is reserved for the command byte, buf[1..len] is the payload; keep
    // buf untouched until pollAsync() is Done. Refuses a full TX FIFO: writing one corrupts queued
    // payloads on these modules. Synchronous calls first wait for a pending async write.
    bool startQueueInPlace(uint8_t* buf, uint8_t len, bool requestAck);
    AsyncResult pollAsync();

    void clearFlags(uint8_t mask);  // STATUS bits, write 1 to clear

    static constexpr uint8_t kStatusRxDr = 1 << 6;
    static constexpr uint8_t kStatusTxDs = 1 << 5;
    static constexpr uint8_t kStatusMaxRt = 1 << 4;

    void startListening();
    uint8_t readPayload(uint8_t* buf);  // returns length, 0 if nothing/invalid
    // buf/lastLen get the newest RX FIFO entry; lastLen untouched if the FIFO was empty.
    void drainRx(uint8_t* buf, uint8_t& lastLen);
    bool writeAckPayload(uint8_t pipe, const uint8_t* data, uint8_t len);
    bool txFifoEmpty();

    void flushTx();
    void flushRx();

    // Returns RPD (received power > -64 dBm). Leaves the chip in standby (CE low).
    bool carrierDetect(uint8_t channel, uint32_t dwellUs = 200);
    void setChannel(uint8_t channel);

    uint8_t status();
    uint8_t fifoStatus() { return readRegister(0x17); }
    uint8_t observeTx() { return readRegister(0x08); }   // PLOS_CNT | ARC_CNT

private:
    uint8_t readRegister(uint8_t reg);
    void writeRegister(uint8_t reg, uint8_t value);
    void waitAsync();
    uint8_t command(uint8_t cmd, const uint8_t* tx, uint8_t* rx, uint8_t len);

    hal::ISpi& spi_;
    hal::IOutputPin& ce_;
    hal::IOutputPin& csn_;
    hal::IClock& clock_;
    hal::IInputPin* irq_;
    uint8_t configReg_ = 0;
    bool txPending_ = false;
    uint32_t txStartUs_ = 0;
    uint8_t lastRetries_ = 0;
    bool asyncPending_ = false;
    uint8_t asyncRx_[1 + kMaxPayload] = {};
};

}  // namespace nrf24
