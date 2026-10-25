#include "nrf24/Nrf24.hpp"

#include <cstring>

namespace nrf24 {
namespace {

constexpr uint8_t CONFIG = 0x00;
constexpr uint8_t EN_AA = 0x01;
constexpr uint8_t EN_RXADDR = 0x02;
constexpr uint8_t SETUP_AW = 0x03;
constexpr uint8_t SETUP_RETR = 0x04;
constexpr uint8_t RF_CH = 0x05;
constexpr uint8_t RF_SETUP = 0x06;
constexpr uint8_t STATUS = 0x07;
constexpr uint8_t OBSERVE_TX = 0x08;
constexpr uint8_t RPD = 0x09;
constexpr uint8_t RX_ADDR_P0 = 0x0A;
constexpr uint8_t TX_ADDR = 0x10;
constexpr uint8_t FIFO_STATUS = 0x17;
constexpr uint8_t DYNPD = 0x1C;
constexpr uint8_t FEATURE = 0x1D;

// CONFIG bits
constexpr uint8_t MASK_TX_DS = 1 << 5;  // no IRQ per sent packet (thousands per second)
constexpr uint8_t EN_CRC = 1 << 3;
constexpr uint8_t CRCO = 1 << 2;
constexpr uint8_t PWR_UP = 1 << 1;
constexpr uint8_t PRIM_RX = 1 << 0;

// STATUS bits
constexpr uint8_t RX_DR = 1 << 6;
constexpr uint8_t TX_DS = 1 << 5;
constexpr uint8_t MAX_RT = 1 << 4;
constexpr uint8_t RX_P_NO_MASK = 0x0E;

// FIFO_STATUS bits
constexpr uint8_t FIFO_TX_EMPTY = 1 << 4;
constexpr uint8_t FIFO_RX_EMPTY = 1 << 0;

// FEATURE bits
constexpr uint8_t EN_DPL = 1 << 2;
constexpr uint8_t EN_ACK_PAY = 1 << 1;
constexpr uint8_t EN_DYN_ACK = 1 << 0;
constexpr uint8_t kFeature = EN_DPL | EN_ACK_PAY | EN_DYN_ACK;

constexpr uint8_t R_REGISTER = 0x00;
constexpr uint8_t W_REGISTER = 0x20;
constexpr uint8_t R_RX_PAYLOAD = 0x61;
constexpr uint8_t W_TX_PAYLOAD = 0xA0;
constexpr uint8_t W_ACK_PAYLOAD = 0xA8;
constexpr uint8_t W_TX_PAYLOAD_NOACK = 0xB0;
constexpr uint8_t FLUSH_TX = 0xE1;
constexpr uint8_t FLUSH_RX = 0xE2;
constexpr uint8_t R_RX_PL_WID = 0x60;
constexpr uint8_t ACTIVATE = 0x50;
constexpr uint8_t NOP = 0xFF;

// Guard against a lost TX_DS/MAX_RT; worst case ARD 4 ms * 15.
constexpr uint32_t kTxTimeoutUs = 70000;

}  // namespace

HAL_RAM_FUNC void Nrf24::waitAsync() {
    while (asyncPending_ && pollAsync() == AsyncResult::Busy) {
    }
}

HAL_RAM_FUNC bool Nrf24::startQueueInPlace(uint8_t* buf, uint8_t len, bool requestAck) {
    if (asyncPending_ || len == 0 || len > kMaxPayload) {
        return false;
    }
    // STATUS.TX_FULL; the datasheet says the write is ignored, but these modules corrupt the FIFO.
    if (status() & 0x01) {
        return false;
    }
    buf[0] = requestAck ? W_TX_PAYLOAD : W_TX_PAYLOAD_NOACK;
    csn_.write(false);
    if (!spi_.startTransfer(buf, asyncRx_, 1u + len)) {
        csn_.write(true);
        return false;
    }
    asyncPending_ = true;
    return true;
}

HAL_RAM_FUNC AsyncResult Nrf24::pollAsync() {
    if (!asyncPending_) {
        return AsyncResult::Idle;
    }
    if (spi_.busy()) {
        return AsyncResult::Busy;
    }
    csn_.write(true);  // CSN rising edge commits the payload
    asyncPending_ = false;
    return AsyncResult::Done;
}

uint8_t Nrf24::command(uint8_t cmd, const uint8_t* tx, uint8_t* rx, uint8_t len) {
    waitAsync();
    uint8_t out[1 + kMaxPayload];
    uint8_t in[1 + kMaxPayload];
    out[0] = cmd;
    if (tx != nullptr) {
        std::memcpy(&out[1], tx, len);
    } else {
        std::memset(&out[1], NOP, len);
    }
    csn_.write(false);
    spi_.transfer(out, in, 1u + len);
    csn_.write(true);
    if (rx != nullptr) {
        std::memcpy(rx, &in[1], len);
    }
    return in[0];  // STATUS is clocked out during the command byte
}

uint8_t Nrf24::readRegister(uint8_t reg) {
    uint8_t v = 0;
    command(R_REGISTER | reg, nullptr, &v, 1);
    return v;
}

void Nrf24::writeRegister(uint8_t reg, uint8_t value) { command(W_REGISTER | reg, &value, nullptr, 1); }

HAL_RAM_FUNC uint8_t Nrf24::status() {
    waitAsync();
    const uint8_t nop = NOP;
    uint8_t st = 0;
    csn_.write(false);
    spi_.transfer(&nop, &st, 1);
    csn_.write(true);
    return st;
}

HAL_RAM_FUNC bool Nrf24::irqPending() {
    if (irq_ != nullptr) {
        return !irq_->read();
    }
    return (status() & (RX_DR | MAX_RT)) != 0;
}

void Nrf24::clearFlags(uint8_t mask) { writeRegister(STATUS, mask); }

void Nrf24::flushTx() { command(FLUSH_TX, nullptr, nullptr, 0); }

void Nrf24::flushRx() { command(FLUSH_RX, nullptr, nullptr, 0); }

bool Nrf24::init(const Config& cfg) {
    ce_.write(false);
    csn_.write(true);
    clock_.delayUs(100000);  // power-on reset

    // Probe: registers must read back what we wrote.
    configReg_ = MASK_TX_DS | EN_CRC | CRCO;
    writeRegister(CONFIG, configReg_);
    writeRegister(SETUP_AW, 0x03);  // 5-byte address
    if (readRegister(CONFIG) != configReg_ || readRegister(SETUP_AW) != 0x03) {
        return false;
    }

    writeRegister(SETUP_RETR, static_cast<uint8_t>(((cfg.retryDelay & 0x0F) << 4) | (cfg.retryCount & 0x0F)));
    writeRegister(RF_CH, cfg.channel & 0x7F);

    uint8_t rf = static_cast<uint8_t>(static_cast<uint8_t>(cfg.paLevel) << 1);
    if (cfg.dataRate == DataRate::Kbps250) rf |= 1 << 5;
    if (cfg.dataRate == DataRate::Mbps2) rf |= 1 << 3;
    writeRegister(RF_SETUP, rf);

    command(W_REGISTER | RX_ADDR_P0, cfg.address, nullptr, 5);
    command(W_REGISTER | TX_ADDR, cfg.address, nullptr, 5);
    writeRegister(EN_AA, 0x01);      // auto-ack on pipe 0
    writeRegister(EN_RXADDR, 0x01);  // only pipe 0

    // Some (older/clone) chips need ACTIVATE before FEATURE is writable.
    writeRegister(FEATURE, kFeature);
    if (readRegister(FEATURE) != kFeature) {
        const uint8_t key = 0x73;
        command(ACTIVATE, &key, nullptr, 1);
        writeRegister(FEATURE, kFeature);
        if (readRegister(FEATURE) != kFeature) {
            return false;
        }
    }
    writeRegister(DYNPD, 0x01);

    flushTx();
    flushRx();
    clearFlags(RX_DR | TX_DS | MAX_RT);

    configReg_ |= PWR_UP;
    writeRegister(CONFIG, configReg_);
    clock_.delayUs(5000);  // power down -> standby needs 1.5 ms
    txPending_ = false;
    return readRegister(RF_CH) == (cfg.channel & 0x7F);
}

void Nrf24::setTxMode() {
    ce_.write(false);
    configReg_ &= static_cast<uint8_t>(~PRIM_RX);
    writeRegister(CONFIG, configReg_);
    txPending_ = false;
}

bool Nrf24::startTransmit(const uint8_t* data, uint8_t len) {
    if (txPending_ || len == 0 || len > kMaxPayload) {
        return false;
    }
    if (!(readRegister(FIFO_STATUS) & FIFO_TX_EMPTY)) {
        flushTx();
    }
    clearFlags(TX_DS | MAX_RT);
    command(W_TX_PAYLOAD, data, nullptr, len);
    ce_.write(true);  // >= 10 us pulse sends exactly one packet
    clock_.delayUs(15);
    ce_.write(false);
    txPending_ = true;
    txStartUs_ = clock_.micros();
    return true;
}

TxResult Nrf24::pollTransmit(uint8_t* ackBuf, uint8_t& ackLen) {
    ackLen = 0;
    if (!txPending_) {
        return TxResult::Idle;
    }
    const uint8_t st = status();
    if (!(st & (TX_DS | MAX_RT))) {
        if (clock_.micros() - txStartUs_ > kTxTimeoutUs) {
            flushTx();
            txPending_ = false;
            return TxResult::Failed;
        }
        return TxResult::Pending;
    }
    // Drain whatever RX_DR says: a full 3-deep RX FIFO rejects every ACK payload (-> MAX_RT forever).
    drainRx(ackBuf, ackLen);
    lastRetries_ = readRegister(OBSERVE_TX) & 0x0F;
    txPending_ = false;
    if (st & TX_DS) {
        clearFlags(TX_DS | RX_DR);
        return TxResult::Ok;
    }
    clearFlags(MAX_RT | RX_DR);
    flushTx();
    return TxResult::Failed;
}

void Nrf24::drainRx(uint8_t* buf, uint8_t& lastLen) {
    uint8_t len;
    while ((len = readPayload(buf)) != 0) {
        lastLen = len;
    }
}

void Nrf24::setStreaming(bool on) {
    txPending_ = false;
    clearFlags(TX_DS | MAX_RT);
    ce_.write(on);
}

void Nrf24::startListening() {
    configReg_ |= PRIM_RX;
    writeRegister(CONFIG, configReg_);
    clearFlags(RX_DR | TX_DS | MAX_RT);
    ce_.write(true);
    clock_.delayUs(130);  // standby -> RX settling
}

uint8_t Nrf24::readPayload(uint8_t* buf) {
    // RX_DR is not cleared per packet: RX_P_NO is what counts, and it saves an SPI transaction.
    if ((status() & RX_P_NO_MASK) == RX_P_NO_MASK) {
        return 0;
    }
    uint8_t width = 0;
    command(R_RX_PL_WID, nullptr, &width, 1);
    if (width == 0 || width > kMaxPayload) {  // corrupted width -> datasheet says flush
        flushRx();
        return 0;
    }
    command(R_RX_PAYLOAD, nullptr, buf, width);
    return width;
}

bool Nrf24::writeAckPayload(uint8_t pipe, const uint8_t* data, uint8_t len) {
    if (pipe > 5 || len == 0 || len > kMaxPayload) {
        return false;
    }
    command(static_cast<uint8_t>(W_ACK_PAYLOAD | pipe), data, nullptr, len);
    return true;
}

void Nrf24::setChannel(uint8_t channel) { writeRegister(RF_CH, channel & 0x7F); }

bool Nrf24::carrierDetect(uint8_t channel, uint32_t dwellUs) {
    ce_.write(false);
    configReg_ |= PRIM_RX;
    writeRegister(CONFIG, configReg_);
    setChannel(channel);
    ce_.write(true);
    clock_.delayUs(dwellUs);  // 130 us settling + listening window
    ce_.write(false);         // RPD is latched when leaving RX
    return (readRegister(RPD) & 0x01) != 0;
}

bool Nrf24::txFifoEmpty() { return (readRegister(FIFO_STATUS) & FIFO_TX_EMPTY) != 0; }

}  // namespace nrf24
