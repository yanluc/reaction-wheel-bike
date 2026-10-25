#pragma once
// VL53L1X time-of-flight sensor (register sequence from ST's ULD). init() is blocking (boot, new
// address, ~0.1 s calibration). Measurements are one-shot: trigger() starts one, so the MCU can start
// several sensors at the same moment. poll() reads the result by DMA when GPIO1 signals data ready
// (or, without the pin, after polling the status register every 2 ms).

#include <cstdint>

#include "hal/Hal.hpp"
#include "interfaces/Devices.hpp"

namespace bike {

class Vl53l1x final : public IDistanceSensor {
public:
    static constexpr uint8_t kDefaultAddress = 0x29;

    struct Config {
        uint8_t address = kDefaultAddress;  // 7-bit, set after boot (volatile)
        uint16_t timingBudgetMs = 20;       // short mode: 15, 20, 33, 50, 100, 200, 500
    };

    Vl53l1x(hal::II2c& bus, hal::IOutputPin& xshut, hal::IInputPin* gpio1, hal::IClock& clock, const Config& cfg)
        : bus_(bus), xshut_(xshut), gpio1_(gpio1), clock_(clock), cfg_(cfg) {}

    void shutdown() { xshut_.write(false); }
    bool init() override;
    bool read(float& meters, uint32_t& timeUs) override;
    bool trigger();  // starts one measurement (async); false if not idle or the bus is taken
    bool idle() const { return !ok_ || state_ == State::Idle; }
    void poll();
    uint32_t timingBudgetUs() const { return cfg_.timingBudgetMs * 1000u; }

    bool ok() const { return ok_; }
    uint8_t lastStatus() const { return lastStatus_; }  // raw RESULT__RANGE_STATUS, 9 = valid
    uint16_t lastMm() const { return lastMm_; }
    uint32_t measurements() const { return measurements_; }
    uint32_t invalid() const { return invalid_; }
    uint32_t busErrors() const { return busErrors_; }
    uint32_t timeouts() const { return timeouts_; }

private:
    enum class State : uint8_t { Idle, Triggering, Measuring, CheckStatus, ReadResult, ClearIrq };

    bool wrByte(uint16_t reg, uint8_t v);
    bool wrWord(uint16_t reg, uint16_t v);
    bool rdByte(uint16_t reg, uint8_t& v);
    bool rdWord(uint16_t reg, uint16_t& v);
    bool waitDataReady(uint32_t timeoutMs);
    bool dataReady(uint8_t tioStatus) const { return (tioStatus & 1) == 1; }  // polarity: active high
    void finishResult();
    hal::II2c::Result busResult();  // bus_.result(), counting errors

    hal::II2c& bus_;
    hal::IOutputPin& xshut_;
    hal::IInputPin* gpio1_;
    hal::IClock& clock_;
    Config cfg_;
    uint8_t addr_ = kDefaultAddress;

    bool ok_ = false;
    State state_ = State::Idle;
    uint32_t lastPollMs_ = 0;
    uint8_t status_ = 0;
    uint8_t result_[17] = {};
    uint8_t clear_ = 0x01;
    uint8_t oneShot_ = 0x10;
    uint32_t startUs_ = 0;

    bool fresh_ = false;
    uint8_t lastStatus_ = 0;
    uint16_t lastMm_ = 0;
    uint32_t lastTimeUs_ = 0;
    uint32_t measurements_ = 0;
    uint32_t invalid_ = 0;
    uint32_t busErrors_ = 0;
    uint32_t timeouts_ = 0;
};

}  // namespace bike
