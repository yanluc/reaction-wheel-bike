#pragma once
// ISM330DHCX on SPI (mode 3, <= 10 MHz): accel + gyro at 6.66 kHz. INT1 pulses on every gyro sample.
// Its IRQ (the only one) queues the previous DMA burst read and starts the next, so a sample is
// delivered one period (150 us) late. The queue absorbs main-loop stalls (32 samples = 4.8 ms).

#include <cstdint>

#include "hal/Hal.hpp"
#include "interfaces/Devices.hpp"

namespace bike {

class Ism330dhcx final : public IImu {
public:
    enum class AccelRange : uint8_t { G2 = 0, G16 = 1, G4 = 2, G8 = 3 };
    enum class GyroRange : uint8_t { Dps250 = 0, Dps500 = 1, Dps1000 = 2, Dps2000 = 3 };

    struct Config {
        AccelRange accel = AccelRange::G4;
        GyroRange gyro = GyroRange::Dps1000;
        // Body axis i (x forward, y left, z up) = sign * sensor axis |axis[i]| (1..3).
        int8_t axis[3] = {1, 2, 3};
        uint32_t stampTicksPerUs = 80;  // onDataReady() stamp rate (DWT cycles at 80 MHz)
    };

    Ism330dhcx(hal::ISpi& spi, hal::IOutputPin& cs, hal::IClock& clock, const Config& cfg)
        : spi_(spi), cs_(cs), clock_(clock), cfg_(cfg) {}

    bool init() override;
    bool read(ImuSample& out) override;

    void onDataReady(uint32_t stamp);  // IRQ context

    bool ok() const { return ok_; }
    uint8_t whoAmI() const { return whoAmI_; }
    uint32_t samples() const { return samples_; }
    uint32_t overruns() const { return overruns_; }    // data ready while the previous read was running
    uint32_t overflows() const { return overflows_; }  // queue full: main loop stalled > 4.8 ms
    uint32_t gaps() const { return gaps_; }            // > 1.5 ODR periods between queued samples

private:
    static constexpr uint8_t kBurstStart = 0x22;  // OUTX_L_G .. OUTZ_H_A
    static constexpr uint8_t kBurstLen = 12;
    static constexpr uint8_t kQueueSize = 32;

    struct Raw {
        uint32_t data[kBurstLen / 4];
        uint32_t stamp;
    };

    bool writeReg(uint8_t reg, uint8_t value);
    uint8_t readReg(uint8_t reg);

    hal::ISpi& spi_;
    hal::IOutputPin& cs_;
    hal::IClock& clock_;
    Config cfg_;

    volatile bool ok_ = false;
    volatile bool busy_ = false;
    bool started_ = false;
    uint8_t whoAmI_ = 0;
    float accelScale_ = 0.0f;
    float gyroScale_ = 0.0f;
    uint32_t pendingStamp_ = 0;
    uint8_t tx_[1 + kBurstLen] = {};
    // Burst data lands word-aligned at rxBuf_[4], after 3 bytes of padding and the address byte.
    alignas(4) uint8_t rxBuf_[4 + kBurstLen] = {};
    uint8_t* const rx_ = &rxBuf_[3];

    Raw queue_[kQueueSize] = {};
    volatile uint8_t head_ = 0;
    volatile uint8_t tail_ = 0;

    uint32_t lastStamp_ = 0;
    uint32_t stampRemainder_ = 0;
    uint32_t timeUs_ = 0;
    uint32_t samples_ = 0;
    volatile uint32_t overruns_ = 0;
    volatile uint32_t overflows_ = 0;
    uint32_t gaps_ = 0;
};

}  // namespace bike
