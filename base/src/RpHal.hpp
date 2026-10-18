#pragma once
// RP2040 adapters for the hal:: interfaces (common/include/hal/Hal.hpp).

#include <cstdint>

#include "hal/Hal.hpp"
#include "hardware/spi.h"

namespace base {

class Pin final : public hal::IOutputPin {
public:
    // Also nRF24 CSN: PL022 hardware CSn toggles between frames in SPI mode 0.
    Pin(uint gpio, bool initial);
    void write(bool high) override;

private:
    uint gpio_;
};

class InputPin final : public hal::IInputPin {
public:
    explicit InputPin(uint gpio);  // with pull-up: reads high if the module is unplugged
    bool read() override;

private:
    uint gpio_;
};

class DmaSpi final : public hal::ISpi {
public:
    DmaSpi(spi_inst_t* spi, uint baudrate, uint sck, uint mosi, uint miso);
    bool transfer(const uint8_t* tx, uint8_t* rx, size_t len) override;

private:
    spi_inst_t* spi_;
    int txChan_;
    int rxChan_;
};

class Clock final : public hal::IClock {
public:
    uint32_t millis() override;
    uint32_t micros() override;
    void delayUs(uint32_t us) override;
};

}  // namespace base
