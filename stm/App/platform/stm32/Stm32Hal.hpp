#pragma once

#include <cstdint>

#include "hal/Hal.hpp"
#include "main.h"

namespace bike::stm32 {

class Pin final : public hal::IOutputPin {
public:
    Pin(GPIO_TypeDef* port, uint16_t pin) : port_(port), pin_(pin) {}
    void write(bool high) override { port_->BSRR = high ? pin_ : static_cast<uint32_t>(pin_) << 16; }
    void toggle() { HAL_GPIO_TogglePin(port_, pin_); }

private:
    GPIO_TypeDef* port_;
    uint16_t pin_;
};

class InputPin final : public hal::IInputPin {
public:
    InputPin(GPIO_TypeDef* port, uint16_t pin) : port_(port), pin_(pin) {}
    bool read() override { return (port_->IDR & pin_) != 0; }

private:
    GPIO_TypeDef* port_;
    uint16_t pin_;
};

// Per transfer only reloads the CubeMX-configured DMA channels; completion from RX CNDTR, no IRQs.
// transfer() blocks (short ones polled); startTransfer() + busy() is asynchronous.
class DmaSpi final : public hal::ISpi {
public:
    explicit DmaSpi(SPI_HandleTypeDef* h) : h_(h) {}
    bool transfer(const uint8_t* tx, uint8_t* rx, size_t len) override;
    bool startTransfer(const uint8_t* tx, uint8_t* rx, size_t len) override;
    bool busy() const override;

private:
    void finish() const;

    static constexpr size_t kMaxLen = 64;
    static constexpr size_t kPolledMaxLen = 4;
    SPI_HandleTypeDef* h_;
    mutable bool active_ = false;
    bool configured_ = false;
    uint8_t txScratch_[kMaxLen];
    uint8_t rxScratch_[kMaxLen];
};

class DwtClock final : public hal::IClock {
public:
    void init();
    uint32_t millis() override { return HAL_GetTick(); }
    uint32_t micros() override;
    void delayUs(uint32_t us) override;

private:
    uint32_t cyclesPerUs_ = 80;
    uint32_t lastCycles_ = 0;
    uint32_t pendingCycles_ = 0;
    uint32_t us_ = 0;
};

}  // namespace bike::stm32
