#pragma once
// Minimal HAL shared by the STM32 and RP2040 firmware. Destructors are protected and non-virtual:
// a virtual one would pull operator delete (and the heap) into the firmware.

#include <cstddef>
#include <cstdint>

// STM32: hot paths run from RAM (flash has 4 wait states at 80 MHz and only a 1 KB I-cache).
#if defined(STM32L4A6xx)
#define HAL_RAM_FUNC __attribute__((section(".RamFunc")))
#else
#define HAL_RAM_FUNC
#endif

namespace hal {

class ISpi {
public:
    // tx or rx may be nullptr (sends 0xFF / discards).
    virtual bool transfer(const uint8_t* tx, uint8_t* rx, size_t len) = 0;

    // Non-blocking (e.g. DMA); buffers must stay valid until !busy().
    virtual bool startTransfer(const uint8_t* tx, uint8_t* rx, size_t len) { return transfer(tx, rx, len); }
    virtual bool busy() const { return false; }

protected:
    ~ISpi() = default;
};

class IOutputPin {
public:
    virtual void write(bool high) = 0;

protected:
    ~IOutputPin() = default;
};

class IInputPin {
public:
    virtual bool read() = 0;

protected:
    ~IInputPin() = default;
};

class IClock {
public:
    virtual uint32_t millis() = 0;
    virtual uint32_t micros() = 0;
    virtual void delayUs(uint32_t us) = 0;

protected:
    ~IClock() = default;
};

}  // namespace hal
