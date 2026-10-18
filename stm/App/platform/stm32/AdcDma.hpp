#pragma once
// ADC1 scan (stm.ioc: A0/PA3, A1/PC0, A2/PC3, VREFINT; oversampling x256 -> 16 bit) by circular DMA.
// Millivolts use VDDA measured via VREFINT: needed for non-ratiometric sensors (ACS37041).

#include <cstdint>

#include "interfaces/Devices.hpp"
#include "main.h"

namespace bike::stm32 {

class AdcDma {
public:
    enum Channel : uint8_t { kA0 = 0, kA1 = 1, kA2 = 2, kVrefint = 3, kCount = 4 };

    explicit AdcDma(ADC_HandleTypeDef* h) : h_(h) {}

    bool start();
    bool running() const { return running_; }
    float vddaMv() const;
    float millivolts(Channel ch) const;

private:
    ADC_HandleTypeDef* h_;
    bool running_ = false;
    volatile uint16_t raw_[kCount] = {};  // written by DMA, rank order
};

class AdcInput final : public IAnalogInput {
public:
    AdcInput(const AdcDma& adc, AdcDma::Channel ch) : adc_(adc), ch_(ch) {}
    bool readMillivolts(float& mv) override {
        if (!adc_.running()) return false;
        mv = adc_.millivolts(ch_);
        return true;
    }

private:
    const AdcDma& adc_;
    AdcDma::Channel ch_;
};

}  // namespace bike::stm32
