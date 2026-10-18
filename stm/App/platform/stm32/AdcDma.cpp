#include "platform/stm32/AdcDma.hpp"

namespace bike::stm32 {
namespace {
constexpr float kFullScale = 4095.0f * 16.0f;  // 12-bit, oversampling x256 >> 4 = 16x
}

bool AdcDma::start() {
    if (HAL_ADCEx_Calibration_Start(h_, ADC_SINGLE_ENDED) != HAL_OK) {
        return false;
    }
    running_ = HAL_ADC_Start_DMA(h_, reinterpret_cast<uint32_t*>(const_cast<uint16_t*>(raw_)), kCount) == HAL_OK;
    return running_;
}

float AdcDma::vddaMv() const {
    const uint16_t vref = raw_[kVrefint];
    if (vref == 0) return 0.0f;
    // VREFINT_CAL: factory 12-bit VREFINT reading at VDDA = 3.0 V.
    return static_cast<float>(VREFINT_CAL_VREF) * (*VREFINT_CAL_ADDR) * 16.0f / vref;
}

float AdcDma::millivolts(Channel ch) const {
    if (!running_) return 0.0f;
    return raw_[ch] * vddaMv() / kFullScale;
}

}  // namespace bike::stm32
