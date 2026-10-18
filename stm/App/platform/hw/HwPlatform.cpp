#include "platform/hw/HwPlatform.hpp"

#include <cstdio>

namespace bike {

bool HwPlatform::init() {
    // Sensor init failures show up in read(); the motor must always come up disabled.
    imu_.init();
    distLeft_.init();
    distRight_.init();
    const bool adcOk = adc_.start();
    current_.init();
    return motor_.init() && adcOk;
}

int HwPlatform::diagnostics(char* out, int size) {
    if (!adc_.running()) {
        return std::snprintf(out, size, "ADC not running");
    }
    using A = stm32::AdcDma;
    float amps = 0.0f;
    current_.read(amps);
    return std::snprintf(out, size, "ADC mV: A0=%d A1=%d A2=%d VDDA=%d | current(A%d)=%d mA",
                         static_cast<int>(adc_.millivolts(A::kA0)), static_cast<int>(adc_.millivolts(A::kA1)),
                         static_cast<int>(adc_.millivolts(A::kA2)), static_cast<int>(adc_.vddaMv()),
                         static_cast<int>(kCurrentChannel), static_cast<int>(amps * 1000.0f));
}

}  // namespace bike
