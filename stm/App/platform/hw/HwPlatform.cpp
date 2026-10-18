#include "platform/hw/HwPlatform.hpp"

#include <cstdio>

namespace bike {

bool HwPlatform::init() {
    // Both ToF sensors boot at 0x29: hold both in reset, then bring them up one by one with new addresses.
    distLeft_.shutdown();
    distRight_.shutdown();
    d_.clock.delayUs(5000);
    const bool imuOk = imu_.init();
    const bool leftOk = distLeft_.init();
    const bool rightOk = distRight_.init();
    const bool adcOk = d_.adc.start();
    current_.init();
    return motor_.init() && adcOk && imuOk && leftOk && rightOk;
}

int HwPlatform::diagnostics(uint8_t line, char* out, int size) {
    using A = stm32::AdcDma;
    switch (line) {
        case 0: {
            const uint32_t n = imu_.samples();
            const int len = std::snprintf(out, size, "IMU ok=%d who=0x%02X samples/s=%lu gaps=%lu overruns=%lu overflows=%lu",
                                          imu_.ok() ? 1 : 0, imu_.whoAmI(), static_cast<unsigned long>(n - lastImuSamples_),
                                          static_cast<unsigned long>(imu_.gaps()), static_cast<unsigned long>(imu_.overruns()),
                                          static_cast<unsigned long>(imu_.overflows()));
            lastImuSamples_ = n;
            return len;
        }
        case 1:
            return std::snprintf(
                out, size, "ToF L ok=%d %umm st=%u n=%lu bad=%lu | R ok=%d %umm st=%u n=%lu bad=%lu | i2c err=%lu",
                distLeft_.ok() ? 1 : 0, distLeft_.lastMm(), distLeft_.lastStatus(),
                static_cast<unsigned long>(distLeft_.measurements()), static_cast<unsigned long>(distLeft_.invalid()),
                distRight_.ok() ? 1 : 0, distRight_.lastMm(), distRight_.lastStatus(),
                static_cast<unsigned long>(distRight_.measurements()), static_cast<unsigned long>(distRight_.invalid()),
                static_cast<unsigned long>(d_.i2c.errors()));
        case 2: {
            if (!d_.adc.running()) {
                return std::snprintf(out, size, "ADC not running");
            }
            float amps = 0.0f;
            current_.read(amps);
            return std::snprintf(out, size, "ADC mV: A0=%d A1=%d A2=%d VDDA=%d | current(A%d)=%d mA",
                                 static_cast<int>(d_.adc.millivolts(A::kA0)), static_cast<int>(d_.adc.millivolts(A::kA1)),
                                 static_cast<int>(d_.adc.millivolts(A::kA2)), static_cast<int>(d_.adc.vddaMv()),
                                 static_cast<int>(kCurrentChannel), static_cast<int>(amps * 1000.0f));
        }
        default:
            return 0;
    }
}

}  // namespace bike
