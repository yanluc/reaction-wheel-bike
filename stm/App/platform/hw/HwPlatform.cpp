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

void HwPlatform::poll() {
    distLeft_.poll();
    distRight_.poll();
    const uint32_t now = d_.clock.micros();
    if (!triggerLeft_ && !triggerRight_ && now - lastTofTriggerUs_ >= kTofPeriodUs && distLeft_.idle() &&
        distRight_.idle()) {
        lastTofTriggerUs_ = now;
        triggerLeft_ = distLeft_.ok();
        triggerRight_ = distRight_.ok();
    }
    // Back to back: the second start follows the first by one short I2C write (~0.1 ms).
    if (triggerLeft_ && distLeft_.trigger()) triggerLeft_ = false;
    if (triggerRight_ && distRight_.trigger()) triggerRight_ = false;
}

// uint32_t is unsigned long on arm-none-eabi: %lu.
int HwPlatform::diagnostics(uint8_t line, char* out, int size) {
    using A = stm32::AdcDma;
    const Vl53l1x& l = distLeft_;
    const Vl53l1x& r = distRight_;
    switch (line) {
        case 0: {
            const uint32_t n = imu_.samples();
            const int len =
                std::snprintf(out, size, "IMU ok=%d who=0x%02X samples/s=%lu gaps=%lu overruns=%lu overflows=%lu",
                              imu_.ok(), imu_.whoAmI(), n - lastImuSamples_, imu_.gaps(), imu_.overruns(),
                              imu_.overflows());
            lastImuSamples_ = n;
            return len;
        }
        case 1:
            return std::snprintf(
                out, size, "ToF L ok=%d %umm st=%u n=%lu bad=%lu | R ok=%d %umm st=%u n=%lu bad=%lu | i2c err=%lu bus=%lu tmo=%lu",
                l.ok(), l.lastMm(), l.lastStatus(), l.measurements(), l.invalid(), r.ok(), r.lastMm(),
                r.lastStatus(), r.measurements(), r.invalid(), d_.i2c.errors(), l.busErrors() + r.busErrors(),
                l.timeouts() + r.timeouts());
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
