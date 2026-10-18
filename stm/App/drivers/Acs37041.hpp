#pragma once
// ACS37041 (+/-30 A, 3.3 V). Output is not ratiometric (1.65 V + 44 mV/A), so the input must give
// absolute millivolts (AdcInput does, via VREFINT).

#include "interfaces/Devices.hpp"

namespace bike {

class Acs37041 final : public ICurrentSensor {
public:
    static constexpr float kZeroMv = 1650.0f;
    static constexpr float kMvPerAmp = 44.0f;

    explicit Acs37041(IAnalogInput& in) : in_(in) {}

    bool init() override { return true; }

    bool read(float& amps) override {
        float mv;
        if (!in_.readMillivolts(mv)) return false;
        amps = (mv - kZeroMv - zeroTrimMv_) / kMvPerAmp;
        return true;
    }

    // Measured output at 0 A minus 1.65 V.
    void setZeroTrimMv(float mv) { zeroTrimMv_ = mv; }

private:
    IAnalogInput& in_;
    float zeroTrimMv_ = 0.0f;
};

}  // namespace bike
