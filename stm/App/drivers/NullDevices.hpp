#pragma once
// Placeholders for undecided parts: they report no data, so the supervisor keeps the motor disarmed.

#include "hal/Hal.hpp"
#include "interfaces/Devices.hpp"

namespace bike {

class NullImu final : public IImu {
public:
    bool init() override { return false; }
    bool read(ImuSample&) override { return false; }
};

class NullDistanceSensor final : public IDistanceSensor {
public:
    bool init() override { return false; }
    bool read(float&, uint32_t&) override { return false; }
};

class NullCurrentSensor final : public ICurrentSensor {
public:
    bool init() override { return false; }
    bool read(float&) override { return false; }
};

class NullPwmOutput final : public IPwmOutput {
public:
    void setDuty(float) override {}
};

class NullPin final : public hal::IOutputPin {
public:
    void write(bool) override {}
};

}  // namespace bike
