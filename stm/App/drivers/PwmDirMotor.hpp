#pragma once

#include <algorithm>
#include <cmath>

#include "hal/Hal.hpp"
#include "interfaces/Devices.hpp"

namespace bike {

class PwmDirMotor final : public IMotor {
public:
    PwmDirMotor(IPwmOutput& pwm, hal::IOutputPin& dir, bool invert = false)
        : pwm_(pwm), dir_(dir), invert_(invert) {}

    bool init() override {
        setEnabled(false);
        return true;
    }

    void setEnabled(bool enabled) override {
        enabled_ = enabled;
        if (!enabled_) {
            command_ = 0.0f;
            pwm_.setDuty(0.0f);
        }
    }

    void setCommand(float command) override {
        command_ = enabled_ ? std::clamp(command, -1.0f, 1.0f) : 0.0f;
        dir_.write((command_ >= 0.0f) != invert_);
        pwm_.setDuty(std::fabs(command_));
    }

    float command() const override { return command_; }
    bool enabled() const { return enabled_; }

private:
    IPwmOutput& pwm_;
    hal::IOutputPin& dir_;
    bool invert_;
    bool enabled_ = false;
    float command_ = 0.0f;
};

}  // namespace bike
