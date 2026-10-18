#pragma once

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
        if (command > 1.0f) command = 1.0f;
        if (command < -1.0f) command = -1.0f;
        command_ = enabled_ ? command : 0.0f;
        const bool forward = (command_ >= 0.0f) != invert_;
        dir_.write(forward);
        pwm_.setDuty(command_ >= 0.0f ? command_ : -command_);
    }

    float command() const override { return command_; }

private:
    IPwmOutput& pwm_;
    hal::IOutputPin& dir_;
    bool invert_;
    bool enabled_ = false;
    float command_ = 0.0f;
};

}  // namespace bike
