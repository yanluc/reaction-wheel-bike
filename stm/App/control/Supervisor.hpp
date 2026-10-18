#pragma once

#include <cmath>

#include "interfaces/Control.hpp"

namespace bike {

class Supervisor {
public:
    struct Limits {
        float maxRoll = 0.6f;  // rad
    };

    explicit Supervisor(const Limits& limits) : limits_(limits) {}

    void requestArm() { armRequested_ = true; }
    void disarm() {
        armRequested_ = false;
        armed_ = false;
    }

    // Returns true if the motor may run this cycle.
    bool update(const BikeState& s, bool radioLinkUp) {
        failsafe_ = !s.imuValid || std::fabs(s.roll) > limits_.maxRoll || !radioLinkUp;
        if (failsafe_) {
            disarm();
        } else if (armRequested_) {
            armed_ = true;
        }
        return armed_;
    }

    bool armed() const { return armed_; }
    bool failsafe() const { return failsafe_; }

private:
    Limits limits_;
    bool armRequested_ = false;
    bool armed_ = false;
    bool failsafe_ = false;
};

}  // namespace bike
