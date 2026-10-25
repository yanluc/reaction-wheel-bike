#pragma once

#include <algorithm>

#include "interfaces/Control.hpp"

namespace bike {

class NullController final : public IController {
public:
    const char* name() const override { return "null"; }
    void reset() override {}
    float update(const BikeState&, float) override { return 0.0f; }
};

class PdController final : public IController {
public:
    PdController(float kp, float kd) : kp_(kp), kd_(kd) {}

    const char* name() const override { return "pd"; }
    void reset() override {}

    float update(const BikeState& s, float) override {
        return std::clamp(-(kp_ * s.roll + kd_ * s.rollRate), -1.0f, 1.0f);
    }

    bool setParam(uint8_t index, float value) override {
        switch (index) {
            case 0: kp_ = value; return true;
            case 1: kd_ = value; return true;
            default: return false;
        }
    }

private:
    float kp_;
    float kd_;
};

}  // namespace bike
