#pragma once

#include <cmath>

#include "interfaces/Control.hpp"

namespace bike {

class ComplementaryEstimator final : public IEstimator {
public:
    explicit ComplementaryEstimator(float alpha = 0.98f) : alpha_(alpha) {}

    void reset() override { initialized_ = false; }

    void update(const ImuSample& imu, float dt, BikeState& state) override {
        const float rollAcc = std::atan2(imu.accel[1], imu.accel[2]);
        const float rate = imu.gyro[0];
        if (!initialized_) {
            state.roll = rollAcc;
            initialized_ = true;
        } else {
            state.roll = alpha_ * (state.roll + rate * dt) + (1.0f - alpha_) * rollAcc;
        }
        state.rollRate = rate;
    }

private:
    float alpha_;
    bool initialized_ = false;
};

}  // namespace bike
