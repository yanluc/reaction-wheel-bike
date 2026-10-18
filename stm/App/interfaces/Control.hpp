#pragma once

#include <cstdint>

#include "interfaces/Devices.hpp"

namespace bike {

struct BikeState {
    float roll = 0.0f;      // rad, right-hand about x forward: positive = leaning right
    float rollRate = 0.0f;  // rad/s
    float distLeft = 0.0f;  // m
    float distRight = 0.0f; // m
    float current = 0.0f;   // A
    bool imuValid = false;
    bool estimatorReady = false;  // false while calibrating: arming is blocked
};

class IEstimator {
public:
    virtual void reset() = 0;
    virtual void update(const ImuSample& imu, float dt, BikeState& state) = 0;
    virtual void updateDistances(float left, float right, BikeState& state) { (void)left; (void)right; (void)state; }
    // One log line (e.g. calibration state); returns chars written.
    virtual int status(char* out, int size) const { (void)out; (void)size; return 0; }

protected:
    ~IEstimator() = default;
};

class IController {
public:
    virtual const char* name() const = 0;
    virtual void reset() = 0;
    // Motor command -1..1.
    virtual float update(const BikeState& state, float dt) = 0;
    // SetParam from the radio; false for an unknown index.
    virtual bool setParam(uint8_t index, float value) { (void)index; (void)value; return false; }

protected:
    ~IController() = default;
};

}  // namespace bike
