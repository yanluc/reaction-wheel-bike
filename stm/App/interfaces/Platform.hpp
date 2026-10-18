#pragma once

#include <cstdint>

#include "interfaces/Devices.hpp"

namespace bike {

class IPlatform {
public:
    virtual const char* name() const = 0;
    virtual bool isSimulation() const = 0;
    virtual bool init() = 0;
    virtual void poll() = 0;  // called every main-loop iteration

    virtual IImu& imu() = 0;
    virtual IDistanceSensor& distanceLeft() = 0;
    virtual IDistanceSensor& distanceRight() = 0;
    virtual ICurrentSensor& current() = 0;
    virtual IMotor& motor() = 0;

    // Logged once per second, lines 0, 1, ... until one returns 0.
    virtual int diagnostics(uint8_t line, char* out, int size) { (void)line; (void)out; (void)size; return 0; }

protected:
    ~IPlatform() = default;
};

}  // namespace bike
