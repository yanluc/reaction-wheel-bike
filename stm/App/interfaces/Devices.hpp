#pragma once
// No heap: interfaces have protected non-virtual destructors.

#include <cstdint>

namespace bike {

struct ImuSample {
    float accel[3];  // m/s^2, body frame (x forward, y left, z up)
    float gyro[3];   // rad/s
    uint32_t timeUs;
};

class IImu {
public:
    virtual bool init() = 0;
    virtual bool read(ImuSample& out) = 0;  // false if no fresh/valid sample

protected:
    ~IImu() = default;
};

class IDistanceSensor {
public:
    virtual bool init() = 0;
    virtual bool read(float& meters) = 0;

protected:
    ~IDistanceSensor() = default;
};

class ICurrentSensor {
public:
    virtual bool init() = 0;
    virtual bool read(float& amps) = 0;

protected:
    ~ICurrentSensor() = default;
};

class IMotor {
public:
    virtual bool init() = 0;
    virtual void setEnabled(bool enabled) = 0;
    virtual void setCommand(float command) = 0;  // -1..1, sign = direction
    virtual float command() const = 0;

protected:
    ~IMotor() = default;
};

class IAnalogInput {
public:
    virtual bool readMillivolts(float& mv) = 0;

protected:
    ~IAnalogInput() = default;
};

class IPwmOutput {
public:
    virtual void setDuty(float duty) = 0;  // 0..1

protected:
    ~IPwmOutput() = default;
};

}  // namespace bike
