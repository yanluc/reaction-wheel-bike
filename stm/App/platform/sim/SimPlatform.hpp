#pragma once
// Devices backed by a PC simulator over the serial link: each proto::SimSensors is answered with
// proto::SimActuators (lock-step). SimPlatform::onFrame() fills the public fields.

#include <utility>

#include "drivers/NullDevices.hpp"
#include "drivers/PwmDirMotor.hpp"
#include "interfaces/Platform.hpp"
#include "platform/stm32/SerialLink.hpp"
#include "proto/Messages.hpp"

namespace bike {

struct SimImu final : IImu {
    bool init() override { return true; }
    bool read(ImuSample& out) override {
        out = sample;
        return std::exchange(fresh, false);
    }

    ImuSample sample{};
    bool fresh = false;
};

struct SimDistanceSensor final : IDistanceSensor {
    bool init() override { return true; }
    bool read(float& meters, uint32_t& t) override {
        meters = value;
        t = timeUs;
        return std::exchange(fresh, false);
    }

    float value = 0.0f;
    uint32_t timeUs = 0;
    bool fresh = false;
};

struct SimCurrentSensor final : ICurrentSensor {
    bool init() override { return true; }
    bool read(float& amps) override {
        amps = value;
        return valid;
    }

    float value = 0.0f;
    bool valid = false;
};

class SimPlatform final : public IPlatform, public stm32::IFrameListener {
public:
    explicit SimPlatform(stm32::SerialLink& link) : link_(link), motor_(pwm_, dir_) {}

    const char* name() const override { return "SIM"; }
    bool isSimulation() const override { return true; }
    bool init() override;
    void poll() override {}

    IImu& imu() override { return imu_; }
    IDistanceSensor& distanceLeft() override { return distLeft_; }
    IDistanceSensor& distanceRight() override { return distRight_; }
    ICurrentSensor& current() override { return current_; }
    IMotor& motor() override { return motor_; }

    void onFrame(const uint8_t* msg, size_t len) override;

private:
    stm32::SerialLink& link_;
    SimImu imu_;
    SimDistanceSensor distLeft_;
    SimDistanceSensor distRight_;
    SimCurrentSensor current_;
    NullPwmOutput pwm_;  // the command goes back to the simulator, not to a pin
    NullPin dir_;
    PwmDirMotor motor_;
};

}  // namespace bike
