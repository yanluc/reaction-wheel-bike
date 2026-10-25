#pragma once
// Devices backed by a PC simulator over the serial link: each proto::SimSensors is answered with
// proto::SimActuators (lock-step).

#include <algorithm>

#include "interfaces/Platform.hpp"
#include "platform/stm32/SerialLink.hpp"
#include "proto/Messages.hpp"

namespace bike {

class SimPlatform;

class SimImu final : public IImu {
public:
    bool init() override { return true; }
    bool read(ImuSample& out) override {
        out = sample_;
        const bool fresh = fresh_;
        fresh_ = false;
        return fresh;
    }

private:
    friend class SimPlatform;
    ImuSample sample_{};
    bool fresh_ = false;
};

class SimDistanceSensor final : public IDistanceSensor {
public:
    bool init() override { return true; }
    bool read(float& meters, uint32_t& timeUs) override {
        meters = value_;
        timeUs = timeUs_;
        const bool fresh = valid_;
        valid_ = false;
        return fresh;
    }

private:
    friend class SimPlatform;
    float value_ = 0.0f;
    uint32_t timeUs_ = 0;
    bool valid_ = false;
};

class SimCurrentSensor final : public ICurrentSensor {
public:
    bool init() override { return true; }
    bool read(float& amps) override {
        amps = value_;
        return valid_;
    }

private:
    friend class SimPlatform;
    float value_ = 0.0f;
    bool valid_ = false;
};

class SimMotor final : public IMotor {
public:
    bool init() override { return true; }
    void setEnabled(bool enabled) override {
        enabled_ = enabled;
        if (!enabled) command_ = 0.0f;
    }
    void setCommand(float command) override { command_ = enabled_ ? std::clamp(command, -1.0f, 1.0f) : 0.0f; }
    float command() const override { return command_; }
    bool enabled() const { return enabled_; }

private:
    bool enabled_ = false;
    float command_ = 0.0f;
};

class SimPlatform final : public IPlatform, public stm32::IFrameListener {
public:
    explicit SimPlatform(stm32::SerialLink& link) : link_(link) {}

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
    SimMotor motor_;
};

}  // namespace bike
