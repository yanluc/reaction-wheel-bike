#pragma once
// TODO: IMU (SPI), distance (2x I2C) and motor PWM/DIR are still Null* placeholders.

#include "drivers/Acs37041.hpp"
#include "drivers/NullDevices.hpp"
#include "drivers/PwmDirMotor.hpp"
#include "interfaces/Platform.hpp"
#include "platform/stm32/AdcDma.hpp"

namespace bike {

class HwPlatform final : public IPlatform {
public:
    // Arduino pin wired to the current sensor's VOUT.
    static constexpr stm32::AdcDma::Channel kCurrentChannel = stm32::AdcDma::kA0;

    explicit HwPlatform(stm32::AdcDma& adc)
        : adc_(adc), currentIn_(adc, kCurrentChannel), current_(currentIn_), motor_(pwm_, dir_) {}

    const char* name() const override { return "HW"; }
    bool isSimulation() const override { return false; }
    bool init() override;
    void poll() override {}

    IImu& imu() override { return imu_; }
    IDistanceSensor& distanceLeft() override { return distLeft_; }
    IDistanceSensor& distanceRight() override { return distRight_; }
    ICurrentSensor& current() override { return current_; }
    IMotor& motor() override { return motor_; }
    int diagnostics(char* out, int size) override;

private:
    NullImu imu_;
    NullDistanceSensor distLeft_;
    NullDistanceSensor distRight_;
    stm32::AdcDma& adc_;
    stm32::AdcInput currentIn_;
    Acs37041 current_;
    NullPwmOutput pwm_;
    NullPin dir_;
    PwmDirMotor motor_;
};

}  // namespace bike
