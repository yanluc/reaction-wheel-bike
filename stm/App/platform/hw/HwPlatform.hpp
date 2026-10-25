#pragma once
// TODO: motor PWM/DIR is still a Null* placeholder.

#include "drivers/Acs37041.hpp"
#include "drivers/Ism330dhcx.hpp"
#include "drivers/NullDevices.hpp"
#include "drivers/PwmDirMotor.hpp"
#include "drivers/Vl53l1x.hpp"
#include "interfaces/Platform.hpp"
#include "platform/stm32/AdcDma.hpp"
#include "platform/stm32/Stm32Hal.hpp"

namespace bike {

class HwPlatform final : public IPlatform {
public:
    // Arduino pin wired to the current sensor's VOUT.
    static constexpr stm32::AdcDma::Channel kCurrentChannel = stm32::AdcDma::kA0;
    // Both ToF sensors (one per I2C bus, both at 0x29) are triggered together (one-shot) so a
    // left/right pair describes one moment.
    static constexpr uint32_t kTofPeriodUs = 25000;  // 20 ms timing budget + read-out

    struct Deps {
        hal::IClock& clock;
        stm32::AdcDma& adc;
        hal::ISpi& imuSpi;
        hal::IOutputPin& imuCs;
        stm32::DmaI2c& i2cLeft;
        hal::IInputPin& tofLeftGpio1;
        stm32::DmaI2c& i2cRight;
        hal::IInputPin& tofRightGpio1;
    };

    HwPlatform(const Deps& d, const Ism330dhcx::Config& imuCfg)
        : d_(d),
          imu_(d.imuSpi, d.imuCs, d.clock, imuCfg),
          distLeft_(d.i2cLeft, d.tofLeftGpio1, d.clock, Vl53l1x::Config{}),
          distRight_(d.i2cRight, d.tofRightGpio1, d.clock, Vl53l1x::Config{}),
          currentIn_(d.adc, kCurrentChannel),
          current_(currentIn_),
          motor_(pwm_, dir_) {}

    const char* name() const override { return "HW"; }
    bool isSimulation() const override { return false; }
    bool init() override;
    void poll() override;

    IImu& imu() override { return imu_; }
    Ism330dhcx& imuDriver() { return imu_; }
    IDistanceSensor& distanceLeft() override { return distLeft_; }
    IDistanceSensor& distanceRight() override { return distRight_; }
    ICurrentSensor& current() override { return current_; }
    IMotor& motor() override { return motor_; }
    int diagnostics(uint8_t line, char* out, int size) override;

private:
    Deps d_;
    Ism330dhcx imu_;
    Vl53l1x distLeft_;
    Vl53l1x distRight_;
    stm32::AdcInput currentIn_;
    Acs37041 current_;
    NullPwmOutput pwm_;
    NullPin dir_;
    PwmDirMotor motor_;
    uint32_t lastImuSamples_ = 0;
    uint32_t lastTofTriggerUs_ = 0;
};

}  // namespace bike
