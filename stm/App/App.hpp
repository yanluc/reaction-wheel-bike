#pragma once

#include <cstdint>

#include "comms/RadioStream.hpp"
#include "control/Supervisor.hpp"
#include "interfaces/Control.hpp"
#include "interfaces/Platform.hpp"
#include "platform/stm32/SerialLink.hpp"

namespace bike {

class App final : public ICommandSink, public stm32::IFrameListener {
public:
    struct Deps {
        hal::IClock& clock;
        stm32::SerialLink& serial;
        IPlatform& platform;
        RadioStream& radio;
        IEstimator& estimator;
        IController* const* controllers;
        uint8_t controllerCount;
        Supervisor& supervisor;
        hal::IOutputPin& ledLink;
        hal::IOutputPin& ledArmed;
    };

    explicit App(const Deps& d) : d_(d) {}

    void init(const nrf24::Config& radioCfg);
    [[noreturn]] void run();

    void onCommand(const proto::Command& cmd) override;
    void onFrame(const uint8_t* msg, size_t len) override;

private:
    static constexpr uint32_t kControlPeriodUs = 2000;
    static constexpr uint32_t kAuxPeriodUs = 20000;
    static constexpr uint32_t kRadioPeriodUs = 10000;
    static constexpr uint32_t kSerialTelemetryPeriodUs = 50000;
    static constexpr uint32_t kStatsPeriodUs = 1000000;
    static constexpr uint32_t kImuTimeoutUs = 20000;
    static constexpr float kMaxImuDt = 0.01f;
    static constexpr int32_t kMaxTofSkewUs = 5000;

    void imuStep(uint32_t now);
    void distanceStep();
    void controlStep(float dt);
    void auxStep();
    proto::Telemetry buildTelemetry(uint16_t seq);  // seq per link: per-link loss stats
    void selectController(uint8_t index);
    bool logStatsLine(uint8_t line);

    Deps d_;
    BikeState state_;
    IController* controller_ = nullptr;
    uint8_t controllerIndex_ = 0;
    uint16_t radioSeq_ = 0;
    uint16_t serialSeq_ = 0;
    uint16_t lastCmdId_ = 0;
    uint32_t lastCmdStamp_ = 0;
    uint32_t imuSamples_ = 0;
    uint32_t lastImuTimeUs_ = 0;
    uint32_t lastImuUs_ = 0;
    bool haveImu_ = false;
    bool newLeft_ = false;
    bool newRight_ = false;
    uint32_t leftUs_ = 0;
    uint32_t rightUs_ = 0;
    uint32_t loops_ = 0;
    uint32_t maxLoopGapUs_ = 0;
    struct {
        uint32_t loops;
        uint32_t maxLoopGapUs;
        uint32_t imuSamples;
    } snap_{};
    uint8_t statsLine_ = 0;
};

}  // namespace bike
