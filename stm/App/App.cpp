#include "App.hpp"

#include <algorithm>
#include <cstring>

namespace bike {
namespace {

int16_t toMilli(float v) { return static_cast<int16_t>(std::clamp(v * 1000.0f, -32768.0f, 32767.0f)); }

uint16_t toMm(float meters) { return static_cast<uint16_t>(std::clamp(meters * 1000.0f, 0.0f, 65535.0f)); }

struct Periodic {
    uint32_t period;
    uint32_t next;
    bool due(uint32_t now) {
        if (static_cast<int32_t>(now - next) < 0) return false;
        next += period;
        if (static_cast<int32_t>(now - next) >= 0) next = now + period;  // skip missed slots
        return true;
    }
};

}  // namespace

void App::init(const nrf24::Config& radioCfg) {
    d_.serial.start();
    d_.serial.addListener(proto::MsgType::Command, *this);
    d_.serial.log("bike fw: platform=%s, SYSCLK=%lu MHz", d_.platform.name(), SystemCoreClock / 1000000u);

    if (!d_.platform.init()) {
        d_.serial.log("platform init FAILED");
    }

    if (d_.radio.init(radioCfg)) {
        d_.serial.log("nRF24 OK: ch=%u, PTX", radioCfg.channel);
    } else {
        d_.serial.log("nRF24 NOT RESPONDING - check wiring/3V3 supply");
    }

    d_.estimator.reset();
    selectController(0);
}

void App::selectController(uint8_t index) {
    if (index >= d_.controllerCount) {
        return;
    }
    controllerIndex_ = index;
    controller_ = d_.controllers[index];
    controller_->reset();
}

void App::onCommand(const proto::Command& cmd) {
    lastCmdId_ = cmd.id;
    lastCmdStamp_ = cmd.stamp;
    switch (cmd.cmd) {
        case proto::CommandId::Arm:
            d_.supervisor.requestArm();
            break;
        case proto::CommandId::Disarm:
            d_.supervisor.disarm();
            break;
        case proto::CommandId::SetController:
            d_.supervisor.disarm();  // never switch control law while the wheel spins
            selectController(static_cast<uint8_t>(cmd.arg));
            break;
        case proto::CommandId::Calibrate:
            d_.supervisor.disarm();
            d_.estimator.reset();
            break;
        case proto::CommandId::SetParam:
            controller_->setParam(cmd.param, static_cast<float>(cmd.arg) / 1000.0f);
            break;
        default:  // Ping/Nop: only echoed in telemetry
            break;
    }
}

void App::onFrame(const uint8_t* msg, size_t len) {
    if (len != sizeof(proto::Command)) {
        return;
    }
    proto::Command cmd;
    std::memcpy(&cmd, msg, sizeof(cmd));
    onCommand(cmd);
}

void App::imuStep(uint32_t now) {
    ImuSample imu;
    while (d_.platform.imu().read(imu)) {
        float dt = haveImu_ ? static_cast<float>(imu.timeUs - lastImuTimeUs_) * 1e-6f : 0.0f;
        if (dt > kMaxImuDt) dt = kMaxImuDt;
        d_.estimator.update(imu, dt, state_);
        lastImuTimeUs_ = imu.timeUs;
        lastImuUs_ = now;
        haveImu_ = true;
        ++imuSamples_;
    }
    state_.imuValid = haveImu_ && now - lastImuUs_ < kImuTimeoutUs;
}

void App::distanceStep() {
    float v;
    if (d_.platform.distanceLeft().read(v, leftUs_)) {
        state_.distLeft = v;
        newLeft_ = true;
    }
    if (d_.platform.distanceRight().read(v, rightUs_)) {
        state_.distRight = v;
        newRight_ = true;
    }
    if (!newLeft_ || !newRight_) {
        return;
    }
    // Only a pair measured at the same moment gives the roll; otherwise keep the newer one and wait.
    const int32_t skew = static_cast<int32_t>(leftUs_ - rightUs_);
    if (skew > kMaxTofSkewUs) {
        newRight_ = false;
    } else if (skew < -kMaxTofSkewUs) {
        newLeft_ = false;
    } else {
        newLeft_ = newRight_ = false;
        d_.estimator.updateDistances(state_.distLeft, state_.distRight, state_);
    }
}

void App::controlStep(float dt) {
    IMotor& motor = d_.platform.motor();
    const bool run = d_.supervisor.update(state_, d_.radio.linkUp());
    motor.setEnabled(run);
    if (run) {
        motor.setCommand(controller_->update(state_, dt));
    }
}

void App::auxStep() {
    float v;
    if (d_.platform.current().read(v)) state_.current = v;
}

proto::Telemetry App::buildTelemetry(uint16_t seq) {
    proto::Telemetry t{};
    t.type = proto::MsgType::Telemetry;
    t.seq = seq;
    t.uptimeMs = d_.clock.millis();
    t.controller = controllerIndex_;
    t.flags = (d_.supervisor.armed() ? proto::kFlagArmed : 0) | (state_.imuValid ? proto::kFlagImuOk : 0) |
              (d_.radio.linkUp() ? proto::kFlagRadioLink : 0) |
              (d_.platform.isSimulation() ? proto::kFlagSimPlatform : 0) |
              (d_.supervisor.failsafe() ? proto::kFlagFailsafe : 0) |
              (state_.estimatorReady ? proto::kFlagEstimatorReady : 0);
    t.rollMrad = toMilli(state_.roll);
    t.rollRateMradS = toMilli(state_.rollRate);
    t.motorPermille = toMilli(d_.platform.motor().command());
    t.currentMa = toMilli(state_.current);
    t.distLeftMm = toMm(state_.distLeft);
    t.distRightMm = toMm(state_.distRight);
    t.lastCmdId = lastCmdId_;
    t.lastCmdStamp = lastCmdStamp_;
    t.txFailures = static_cast<uint16_t>(d_.radio.stats().telemetryFailed);
    return t;
}

// uint32_t is unsigned long on arm-none-eabi: %lu.
bool App::logStatsLine(uint8_t line) {
    if (line == 0) {
        const RadioLink::Stats& rs = d_.radio.stats();
        d_.serial.log("radio tlm sent=%lu acked=%lu fail=%lu cmds=%lu link=%d", rs.telemetrySent, rs.telemetryAcked,
                      rs.telemetryFailed, rs.commands, d_.radio.linkUp());
        return true;
    }
    if (line == 1) {
        d_.serial.log("loops/s=%lu maxgap=%luus imu/s=%lu roll=%dmrad armed=%d failsafe=%d ctrl=%s uart rxerr=%lu drop=%lu",
                      snap_.loops, snap_.maxLoopGapUs, snap_.imuSamples, static_cast<int>(state_.roll * 1000.0f),
                      d_.supervisor.armed(), d_.supervisor.failsafe(), controller_->name(), d_.serial.rxErrors(),
                      d_.serial.droppedTx());
        return true;
    }
    char diag[proto::kMaxFramePayload - 1];
    if (line == 2) {
        if (d_.estimator.status(diag, sizeof(diag)) > 0) d_.serial.log("%s", diag);
        return true;
    }
    if (d_.platform.diagnostics(static_cast<uint8_t>(line - 3), diag, sizeof(diag)) <= 0) {
        return false;
    }
    d_.serial.log("%s", diag);
    return true;
}

void App::run() {
    uint32_t now = d_.clock.micros();
    Periodic control{kControlPeriodUs, now};
    Periodic aux{kAuxPeriodUs, now};
    Periodic radio{kRadioPeriodUs, now};
    Periodic serialTlm{kSerialTelemetryPeriodUs, now};
    Periodic stats{kStatsPeriodUs, now + kStatsPeriodUs};
    uint32_t lastControlUs = now;
    uint32_t lastLoopUs = now;
    uint32_t blink = 0;

    while (true) {
        now = d_.clock.micros();
        ++loops_;
        if (now - lastLoopUs > maxLoopGapUs_) maxLoopGapUs_ = now - lastLoopUs;
        lastLoopUs = now;

        d_.serial.poll();
        d_.platform.poll();
        d_.radio.poll();
        imuStep(now);
        distanceStep();

        if (control.due(now)) {
            controlStep(static_cast<float>(now - lastControlUs) * 1e-6f);
            lastControlUs = now;
        }
        if (aux.due(now)) {
            auxStep();
        }
        if (radio.due(now)) {
            const proto::Telemetry t = buildTelemetry(radioSeq_);
            if (d_.radio.sendTelemetry(t)) {
                ++radioSeq_;
            }
        }
        if (serialTlm.due(now)) {
            const proto::Telemetry t = buildTelemetry(serialSeq_);
            if (d_.serial.send(&t, sizeof(t))) {
                ++serialSeq_;
            }

            d_.ledLink.write(d_.radio.linkUp());
            ++blink;
            d_.ledArmed.write(d_.supervisor.armed() || (d_.supervisor.failsafe() && (blink & 4)));
        }
        if (stats.due(now)) {
            snap_ = {loops_, maxLoopGapUs_, imuSamples_};
            loops_ = 0;
            maxLoopGapUs_ = 0;
            imuSamples_ = 0;
            statsLine_ = 1;
        } else if (statsLine_ != 0) {
            // One line per iteration: a whole report at once stalls the loop for ~1 ms.
            statsLine_ = logStatsLine(statsLine_ - 1) ? statsLine_ + 1 : 0;
        }
    }
}

}  // namespace bike
