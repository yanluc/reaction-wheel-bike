#include "App.hpp"

#include <cstring>

namespace bike {
namespace {

int16_t toMilli(float v) {
    float m = v * 1000.0f;
    if (m > 32767.0f) m = 32767.0f;
    if (m < -32768.0f) m = -32768.0f;
    return static_cast<int16_t>(m);
}

uint16_t toMm(float meters) {
    if (meters < 0.0f) return 0;
    const float mm = meters * 1000.0f;
    return mm > 65535.0f ? 65535 : static_cast<uint16_t>(mm);
}

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
    d_.serial.log("bike fw: platform=%s, SYSCLK=%lu MHz", d_.platform.name(),
                  static_cast<unsigned long>(SystemCoreClock / 1000000u));

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

void App::controlStep(float dt) {
    ImuSample imu;
    if (d_.platform.imu().read(imu)) {
        d_.estimator.update(imu, dt, state_);
        state_.imuValid = true;
        ++imuSamples_;
    } else {
        state_.imuValid = false;
    }

    IMotor& motor = d_.platform.motor();
    const bool run = d_.supervisor.update(state_, d_.radio.linkUp());
    if (run) {
        motor.setEnabled(true);
        motor.setCommand(controller_->update(state_, dt));
    } else {
        motor.setEnabled(false);
    }
}

void App::auxStep() {
    float v;
    if (d_.platform.distanceLeft().read(v)) state_.distLeft = v;
    if (d_.platform.distanceRight().read(v)) state_.distRight = v;
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
              (d_.supervisor.failsafe() ? proto::kFlagFailsafe : 0);
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

void App::run() {
    const AppTiming timing;
    uint32_t now = d_.clock.micros();
    Periodic control{timing.controlPeriodUs, now};
    Periodic aux{timing.auxPeriodUs, now};
    Periodic radio{timing.radioPeriodUs, now};
    Periodic serialTlm{timing.serialTelemetryPeriodUs, now};
    Periodic stats{timing.statsPeriodUs, now + timing.statsPeriodUs};
    uint32_t lastControlUs = now;
    uint32_t blink = 0;

    while (true) {
        now = d_.clock.micros();
        ++loops_;

        d_.serial.poll();
        d_.platform.poll();
        d_.radio.poll();

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
            const RadioStream::Stats& rs = d_.radio.stats();
            d_.serial.log("radio tlm sent=%lu acked=%lu fail=%lu cmds=%lu link=%d stream=%lu",
                          static_cast<unsigned long>(rs.telemetrySent), static_cast<unsigned long>(rs.telemetryAcked),
                          static_cast<unsigned long>(rs.telemetryFailed), static_cast<unsigned long>(rs.commands),
                          d_.radio.linkUp() ? 1 : 0, static_cast<unsigned long>(rs.dataPackets));
            d_.serial.log("loops/s=%lu imu/s=%lu armed=%d failsafe=%d ctrl=%s uart rxerr=%lu drop=%lu",
                          static_cast<unsigned long>(loops_), static_cast<unsigned long>(imuSamples_),
                          d_.supervisor.armed() ? 1 : 0, d_.supervisor.failsafe() ? 1 : 0, controller_->name(),
                          static_cast<unsigned long>(d_.serial.rxErrors()),
                          static_cast<unsigned long>(d_.serial.droppedTx()));
            char diag[96];
            if (d_.platform.diagnostics(diag, sizeof(diag)) > 0) {
                d_.serial.log("%s", diag);
            }
            loops_ = 0;
            imuSamples_ = 0;
        }
    }
}

}  // namespace bike
