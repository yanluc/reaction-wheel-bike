#include "platform/sim/SimPlatform.hpp"

#include <cstring>

namespace bike {

bool SimImu::read(ImuSample& out) {
    if (!fresh_) {
        return false;
    }
    out = sample_;
    fresh_ = false;
    return true;
}

bool SimPlatform::init() {
    link_.addListener(proto::MsgType::SimSensors, *this);
    return motor_.init();
}

void SimPlatform::onFrame(const uint8_t* msg, size_t len) {
    proto::SimSensors s;
    if (len != sizeof(s)) {
        return;
    }
    std::memcpy(&s, msg, sizeof(s));

    for (int i = 0; i < 3; ++i) {
        imu_.sample_.accel[i] = s.accel[i];
        imu_.sample_.gyro[i] = s.gyro[i];
    }
    imu_.sample_.timeUs = s.timeUs;
    imu_.fresh_ = true;
    distLeft_.value_ = s.distLeftM;
    distLeft_.timeUs_ = s.timeUs;
    distLeft_.valid_ = true;
    distRight_.value_ = s.distRightM;
    distRight_.timeUs_ = s.timeUs;
    distRight_.valid_ = true;
    current_.value_ = s.currentA;
    current_.valid_ = true;

    proto::SimActuators a;
    a.motorCommand = motor_.command();
    a.enabled = motor_.enabled() ? 1 : 0;
    link_.send(&a, sizeof(a));
}

}  // namespace bike
