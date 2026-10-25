#include "platform/sim/SimPlatform.hpp"

#include <cstring>

namespace bike {

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
        imu_.sample.accel[i] = s.accel[i];
        imu_.sample.gyro[i] = s.gyro[i];
    }
    imu_.sample.timeUs = s.timeUs;
    imu_.fresh = true;
    distLeft_.value = s.distLeftM;
    distRight_.value = s.distRightM;
    distLeft_.timeUs = distRight_.timeUs = s.timeUs;
    distLeft_.fresh = distRight_.fresh = true;
    current_.value = s.currentA;
    current_.valid = true;

    proto::SimActuators a;
    a.motorCommand = motor_.command();
    a.enabled = motor_.enabled() ? 1 : 0;
    link_.send(&a, sizeof(a));
}

}  // namespace bike
