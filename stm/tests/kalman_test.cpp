// Host test (from stm/tests): g++ -std=c++17 -O2 -I../App -I../../common/include kalman_test.cpp -o /tmp/kalman_test && /tmp/kalman_test
#include <cmath>
#include <cstdio>
#include <random>

#include "control/Estimators.hpp"

namespace {

constexpr float kDeg = 0.017453293f;
constexpr float kDt = 1.0f / 6667.0f;
constexpr int kTofEvery = 133;  // ~50 Hz

int check(const char* what, float value, float expected, float tol) {
    const bool ok = std::fabs(value - expected) <= tol;
    std::printf("  %-32s %9.5f (expected %9.5f +- %.5f) %s\n", what, value, expected, tol, ok ? "OK" : "FAIL");
    return ok ? 0 : 1;
}

struct Bike {
    float roll = 0.0f;           // rad, true
    float bias = 0.0f;           // rad/s, gyro
    float lateral = 0.0f;        // m/s^2 on top of gravity (wheel torque)
    float tofLeftExtra = 0.0f;   // m, left sensor mounted higher
    float wobble = 0.0f;         // rad/s amplitude of motion (someone holding the bike)
    float moveRate = 0.0f;       // rad/s, slow lean change
    float b = 0.2f;
    float z = 0.15f;
    std::mt19937 rng{1};
    std::normal_distribution<float> gyroNoise{0.0f, 0.002f};
    std::normal_distribution<float> accelNoise{0.0f, 0.05f};
    std::normal_distribution<float> tofNoise{0.0f, 0.001f};
    int i = 0;

    void step(bike::KalmanRollEstimator& est, bike::BikeState& state) {
        const float rate = moveRate + wobble * std::sin(2.0f * 3.14159f * 2.0f * i * kDt);
        roll += rate * kDt;
        bike::ImuSample s{};
        s.gyro[0] = rate + bias + gyroNoise(rng);
        s.gyro[1] = gyroNoise(rng);
        s.gyro[2] = gyroNoise(rng);
        s.accel[0] = accelNoise(rng);
        s.accel[1] = 9.80665f * std::sin(roll) + lateral + accelNoise(rng);
        s.accel[2] = 9.80665f * std::cos(roll) + accelNoise(rng);
        est.update(s, kDt, state);
        if (i % kTofEvery == 0) {
            const float left = z + tofLeftExtra + (b / 2) * std::tan(roll) + tofNoise(rng);
            const float right = z - (b / 2) * std::tan(roll) + tofNoise(rng);
            est.updateDistances(left, right, state);
        }
        ++i;
    }
    void run(bike::KalmanRollEstimator& est, bike::BikeState& state, float seconds) {
        for (int n = 0; n < static_cast<int>(seconds / kDt); ++n) step(est, state);
    }
};

bike::KalmanRollEstimator makeEstimator() {
    bike::KalmanRollEstimator::Config cfg;
    cfg.tofBaseline = 0.2f;
    bike::KalmanRollEstimator est(cfg);
    est.reset();
    return est;
}

int steady(float rollDeg, float biasDegS) {
    std::printf("-- still at %.1f deg, bias %.2f deg/s, then wheel torque (lateral accel 0.3 m/s^2)\n", rollDeg, biasDegS);
    Bike bike;
    bike.roll = rollDeg * kDeg;
    bike.bias = biasDegS * kDeg;
    auto est = makeEstimator();
    bike::BikeState state;
    bike.run(est, state, 1.0f);
    int fails = check("not ready after 1 s", state.estimatorReady ? 1.0f : 0.0f, 0.0f, 0.0f);
    bike.run(est, state, 0.6f);
    fails += check("ready after 1.6 s", state.estimatorReady ? 1.0f : 0.0f, 1.0f, 0.0f);
    // 0.1 s after calibration: a few ToF pairs, each with ~0.007 rad noise (1 mm per sensor, b = 0.2 m).
    fails += check("roll right after calibration", state.roll, bike.roll, 0.006f);
    fails += check("bias right after calibration", est.bias(), bike.bias, 0.0005f);
    bike.lateral = 0.3f;  // accel now off by ~1.75 deg: ToF must hold the true angle
    bike.run(est, state, 20.0f);
    fails += check("roll after 20 s", state.roll, bike.roll, 0.003f);
    fails += check("bias after 20 s", est.bias(), bike.bias, 0.002f);
    return fails;
}

int tofMountingOffset() {
    std::printf("-- kickstand at 12 deg, left ToF mounted 8 mm higher (offset absorbed), then lean to -3 deg\n");
    Bike bike;
    bike.roll = 12.0f * kDeg;
    bike.bias = 0.3f * kDeg;
    bike.tofLeftExtra = 0.008f;
    auto est = makeEstimator();
    bike::BikeState state;
    bike.run(est, state, 2.0f);
    int fails = check("ToF calibrated", est.tofCalibrated() ? 1.0f : 0.0f, 1.0f, 0.0f);
    fails += check("ToF offset [rad]", est.tofOffset(),
                   std::atan((0.008f + 0.2f * std::tan(bike.roll)) / 0.2f) - bike.roll, 0.002f);
    // Lifted from the stand to -3 deg over 3 s, then held.
    const float target = -3.0f * kDeg;
    bike.moveRate = (target - bike.roll) / 3.0f;
    bike.run(est, state, 3.0f);
    bike.moveRate = 0.0f;
    bike.run(est, state, 5.0f);
    fails += check("roll after the move", state.roll, target, 0.004f);
    return fails;
}

int motionRestartsCalibration() {
    std::printf("-- held by hand (wobbling) for 3 s, then still\n");
    Bike bike;
    bike.roll = 4.0f * kDeg;
    bike.wobble = 0.3f;
    auto est = makeEstimator();
    bike::BikeState state;
    bike.run(est, state, 3.0f);
    int fails = check("not ready while moving", state.estimatorReady ? 1.0f : 0.0f, 0.0f, 0.0f);
    bike.wobble = 0.0f;
    bike.run(est, state, 2.0f);
    fails += check("ready after standing still", state.estimatorReady ? 1.0f : 0.0f, 1.0f, 0.0f);
    fails += check("roll", state.roll, bike.roll, 0.003f);
    char line[160];
    est.status(line, sizeof(line));
    std::printf("  %s\n", line);
    return fails;
}

int tofSign() {
    std::printf("-- ToF sign: dL growing relative to calibration -> roll increases\n");
    auto est = makeEstimator();
    bike::BikeState state;
    bike::ImuSample s{};
    s.accel[2] = 9.80665f;
    for (int i = 0; i < 6667 * 2; ++i) {
        est.update(s, kDt, state);
        if (i % kTofEvery == 0) est.updateDistances(0.15f, 0.15f, state);
    }
    for (int i = 0; i < 50; ++i) est.updateDistances(0.20f, 0.10f, state);
    return check("roll > 0", state.roll > 0.0f ? 1.0f : 0.0f, 1.0f, 0.0f);
}

}  // namespace

int main() {
    int fails = 0;
    fails += steady(5.0f, 0.5f);
    fails += steady(-8.0f, -1.0f);
    fails += tofMountingOffset();
    fails += motionRestartsCalibration();
    fails += tofSign();
    std::printf(fails == 0 ? "ALL OK\n" : "%d FAILED\n", fails);
    return fails == 0 ? 0 : 1;
}
