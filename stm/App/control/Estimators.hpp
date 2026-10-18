#pragma once

#include <cmath>
#include <cstdint>
#include <cstdio>

#include "interfaces/Control.hpp"

namespace bike {

class ComplementaryEstimator final : public IEstimator {
public:
    explicit ComplementaryEstimator(float alpha = 0.98f) : alpha_(alpha) {}

    void reset() override { initialized_ = false; }

    void update(const ImuSample& imu, float dt, BikeState& state) override {
        const float rollAcc = std::atan2(imu.accel[1], imu.accel[2]);
        const float rate = imu.gyro[0];
        if (!initialized_) {
            state.roll = rollAcc;
            initialized_ = true;
        } else {
            state.roll = alpha_ * (state.roll + rate * dt) + (1.0f - alpha_) * rollAcc;
        }
        state.rollRate = rate;
        state.estimatorReady = true;
    }

private:
    float alpha_;
    bool initialized_ = false;
};

// State [roll, gyro bias]. Gyro drives the prediction; corrections come from the accelerometer
// (weak: reaction-wheel torque adds lateral acceleration) and from the two downward ToF sensors
// (strong: over flat ground dL - dR = b * tan(roll), independent of height).
// After reset() the bike must stand still (any lean, e.g. on the kickstand) for calibSeconds: the
// mean gyro gives the bias, the mean accel the initial roll, and the ToF angle minus that roll the
// ToF offset (mounting, sensor heights, floor slope). Any motion restarts the calibration.
class KalmanRollEstimator final : public IEstimator {
public:
    struct Config {
        float qRoll = 1e-6f;       // rad^2/s
        float qBias = 1e-8f;       // (rad/s)^2/s
        float rAccel = 0.05f;      // rad^2
        float rTof = 1e-4f;        // rad^2
        float accelGate = 1.0f;    // m/s^2, skip accel when | |a| - g | is larger
        float tofBaseline = 0.2f;  // m, lateral distance between the beams
        uint8_t accelEvery = 8;    // accel correction every N IMU samples
        float calibSeconds = 1.5f;
        float stillGyroStd = 0.02f;  // rad/s, per axis
        float stillAccel = 0.5f;     // m/s^2, max | |a| - g |
        uint16_t calibMinTof = 20;   // ToF pairs needed for the ToF offset
    };

    explicit KalmanRollEstimator(const Config& cfg) : cfg_(cfg) {}

    void reset() override {
        calibrating_ = true;
        tofCalibrated_ = false;
        restartCalibration();
        count_ = 0;
    }

    void update(const ImuSample& imu, float dt, BikeState& state) override {
        if (calibrating_) {
            calibrate(imu, dt, state);
            return;
        }
        const float rate = imu.gyro[0];
        roll_ += (rate - bias_) * dt;
        p_[0][0] += dt * (dt * p_[1][1] - p_[0][1] - p_[1][0] + cfg_.qRoll);
        p_[0][1] -= dt * p_[1][1];
        p_[1][0] -= dt * p_[1][1];
        p_[1][1] += cfg_.qBias * dt;
        if (++count_ >= cfg_.accelEvery) {
            count_ = 0;
            if (std::fabs(accelNorm(imu) - kG) < cfg_.accelGate) {
                correct(std::atan2(imu.accel[1], imu.accel[2]), cfg_.rAccel);
            }
        }
        state.roll = roll_;
        state.rollRate = rate - bias_;
        state.estimatorReady = true;
    }

    void updateDistances(float left, float right, BikeState& state) override {
        const float z = std::atan((left - right) / cfg_.tofBaseline);
        if (calibrating_) {
            tofSum_ += z;
            ++tofCount_;
            return;
        }
        if (!tofCalibrated_) return;
        correct(z - tofOffset_, cfg_.rTof);
        state.roll = roll_;
    }

    int status(char* out, int size) const override {
        if (calibrating_) {
            return std::snprintf(out, size, "estimator: calibrating (stand still) %d/%d ms, restarts=%lu",
                                 static_cast<int>(calibTime_ * 1000.0f), static_cast<int>(cfg_.calibSeconds * 1000.0f),
                                 static_cast<unsigned long>(restarts_));
        }
        return std::snprintf(out, size, "estimator: ready, start roll=%d mrad, bias=%d mrad/s, tof offset=%d mrad%s",
                             static_cast<int>(startRoll_ * 1000.0f), static_cast<int>(bias_ * 1000.0f),
                             static_cast<int>(tofOffset_ * 1000.0f), tofCalibrated_ ? "" : " (no ToF data)");
    }

    bool calibrating() const { return calibrating_; }
    float bias() const { return bias_; }
    float tofOffset() const { return tofOffset_; }
    bool tofCalibrated() const { return tofCalibrated_; }

private:
    static constexpr float kG = 9.80665f;

    static float accelNorm(const ImuSample& imu) {
        return std::sqrt(imu.accel[0] * imu.accel[0] + imu.accel[1] * imu.accel[1] + imu.accel[2] * imu.accel[2]);
    }

    void restartCalibration() {
        calibTime_ = 0.0f;
        calibCount_ = 0;
        tofSum_ = 0.0f;
        tofCount_ = 0;
        for (int i = 0; i < 3; ++i) {
            gyroSum_[i] = 0.0f;
            gyroSq_[i] = 0.0f;
        }
        accelSum_[0] = accelSum_[1] = 0.0f;
    }

    void calibrate(const ImuSample& imu, float dt, BikeState& state) {
        state.estimatorReady = false;
        if (calibCount_ == 0) {
            for (int i = 0; i < 3; ++i) gyro0_[i] = imu.gyro[i];  // sums relative to the first sample
        }
        if (std::fabs(accelNorm(imu) - kG) > cfg_.stillAccel) {
            ++restarts_;
            restartCalibration();
            return;
        }
        for (int i = 0; i < 3; ++i) {
            const float d = imu.gyro[i] - gyro0_[i];
            gyroSum_[i] += d;
            gyroSq_[i] += d * d;
        }
        accelSum_[0] += imu.accel[1];
        accelSum_[1] += imu.accel[2];
        ++calibCount_;
        calibTime_ += dt;
        if (calibTime_ < cfg_.calibSeconds) {
            return;
        }

        const float n = static_cast<float>(calibCount_);
        for (int i = 0; i < 3; ++i) {
            const float mean = gyroSum_[i] / n;
            if (gyroSq_[i] / n - mean * mean > cfg_.stillGyroStd * cfg_.stillGyroStd) {
                ++restarts_;
                restartCalibration();
                return;
            }
        }
        bias_ = gyro0_[0] + gyroSum_[0] / n;
        roll_ = std::atan2(accelSum_[0], accelSum_[1]);
        startRoll_ = roll_;
        tofCalibrated_ = tofCount_ >= cfg_.calibMinTof;
        tofOffset_ = tofCalibrated_ ? tofSum_ / static_cast<float>(tofCount_) - roll_ : 0.0f;
        p_[0][0] = 1e-4f;
        p_[0][1] = p_[1][0] = 0.0f;
        p_[1][1] = 1e-6f;
        calibrating_ = false;
        state.roll = roll_;
        state.rollRate = imu.gyro[0] - bias_;
        state.estimatorReady = true;
    }

    void correct(float z, float r) {
        const float y = z - roll_;
        const float s = p_[0][0] + r;
        const float k0 = p_[0][0] / s;
        const float k1 = p_[1][0] / s;
        roll_ += k0 * y;
        bias_ += k1 * y;
        const float p00 = p_[0][0];
        const float p01 = p_[0][1];
        p_[0][0] -= k0 * p00;
        p_[0][1] -= k0 * p01;
        p_[1][0] -= k1 * p00;
        p_[1][1] -= k1 * p01;
    }

    Config cfg_;
    float roll_ = 0.0f;
    float bias_ = 0.0f;
    float p_[2][2] = {};
    uint8_t count_ = 0;

    bool calibrating_ = true;
    float calibTime_ = 0.0f;
    uint32_t calibCount_ = 0;
    uint32_t restarts_ = 0;
    float gyro0_[3] = {};
    float gyroSum_[3] = {};
    float gyroSq_[3] = {};
    float accelSum_[2] = {};
    float tofSum_ = 0.0f;
    uint32_t tofCount_ = 0;
    bool tofCalibrated_ = false;
    float tofOffset_ = 0.0f;
    float startRoll_ = 0.0f;
};

}  // namespace bike
