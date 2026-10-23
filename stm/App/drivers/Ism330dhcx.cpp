#include "drivers/Ism330dhcx.hpp"

#include <atomic>

namespace bike {
namespace {

constexpr uint8_t kWhoAmI = 0x0F;
constexpr uint8_t kWhoAmIValue = 0x6B;
constexpr uint8_t kCounterBdrReg1 = 0x0B;
constexpr uint8_t kInt1Ctrl = 0x0D;
constexpr uint8_t kCtrl1Xl = 0x10;
constexpr uint8_t kCtrl2G = 0x11;
constexpr uint8_t kCtrl3C = 0x12;
constexpr uint8_t kCtrl4C = 0x13;
constexpr uint8_t kOdr6667 = 0xA0;
constexpr uint8_t kSwReset = 0x01;
constexpr uint8_t kBduIfInc = 0x44;
constexpr uint8_t kI2cDisable = 0x04;
constexpr uint8_t kInt1DrdyG = 0x02;
constexpr uint8_t kDrdyPulsed = 0x80;
constexpr uint8_t kRead = 0x80;

constexpr float kG = 9.80665f;
constexpr float kDegToRad = 0.017453293f;
constexpr float kAccelMgPerLsb[] = {0.061f, 0.488f, 0.122f, 0.244f};
constexpr float kGyroMdpsPerLsb[] = {8.75f, 17.5f, 35.0f, 70.0f};
constexpr uint32_t kPeriodUs = 150;

}  // namespace

bool Ism330dhcx::writeReg(uint8_t reg, uint8_t value) {
    const uint8_t tx[2] = {reg, value};
    cs_.write(false);
    const bool ok = spi_.transfer(tx, nullptr, 2);
    cs_.write(true);
    return ok;
}

uint8_t Ism330dhcx::readReg(uint8_t reg) {
    const uint8_t tx[2] = {static_cast<uint8_t>(reg | kRead), 0};
    uint8_t rx[2] = {};
    cs_.write(false);
    spi_.transfer(tx, rx, 2);
    cs_.write(true);
    return rx[1];
}

bool Ism330dhcx::init() {
    ok_ = false;
    cs_.write(true);
    uint8_t dummy = 0;
    spi_.transfer(&dummy, nullptr, 1);  // enables SPI so SCK idles high (mode 3) before the first CS

    whoAmI_ = readReg(kWhoAmI);
    if (whoAmI_ != kWhoAmIValue) {
        return false;
    }
    writeReg(kCtrl3C, kSwReset);
    for (int i = 0; i < 100 && (readReg(kCtrl3C) & kSwReset); ++i) {
        clock_.delayUs(10);
    }
    writeReg(kCtrl4C, kI2cDisable);
    writeReg(kCtrl3C, kBduIfInc);
    writeReg(kCtrl1Xl, kOdr6667 | (static_cast<uint8_t>(cfg_.accel) << 2));
    writeReg(kCtrl2G, kOdr6667 | (static_cast<uint8_t>(cfg_.gyro) << 2));
    writeReg(kCounterBdrReg1, kDrdyPulsed);  // edge IRQ: a latched level would stop after one missed read
    writeReg(kInt1Ctrl, kInt1DrdyG);
    if (readReg(kCtrl2G) != (kOdr6667 | (static_cast<uint8_t>(cfg_.gyro) << 2))) {
        return false;
    }

    accelScale_ = kAccelMgPerLsb[static_cast<uint8_t>(cfg_.accel)] * 1e-3f * kG;
    gyroScale_ = kGyroMdpsPerLsb[static_cast<uint8_t>(cfg_.gyro)] * 1e-3f * kDegToRad;
    tx_[0] = kBurstStart | kRead;
    ok_ = true;
    return true;
}

HAL_RAM_FUNC void Ism330dhcx::onDataReady(uint32_t stamp) {
    if (!ok_) {
        return;
    }
    if (busy_) {
        if (spi_.busy()) {
            ++overruns_;
            return;
        }
        cs_.write(true);
        const uint8_t head = head_;
        const uint8_t next = (head + 1) % kQueueSize;
        if (next == tail_) {
            ++overflows_;
        } else {
            Raw& r = queue_[head];
            const uint32_t* src = reinterpret_cast<const uint32_t*>(&rxBuf_[4]);
            r.data[0] = src[0];
            r.data[1] = src[1];
            r.data[2] = src[2];
            r.stamp = pendingStamp_;
            std::atomic_signal_fence(std::memory_order_release);
            head_ = next;
        }
    }
    pendingStamp_ = stamp;
    cs_.write(false);
    busy_ = spi_.startTransfer(tx_, rx_, sizeof(tx_));
    if (!busy_) {
        cs_.write(true);
        ++overruns_;
    }
}

HAL_RAM_FUNC bool Ism330dhcx::read(ImuSample& out) {
    if (!started_) {
        // Capture runs from init(), but the loop starts only after the slow ToF/radio init: drop that backlog.
        started_ = true;
        tail_ = head_;
        overruns_ = 0;
        overflows_ = 0;
        return false;
    }
    const uint8_t tail = tail_;
    if (tail == head_) {
        return false;
    }
    std::atomic_signal_fence(std::memory_order_acquire);
    const Raw& r = queue_[tail];
    const uint8_t* bytes = reinterpret_cast<const uint8_t*>(r.data);
    float raw[6];
    for (int i = 0; i < 6; ++i) {
        raw[i] = static_cast<float>(static_cast<int16_t>(bytes[2 * i] | (bytes[2 * i + 1] << 8)));
    }
    for (int i = 0; i < 3; ++i) {
        const int8_t a = cfg_.axis[i];
        const int idx = (a > 0 ? a : -a) - 1;
        const float sign = a > 0 ? 1.0f : -1.0f;
        out.gyro[i] = sign * raw[idx] * gyroScale_;
        out.accel[i] = sign * raw[3 + idx] * accelScale_;
    }
    const uint32_t stamp = r.stamp;
    std::atomic_signal_fence(std::memory_order_release);
    tail_ = (tail + 1) % kQueueSize;

    const uint32_t tpu = clock_.ticksPerUs();
    if (samples_ == 0) {
        timeUs_ = clock_.micros() - (clock_.ticks() - stamp) / tpu;  // same time base as clock.micros()
    } else {
        stampRemainder_ += stamp - lastStamp_;
        const uint32_t us = stampRemainder_ / tpu;
        stampRemainder_ -= us * tpu;
        if (us > kPeriodUs * 3 / 2) {
            ++gaps_;
        }
        timeUs_ += us;
    }
    lastStamp_ = stamp;
    out.timeUs = timeUs_;
    ++samples_;
    return true;
}

}  // namespace bike
