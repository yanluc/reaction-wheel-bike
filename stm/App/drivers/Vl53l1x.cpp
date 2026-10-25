#include "drivers/Vl53l1x.hpp"

namespace bike {
namespace {

constexpr uint16_t kI2cAddress = 0x0001;
constexpr uint16_t kVhvLoopBound = 0x0008;
constexpr uint16_t kVhvStart = 0x000B;
constexpr uint16_t kTioHvStatus = 0x0031;
constexpr uint16_t kPhasecalTimeout = 0x004B;
constexpr uint16_t kTimeoutA = 0x005E;
constexpr uint16_t kVcselPeriodA = 0x0060;
constexpr uint16_t kTimeoutB = 0x0061;
constexpr uint16_t kVcselPeriodB = 0x0063;
constexpr uint16_t kValidPhaseHigh = 0x0069;
constexpr uint16_t kWoiSd0 = 0x0078;
constexpr uint16_t kInitialPhaseSd0 = 0x007A;
constexpr uint16_t kInterruptClear = 0x0086;
constexpr uint16_t kModeStart = 0x0087;
constexpr uint16_t kResultBlock = 0x0089;
constexpr uint16_t kSystemStatus = 0x00E5;
constexpr uint16_t kModelId = 0x010F;
constexpr uint16_t kConfigStart = 0x002D;
constexpr uint8_t kStatusValid = 9;

// VL51L1X_DEFAULT_CONFIGURATION (registers 0x2D..0x87) from ST VL53L1X ULD.
// COPYRIGHT(c) 2018 STMicroelectronics, BSD 3-Clause License.
constexpr uint8_t kDefaultConfig[] = {
    0x00, 0x01, 0x01, 0x01, 0x02, 0x00, 0x02, 0x08, 0x00, 0x08, 0x10, 0x01, 0x01, 0x00, 0x00, 0x00,
    0x00, 0xff, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x0b, 0x00, 0x00, 0x02, 0x0a, 0x21,
    0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xc8, 0x00, 0x00, 0x38, 0xff, 0x01, 0x00, 0x08, 0x00,
    0x00, 0x01, 0xdb, 0x0f, 0x01, 0xf1, 0x0d, 0x01, 0x68, 0x00, 0x80, 0x08, 0xb8, 0x00, 0x00, 0x00,
    0x00, 0x0f, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x0f, 0x0d, 0x0e, 0x0e, 0x00,
    0x00, 0x02, 0xc7, 0xff, 0x9B, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00,
};
static_assert(sizeof(kDefaultConfig) == 0x87 - 0x2D + 1, "default config covers 0x2D..0x87");

struct TimingBudget {
    uint16_t ms;
    uint16_t a;
    uint16_t b;
};
constexpr TimingBudget kShortModeBudgets[] = {
    {15, 0x001D, 0x0027}, {20, 0x0051, 0x006E},  {33, 0x00D6, 0x006E},  {50, 0x01AE, 0x01E8},
    {100, 0x02E1, 0x0388}, {200, 0x03E1, 0x0496}, {500, 0x0591, 0x05C1},
};

}  // namespace

bool Vl53l1x::wrByte(uint16_t reg, uint8_t v) { return bus_.write(addr_, reg, &v, 1); }

bool Vl53l1x::wrWord(uint16_t reg, uint16_t v) {
    const uint8_t b[2] = {static_cast<uint8_t>(v >> 8), static_cast<uint8_t>(v)};
    return bus_.write(addr_, reg, b, 2);
}

bool Vl53l1x::rdByte(uint16_t reg, uint8_t& v) { return bus_.read(addr_, reg, &v, 1); }

bool Vl53l1x::rdWord(uint16_t reg, uint16_t& v) {
    uint8_t b[2];
    if (!bus_.read(addr_, reg, b, 2)) return false;
    v = static_cast<uint16_t>((b[0] << 8) | b[1]);
    return true;
}

bool Vl53l1x::waitDataReady(uint32_t timeoutMs) {
    const uint32_t start = clock_.millis();
    uint8_t s = 0;
    while (clock_.millis() - start < timeoutMs) {
        if (rdByte(kTioHvStatus, s) && dataReady(s)) return true;
        clock_.delayUs(1000);
    }
    return false;
}

bool Vl53l1x::init() {
    ok_ = false;
    state_ = State::Idle;
    addr_ = kDefaultAddress;
    xshut_.write(true);
    clock_.delayUs(2000);

    uint8_t booted = 0;
    const uint32_t start = clock_.millis();
    while (!(rdByte(kSystemStatus, booted) && (booted & 1))) {
        if (clock_.millis() - start > 100) return false;
        clock_.delayUs(1000);
    }
    if (cfg_.address != kDefaultAddress) {
        if (!wrByte(kI2cAddress, cfg_.address)) return false;
        addr_ = cfg_.address;
    }
    uint16_t model = 0;
    if (!rdWord(kModelId, model) || model != 0xEACC) return false;

    if (!bus_.write(addr_, kConfigStart, kDefaultConfig, sizeof(kDefaultConfig))) return false;
    wrByte(kInterruptClear, 0x01);
    wrByte(kModeStart, 0x40);
    if (!waitDataReady(150)) return false;
    wrByte(kInterruptClear, 0x01);
    wrByte(kModeStart, 0x00);
    wrByte(kVhvLoopBound, 0x09);
    wrByte(kVhvStart, 0x00);

    // Short distance mode (<= 1.3 m, better in ambient light).
    wrByte(kPhasecalTimeout, 0x14);
    wrByte(kVcselPeriodA, 0x07);
    wrByte(kVcselPeriodB, 0x05);
    wrByte(kValidPhaseHigh, 0x38);
    wrWord(kWoiSd0, 0x0705);
    wrWord(kInitialPhaseSd0, 0x0606);

    const TimingBudget* tb = nullptr;
    for (const TimingBudget& t : kShortModeBudgets) {
        if (t.ms == cfg_.timingBudgetMs) tb = &t;
    }
    if (tb == nullptr) return false;
    wrWord(kTimeoutA, tb->a);
    wrWord(kTimeoutB, tb->b);

    wrByte(kInterruptClear, 0x01);
    ok_ = true;
    return true;
}

void Vl53l1x::finishResult() {
    lastStatus_ = result_[0] & 0x1F;
    ++measurements_;
    if (lastStatus_ == kStatusValid) {
        lastMm_ = static_cast<uint16_t>((result_[13] << 8) | result_[14]);
        lastTimeUs_ = startUs_ + timingBudgetUs() / 2;  // middle of the integration window
        fresh_ = true;
    } else {
        ++invalid_;
    }
}

bool Vl53l1x::trigger() {
    if (!ok_ || state_ != State::Idle || !bus_.startWrite(addr_, kModeStart, &oneShot_, 1)) {
        return false;
    }
    state_ = State::Triggering;
    return true;
}

hal::II2c::Result Vl53l1x::busResult() {
    const hal::II2c::Result r = bus_.result();
    if (r == hal::II2c::Result::Error) ++busErrors_;
    return r;
}

void Vl53l1x::poll() {
    using R = hal::II2c::Result;
    if (!ok_) {
        return;
    }
    R r;
    switch (state_) {
        case State::Idle:
            break;
        case State::Triggering:
            r = busResult();
            if (r == R::Pending) break;
            if (r == R::Error) {
                state_ = State::Idle;
                break;
            }
            startUs_ = clock_.micros();
            state_ = State::Measuring;
            break;
        case State::Measuring:
            if (clock_.micros() - startUs_ > 3 * timingBudgetUs()) {
                ++timeouts_;
                state_ = State::Idle;
            } else if (gpio1_.read() && bus_.startRead(addr_, kResultBlock, result_, sizeof(result_))) {
                state_ = State::ReadResult;
            }
            break;
        case State::ReadResult:
            r = busResult();
            if (r == R::Pending) break;
            if (r == R::Error) {
                state_ = State::Measuring;  // retried until the timeout
                break;
            }
            finishResult();
            state_ = bus_.startWrite(addr_, kInterruptClear, &clear_, 1) ? State::ClearIrq : State::Idle;
            break;
        case State::ClearIrq:
            if (busResult() != R::Pending) state_ = State::Idle;
            break;
    }
}

bool Vl53l1x::read(float& meters, uint32_t& timeUs) {
    if (!fresh_) {
        return false;
    }
    meters = static_cast<float>(lastMm_) * 1e-3f;
    timeUs = lastTimeUs_;
    fresh_ = false;
    return true;
}

}  // namespace bike
