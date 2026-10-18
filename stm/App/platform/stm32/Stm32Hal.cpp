#include "platform/stm32/Stm32Hal.hpp"

#include <cstring>

namespace bike::stm32 {
namespace {

constexpr uint32_t kIrqBits = DMA_CCR_TCIE | DMA_CCR_HTIE | DMA_CCR_TEIE;

// Write-only reload from cached CCR values: peripheral reads stall the CPU, buffered writes do not.
HAL_RAM_FUNC void reloadChannel(DMA_Channel_TypeDef* ch, uint32_t ccr, const void* mem, size_t len) {
    ch->CCR = ccr;  // CNDTR/CMAR are writable only while disabled
    ch->CMAR = reinterpret_cast<uint32_t>(mem);
    ch->CNDTR = len;
    ch->CCR = ccr | DMA_CCR_EN;
}

}  // namespace

HAL_RAM_FUNC bool DmaSpi::startTransfer(const uint8_t* tx, uint8_t* rx, size_t len) {
    if (len == 0 || len > kMaxLen || busy()) {
        return false;
    }
    if (tx == nullptr) {
        std::memset(txScratch_, 0xFF, len);
        tx = txScratch_;
    }
    uint8_t* dst = (rx != nullptr) ? rx : rxScratch_;
    SPI_TypeDef* spi = h_->Instance;
    DMA_Channel_TypeDef* rxCh = h_->hdmarx->Instance;
    DMA_Channel_TypeDef* txCh = h_->hdmatx->Instance;

    const bool first = !configured_;
    if (first) {
        rxCh->CCR &= ~(DMA_CCR_EN | kIrqBits);
        txCh->CCR &= ~(DMA_CCR_EN | kIrqBits);
        rxCh->CPAR = reinterpret_cast<uint32_t>(&spi->DR);
        txCh->CPAR = reinterpret_cast<uint32_t>(&spi->DR);
        ccrRx_ = rxCh->CCR;
        ccrTx_ = txCh->CCR;
        configured_ = true;
    }
    // Order per RM0351: RX channel before TX, DMA requests enabled, then SPE (both stay set afterwards).
    reloadChannel(rxCh, ccrRx_, dst, len);
    reloadChannel(txCh, ccrTx_, tx, len);
    if (first) {
        spi->CR2 |= SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN;
        spi->CR1 |= SPI_CR1_SPE;
    }
    active_ = true;
    return true;
}

void DmaSpi::finish() const {
    // Abort path (timeout).
    h_->hdmarx->Instance->CCR &= ~DMA_CCR_EN;
    h_->hdmatx->Instance->CCR &= ~DMA_CCR_EN;
    active_ = false;
}

HAL_RAM_FUNC bool DmaSpi::busy() const {
    if (!active_) {
        return false;
    }
    if (h_->hdmarx->Instance->CNDTR != 0) {
        return true;
    }
    // RX count zero: done; the next startTransfer disables and reloads the channels.
    active_ = false;
    return false;
}

HAL_RAM_FUNC bool DmaSpi::transfer(const uint8_t* tx, uint8_t* rx, size_t len) {
    // Short transfers are polled (cheaper than DMA setup); idle channels ignore the DMA requests.
    if (len <= kPolledMaxLen && !busy()) {
        SPI_TypeDef* spi = h_->Instance;
        spi->CR1 |= SPI_CR1_SPE;
        volatile uint8_t* dr = reinterpret_cast<volatile uint8_t*>(&spi->DR);
        for (size_t i = 0; i < len; ++i) {
            while (!(spi->SR & SPI_SR_TXE)) {
            }
            *dr = (tx != nullptr) ? tx[i] : 0xFF;
            while (!(spi->SR & SPI_SR_RXNE)) {
            }
            const uint8_t b = *dr;
            if (rx != nullptr) rx[i] = b;
        }
        return true;
    }
    if (!startTransfer(tx, rx, len)) {
        return false;
    }
    const uint32_t start = HAL_GetTick();
    while (busy()) {
        if (HAL_GetTick() - start > 2) {  // 64 B at 10 MHz is ~0.05 ms
            finish();
            return false;
        }
    }
    return true;
}

bool DmaI2c::read(uint8_t addr7, uint16_t reg, uint8_t* data, size_t len) {
    if (taken_) return false;
    const bool ok = HAL_I2C_Mem_Read(h_, addr7 << 1, reg, I2C_MEMADD_SIZE_16BIT, data, len, kTimeoutMs) == HAL_OK;
    if (!ok) ++errors_;
    return ok;
}

bool DmaI2c::write(uint8_t addr7, uint16_t reg, const uint8_t* data, size_t len) {
    if (taken_) return false;
    const bool ok = HAL_I2C_Mem_Write(h_, addr7 << 1, reg, I2C_MEMADD_SIZE_16BIT, const_cast<uint8_t*>(data), len,
                                      kTimeoutMs) == HAL_OK;
    if (!ok) ++errors_;
    return ok;
}

bool DmaI2c::startRead(uint8_t addr7, uint16_t reg, uint8_t* data, size_t len) {
    if (taken_ || HAL_I2C_Mem_Read_DMA(h_, addr7 << 1, reg, I2C_MEMADD_SIZE_16BIT, data, len) != HAL_OK) {
        return false;
    }
    taken_ = true;
    startMs_ = HAL_GetTick();
    return true;
}

bool DmaI2c::startWrite(uint8_t addr7, uint16_t reg, const uint8_t* data, size_t len) {
    if (taken_ ||
        HAL_I2C_Mem_Write_DMA(h_, addr7 << 1, reg, I2C_MEMADD_SIZE_16BIT, const_cast<uint8_t*>(data), len) != HAL_OK) {
        return false;
    }
    taken_ = true;
    startMs_ = HAL_GetTick();
    return true;
}

hal::II2c::Result DmaI2c::result() {
    if (!taken_) {
        return Result::Error;
    }
    if (HAL_I2C_GetState(h_) != HAL_I2C_STATE_READY) {
        if (HAL_GetTick() - startMs_ <= kTimeoutMs) {
            return Result::Pending;
        }
        HAL_I2C_DeInit(h_);
        HAL_I2C_Init(h_);
        h_->ErrorCode = HAL_I2C_ERROR_TIMEOUT;
    }
    taken_ = false;
    if (h_->ErrorCode != HAL_I2C_ERROR_NONE) {
        ++errors_;
        return Result::Error;
    }
    return Result::Ok;
}

void DwtClock::init() {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    cyclesPerUs_ = SystemCoreClock / 1000000u;
    lastCycles_ = DWT->CYCCNT;
}

uint32_t DwtClock::micros() {
    // Must be called at least once per CYCCNT wrap (~53 s).
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const uint32_t now = DWT->CYCCNT;
    pendingCycles_ += now - lastCycles_;
    lastCycles_ = now;
    const uint32_t whole = pendingCycles_ / cyclesPerUs_;
    pendingCycles_ -= whole * cyclesPerUs_;
    us_ += whole;
    const uint32_t result = us_;
    __set_PRIMASK(primask);
    return result;
}

void DwtClock::delayUs(uint32_t us) {
    const uint32_t start = DWT->CYCCNT;
    const uint32_t cycles = us * cyclesPerUs_;
    while (DWT->CYCCNT - start < cycles) {
    }
}

}  // namespace bike::stm32
