#include "platform/stm32/SerialLink.hpp"

#include <cstdarg>
#include <cstdio>
#include <cstring>

namespace bike::stm32 {
namespace {
SerialLink* g_lpuart1 = nullptr;
}

void SerialLink::start() {
    if (h_->Instance == LPUART1) {
        g_lpuart1 = this;
    }
    restartRx();
}

void SerialLink::restartRx() {
    rxTail_ = 0;
    HAL_UART_Receive_DMA(h_, rxBuf_, kRxSize);
}

void SerialLink::poll() {
    if (rxRestart_) {  // an RX error aborted the circular DMA (HAL treats any error as blocking)
        rxRestart_ = false;
        restartRx();
    }
    const size_t head = (kRxSize - __HAL_DMA_GET_COUNTER(h_->hdmarx)) % kRxSize;
    while (rxTail_ != head) {
        const uint8_t b = rxBuf_[rxTail_];
        rxTail_ = (rxTail_ + 1) % kRxSize;
        if (!decoder_.feed(b)) {
            continue;
        }
        const auto type = static_cast<proto::MsgType>(decoder_.payload()[0]);
        for (size_t i = 0; i < listenerCount_; ++i) {
            if (listeners_[i].type == type) {
                listeners_[i].listener->onFrame(decoder_.payload(), decoder_.size());
            }
        }
    }
}

bool SerialLink::addListener(proto::MsgType type, IFrameListener& listener) {
    if (listenerCount_ >= kMaxListeners) {
        return false;
    }
    listeners_[listenerCount_++] = {type, &listener};
    return true;
}

bool SerialLink::send(const void* msg, size_t len) {
    uint8_t frame[proto::kMaxFramePayload + proto::kFrameOverhead];
    const size_t n = proto::encodeFrame(msg, len, frame, sizeof(frame));
    if (n == 0) {
        return false;
    }
    const size_t head = txHead_;
    const size_t used = (head + kTxSize - txTail_) % kTxSize;
    if (kTxSize - 1 - used < n) {
        ++droppedTx_;
        return false;
    }
    for (size_t i = 0; i < n; ++i) {
        txBuf_[(head + i) % kTxSize] = frame[i];
    }
    txHead_ = (head + n) % kTxSize;
    kickTx();
    return true;
}

void SerialLink::kickTx() {
    // Runs from the main loop and from the DMA-complete ISR; IRQs off keeps them from racing.
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (txInFlight_ == 0 && txHead_ != txTail_) {
        const size_t tail = txTail_;
        const size_t chunk = (txHead_ > tail) ? (txHead_ - tail) : (kTxSize - tail);
        if (HAL_UART_Transmit_DMA(h_, &txBuf_[tail], static_cast<uint16_t>(chunk)) == HAL_OK) {
            txInFlight_ = chunk;
        }
    }
    __set_PRIMASK(primask);
}

void SerialLink::log(const char* fmt, ...) {
    char msg[proto::kMaxFramePayload];
    msg[0] = static_cast<char>(proto::MsgType::Log);
    va_list ap;
    va_start(ap, fmt);
    const int n = std::vsnprintf(&msg[1], sizeof(msg) - 1, fmt, ap);
    va_end(ap);
    if (n < 0) {
        return;
    }
    const size_t len = 1 + ((static_cast<size_t>(n) < sizeof(msg) - 1) ? static_cast<size_t>(n) : sizeof(msg) - 2);
    send(msg, len);
}

void SerialLink::onTxComplete(UART_HandleTypeDef* h) {
    SerialLink* self = (h->Instance == LPUART1) ? g_lpuart1 : nullptr;
    if (self == nullptr) {
        return;
    }
    self->txTail_ = (self->txTail_ + self->txInFlight_) % kTxSize;
    self->txInFlight_ = 0;
    self->kickTx();
}

void SerialLink::onError(UART_HandleTypeDef* h) {
    SerialLink* self = (h->Instance == LPUART1) ? g_lpuart1 : nullptr;
    if (self == nullptr) {
        return;
    }
    ++self->rxErrors_;
    if (h->RxState == HAL_UART_STATE_READY) {  // reception was aborted -> restart from the loop
        self->rxRestart_ = true;
    }
    if (h->gState == HAL_UART_STATE_READY && self->txInFlight_ != 0) {  // TX aborted -> drop chunk
        self->txTail_ = (self->txTail_ + self->txInFlight_) % kTxSize;
        self->txInFlight_ = 0;
    }
}

}  // namespace bike::stm32

extern "C" {
void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) { bike::stm32::SerialLink::onTxComplete(huart); }
void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) { bike::stm32::SerialLink::onError(huart); }
}
