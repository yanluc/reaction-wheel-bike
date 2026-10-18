#pragma once
// Framed UART link (LPUART1 -> ST-LINK VCP): RX by circular DMA read via the DMA counter,
// TX ring buffer sent in DMA chunks (never blocks).

#include <cstddef>
#include <cstdint>

#include "main.h"
#include "proto/Frame.hpp"
#include "proto/Messages.hpp"

namespace bike::stm32 {

class IFrameListener {
public:
    virtual void onFrame(const uint8_t* msg, size_t len) = 0;

protected:
    ~IFrameListener() = default;
};

class SerialLink {
public:
    explicit SerialLink(UART_HandleTypeDef* h) : h_(h) {}

    void start();
    void poll();

    bool addListener(proto::MsgType type, IFrameListener& listener);
    bool send(const void* msg, size_t len);
    void log(const char* fmt, ...) __attribute__((format(printf, 2, 3)));

    uint32_t droppedTx() const { return droppedTx_; }
    uint32_t rxErrors() const { return rxErrors_; }
    uint32_t crcErrors() const { return decoder_.crcErrors(); }

    // ISR context
    static void onTxComplete(UART_HandleTypeDef* h);
    static void onError(UART_HandleTypeDef* h);

private:
    void kickTx();
    void restartRx();

    static constexpr size_t kRxSize = 512;
    static constexpr size_t kTxSize = 2048;
    static constexpr size_t kMaxListeners = 4;

    struct Listener {
        proto::MsgType type;
        IFrameListener* listener;
    };

    UART_HandleTypeDef* h_;
    uint8_t rxBuf_[kRxSize] = {};
    size_t rxTail_ = 0;
    proto::FrameDecoder decoder_;

    uint8_t txBuf_[kTxSize] = {};
    volatile size_t txHead_ = 0;
    volatile size_t txTail_ = 0;
    volatile size_t txInFlight_ = 0;

    Listener listeners_[kMaxListeners] = {};
    size_t listenerCount_ = 0;
    uint32_t droppedTx_ = 0;
    volatile uint32_t rxErrors_ = 0;
    volatile bool rxRestart_ = false;
};

}  // namespace bike::stm32
