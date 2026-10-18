#include "RpHal.hpp"

#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "pico/time.h"

namespace base {

Pin::Pin(uint gpio, bool initial) : gpio_(gpio) {
    gpio_init(gpio_);
    gpio_put(gpio_, initial);
    gpio_set_dir(gpio_, GPIO_OUT);
}

void Pin::write(bool high) { gpio_put(gpio_, high); }

InputPin::InputPin(uint gpio) : gpio_(gpio) {
    gpio_init(gpio_);
    gpio_set_dir(gpio_, GPIO_IN);
    gpio_pull_up(gpio_);
}

bool InputPin::read() { return gpio_get(gpio_); }

DmaSpi::DmaSpi(spi_inst_t* spi, uint baudrate, uint sck, uint mosi, uint miso) : spi_(spi) {
    spi_init(spi_, baudrate);
    spi_set_format(spi_, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(sck, GPIO_FUNC_SPI);
    gpio_set_function(mosi, GPIO_FUNC_SPI);
    gpio_set_function(miso, GPIO_FUNC_SPI);
    txChan_ = dma_claim_unused_channel(true);
    rxChan_ = dma_claim_unused_channel(true);
}

bool DmaSpi::transfer(const uint8_t* tx, uint8_t* rx, size_t len) {
    static const uint8_t kFill = 0xFF;
    static uint8_t sink;

    dma_channel_config tc = dma_channel_get_default_config(txChan_);
    channel_config_set_transfer_data_size(&tc, DMA_SIZE_8);
    channel_config_set_dreq(&tc, spi_get_dreq(spi_, true));
    channel_config_set_read_increment(&tc, tx != nullptr);
    channel_config_set_write_increment(&tc, false);

    dma_channel_config rc = dma_channel_get_default_config(rxChan_);
    channel_config_set_transfer_data_size(&rc, DMA_SIZE_8);
    channel_config_set_dreq(&rc, spi_get_dreq(spi_, false));
    channel_config_set_read_increment(&rc, false);
    channel_config_set_write_increment(&rc, rx != nullptr);

    dma_channel_configure(rxChan_, &rc, rx != nullptr ? rx : &sink, &spi_get_hw(spi_)->dr, len, false);
    dma_channel_configure(txChan_, &tc, &spi_get_hw(spi_)->dr, tx != nullptr ? tx : &kFill, len, false);
    dma_start_channel_mask((1u << txChan_) | (1u << rxChan_));
    dma_channel_wait_for_finish_blocking(rxChan_);  // RX finishing implies TX finished too
    return true;
}

uint32_t Clock::millis() { return to_ms_since_boot(get_absolute_time()); }
uint32_t Clock::micros() { return time_us_32(); }
void Clock::delayUs(uint32_t us) { sleep_us(us); }

}  // namespace base
