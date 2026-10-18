// Radio base station (Waveshare RP2040-One).
// nRF24L01+ wiring: CE=GP4, CSN=GP1, SCK=GP2, MOSI=GP3, MISO=GP0 (SPI0), IRQ=GP5.

#include "BaseStation.hpp"
#include "RpHal.hpp"
#include "nrf24/Nrf24.hpp"
#include "pico/multicore.h"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"

namespace {
constexpr uint kPinMiso = 0;
constexpr uint kPinCsn = 1;
constexpr uint kPinSck = 2;
constexpr uint kPinMosi = 3;
constexpr uint kPinCe = 4;
constexpr uint kPinIrq = 5;
constexpr uint kSpiBaud = 10'000'000;  // nRF24 max; actual rate is the nearest divider of clk_peri

base::Clock clock;
base::SharedState shared;

void core1Main() {
    static base::UsbCore usb(clock, shared);
    while (true) {
        usb.poll();
    }
}
}  // namespace

int main() {
    stdio_init_all();
    stdio_set_translate_crlf(&stdio_usb, false);  // the USB link carries binary frames
    shared.init();
    multicore_launch_core1(core1Main);

    static base::Pin csn(kPinCsn, true);
    static base::Pin ce(kPinCe, false);
    static base::DmaSpi spi(spi0, kSpiBaud, kPinSck, kPinMosi, kPinMiso);
    static base::InputPin irq(kPinIrq);
    static nrf24::Nrf24 radio(spi, ce, csn, clock, &irq);
    static base::RadioCore station(radio, clock, shared);

    nrf24::Config cfg;  // defaults shared with the bike
    station.init(cfg);

    while (true) {
        station.poll();
    }
}
