cmake_minimum_required(VERSION 3.13)

# Radio base station: Waveshare RP2040-One + nRF24L01+, bridges the bike radio link to USB CDC.
# Toolchain/SDK from the Raspberry Pi Pico VS Code extension install (~/.pico-sdk), see README.md.
set(PICO_BOARD waveshare_rp2040_one CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(bike_base C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

set(COMMON_DIR ${CMAKE_CURRENT_LIST_DIR}/../common)

add_executable(bike_base
    src/main.cpp
    src/BaseStation.cpp
    src/RpHal.cpp
    ${COMMON_DIR}/src/nrf24/Nrf24.cpp
    ${COMMON_DIR}/src/proto/Frame.cpp
)
target_include_directories(bike_base PRIVATE src ${COMMON_DIR}/include)
target_compile_options(bike_base PRIVATE -Wall -Wextra -Wno-unused-parameter)
target_link_libraries(bike_base pico_stdlib pico_multicore pico_util hardware_spi hardware_dma)

# USB CDC to the laptop (binary frames), no UART stdio. USB also provides the picotool reset
# interface, so `picotool load -f` can reflash without pressing BOOTSEL.
pico_enable_stdio_usb(bike_base 1)
pico_enable_stdio_uart(bike_base 0)
pico_set_program_name(bike_base "bike_base")

pico_add_extra_outputs(bike_base)
