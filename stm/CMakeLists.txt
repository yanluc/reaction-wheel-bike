cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# HW = real sensors/motor, SIM = devices backed by the PC simulator over LPUART1
set(BIKE_PLATFORM "HW" CACHE STRING "Bike platform: HW or SIM")
set_property(CACHE BIKE_PLATFORM PROPERTY STRINGS HW SIM)
option(BIKE_RADIO_BENCH "Build the radio throughput benchmark instead of the app" OFF)
option(BIKE_STREAM_TEST "Build the stream test (4 periodic signals) instead of the app" OFF)
option(BIKE_STREAM_ACK "Stream test: ACK every packet instead of NOACK + XOR parity" OFF)
set(BIKE_STREAM_RATE_HZ 6700 CACHE STRING "Stream test sample rate [Hz], multiple of 10")


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME stm)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})
message("Bike platform: " ${BIKE_PLATFORM})

set(COMMON_DIR ${CMAKE_SOURCE_DIR}/../common)

# Enable CMake support for ASM and C languages
enable_language(C CXX ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    App/app_main.cpp
    App/App.cpp
    App/comms/RadioStream.cpp
    App/bench/RadioBench.cpp
    App/bench/StreamTest.cpp
    App/platform/stm32/Stm32Hal.cpp
    App/platform/stm32/SerialLink.cpp
    App/platform/stm32/AdcDma.cpp
    App/platform/hw/HwPlatform.cpp
    App/platform/sim/SimPlatform.cpp
    ${COMMON_DIR}/src/nrf24/Nrf24.cpp
    ${COMMON_DIR}/src/proto/Frame.cpp
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    App
    ${COMMON_DIR}/include
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    $<$<STREQUAL:${BIKE_PLATFORM},SIM>:BIKE_PLATFORM_SIM=1>
    $<$<BOOL:${BIKE_RADIO_BENCH}>:BIKE_RADIO_BENCH=1>
    $<$<BOOL:${BIKE_STREAM_TEST}>:BIKE_STREAM_TEST=1>
    $<$<BOOL:${BIKE_STREAM_ACK}>:BIKE_STREAM_ACK=1>
    BIKE_STREAM_RATE_HZ=${BIKE_STREAM_RATE_HZ}
)

target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:-Wall -Wextra -Wno-unused-parameter>
)

# .RamFunc (hot radio path executed from SRAM) makes a RWX load segment; expected on this MCU.
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -Wl,--no-warn-rwx-segments)

# Hot path (radio stream, drivers, control) at -O2 in release builds; generated HAL code keeps -Os.
file(GLOB_RECURSE BIKE_APP_SOURCES CONFIGURE_DEPENDS ${CMAKE_SOURCE_DIR}/App/*.cpp ${COMMON_DIR}/src/*.cpp)
if(NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
    set_source_files_properties(${BIKE_APP_SOURCES} PROPERTIES COMPILE_OPTIONS "-O2")
endif()

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
