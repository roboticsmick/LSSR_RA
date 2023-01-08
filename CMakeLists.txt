# Set minimum required version of CMake
cmake_minimum_required(VERSION 3.12)

# Include build functions from Pico SDK
# Set Cmake: Configure Environment and Cmake: Configure Environment
# Key: PICO_SDK_PATH
# Value: ~/pico/pico-sdk/external
#include($ENV{PICO_SDK_PATH}/pico_sdk_import.cmake)
#include(~/pico/pico-sdk/external/pico_sdk_import.cmake)
# Copy the ~/pico/pico-sdk/external/pico_sdk_import.cmake file into main folder
include(pico_sdk_import.cmake)

# Set name of project (as PROJECT_NAME) and C/C++ standards
project(main C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Creates a pico-sdk subdirectory in our project for the libraries
pico_sdk_init()

# Tell CMake where to find the executable source file
add_executable(${PROJECT_NAME} 
    src/main.cpp
    inc/LSR_ADXL3xx.c
    inc/LSR_MS56xx.c
    inc/LSR_H3LIS200DLTR.c
    inc/LSR_serial.c
)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(${PROJECT_NAME})

# Link to pico_stdlib (gpio, time, etc. functions)
target_link_libraries(${PROJECT_NAME} 
    pico_stdlib
    hardware_i2c
    hardware_spi
    hardware_uart
    hardware_gpio
)

# Enable usb output, disable uart output
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 1)
