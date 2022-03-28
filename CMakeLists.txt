cmake_minimum_required(VERSION 3.21)

set(PICO_SDK_PATH ../pico-sdk)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)
project(floppyorgan C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.3.0")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.3.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif ()

set(PICO_EXAMPLES_PATH ${PROJECT_SOURCE_DIR})

# Initialize the SDK
pico_sdk_init()
include(example_auto_set_url.cmake)

add_subdirectory(src)

