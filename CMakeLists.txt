cmake_minimum_required(VERSION 3.13)

add_compile_definitions(APP_NAME="mfm")
add_compile_definitions(APP_VERSION="0.1")
add_compile_definitions(BUILD_NUM=1)

set(FREERTOS_CFG_DIRECTORY "${CMAKE_SOURCE_DIR}/FreeRTOS-SMP")
#set(FREERTOS_SRC_DIRECTORY "${CMAKE_SOURCE_DIR}/FreeRTOS-Kernel")

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)
include(FreeRTOS_Kernel_import.cmake)

project(mfm)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

file(MAKE_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/generated)

target_include_directories(FreeRTOS-Kernel INTERFACE
    ${FREERTOS_CFG_DIRECTORY}/
)

add_library(common INTERFACE)

pico_generate_pio_header(common ${CMAKE_CURRENT_LIST_DIR}/src/ws2812.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR}/generated)
target_sources(common INTERFACE
  src/main.c
  src/ws2812.c
  src/debug.c
  src/as5600.c
)

target_link_libraries(common INTERFACE pico_stdlib pico_bootsel_via_double_reset hardware_i2c hardware_pio FreeRTOS-Kernel FreeRTOS-Kernel-Heap4)

add_executable(mfm)
target_link_libraries(mfm common)
pico_add_extra_outputs(mfm)

add_executable(mfm_debug)
target_compile_definitions(mfm_debug PRIVATE
  DEBUG)
target_link_libraries(mfm_debug common)
pico_enable_stdio_usb(mfm_debug 1)
pico_add_extra_outputs(mfm_debug)