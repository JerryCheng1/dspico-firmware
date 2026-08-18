cmake_minimum_required(VERSION 3.13)

# RP2354A = RP2350A die + stacked Winbond W25Q16JV 2 MB QSPI NOR flash on the
# dedicated QSPI pads (datasheet §14.3). It boots from that flash exactly like
# an RP2350 with external flash, so this is a normal XIP build (image runs
# from 0x10000000, survives power-cycle). Must be set before
# pico_sdk_import.cmake, with FORCE, or the SDK's cached rp2040/pico defaults win.
#   PICO_PLATFORM=rp2350-arm-s : Cortex-M33 application core
#   PICO_BOARD=pico2           : board header used only for SDK boilerplate -
#                                its default boot stage 2 (boot2_w25q080)
#                                explicitly supports the in-package W25Q16JV.
#                                Every pin is assigned explicitly by this
#                                firmware (GPIO0/22-29 PSRAM, 3-8 SDIO,
#                                UART1 on the USB_DP/DM Bank-1 pads).
set(PICO_PLATFORM rp2350-arm-s CACHE STRING "rp2350-arm-s" FORCE)
set(PICO_BOARD pico2 CACHE STRING "pico2" FORCE)

include(pico_sdk_import.cmake)
project(DSpico_debug C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

# Standalone RP2354A test firmware (see src/main.cpp). Reuses the unchanged
# PSRAM (src/psram.*, 4-chip port) and SDIO/SdCard/FatFs (src/sd/...) driver
# sources copied from dspico-firmware. Does not build or depend on
# dspico-firmware.
add_executable(DSpico_debug
  src/main.cpp
  src/common.h
  src/psram.c
  src/psram.h
  src/psram.pio
  src/sd/rp2350_sdio.cpp
  src/sd/rp2350_sdio.h
  src/sd/rp2350_sdio.pio
  src/sd/SdCard.cpp
  src/sd/SdCard.h
  src/sd/SdCardInfo.h
  src/sd/fatfs/diskio.cpp
  src/sd/fatfs/diskio.h
  src/sd/fatfs/ff.c
  src/sd/fatfs/ff.h
  src/sd/fatfs/ffconf.h
  src/sd/fatfs/ffunicode.c
  src/sd/fatfs/ffsystem.c
)

target_compile_definitions(DSpico_debug PRIVATE
  DSpico
  ENABLE_UART_LOG          # route the reused drivers' LOG() to the debug UART
  PICO_STACK_SIZE=0x600
)

target_include_directories(DSpico_debug PRIVATE src)

# Debug UART: UART1 muxed onto the Bank-1 USB_DP/DM pads (set up explicitly in
# main.cpp; no Bank-0 GPIO is used for stdio) at 115200; no USB stdio (the
# USB pads carry the UART).
pico_enable_stdio_uart(DSpico_debug 1)
pico_enable_stdio_usb(DSpico_debug 0)

pico_generate_pio_header(DSpico_debug ${CMAKE_CURRENT_LIST_DIR}/src/psram.pio)
pico_generate_pio_header(DSpico_debug ${CMAKE_CURRENT_LIST_DIR}/src/sd/rp2350_sdio.pio)

pico_add_extra_outputs(DSpico_debug)
target_link_libraries(DSpico_debug pico_stdlib hardware_pio hardware_dma hardware_clocks pico_multicore)
