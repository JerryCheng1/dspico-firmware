cmake_minimum_required(VERSION 3.13)
#set(PICO_SDK_PATH "../FIRMWARE/pico-sdk") # enable this if vscode or cmake complains about PICO_SDK_PATH not being set
set(PICO_PLATFORM rp2350 CACHE STRING "Target RP platform")
set(PICO_BOARD dspico_rp2354a CACHE STRING "Target board")
list(APPEND PICO_BOARD_HEADER_DIRS ${CMAKE_CURRENT_LIST_DIR}/src/boards)
list(REMOVE_DUPLICATES PICO_BOARD_HEADER_DIRS)
if(NOT PICO_SDK_PATH)
  if(DEFINED ENV{PICO_SDK_PATH})
    set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
  else()
    set(PICO_SDK_PATH ${CMAKE_CURRENT_LIST_DIR}/pico-sdk)
  endif()
endif()
include(pico_sdk_import.cmake)
project(DSpico_project C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()
add_compile_definitions(DSpico
  PICO_STACK_SIZE=0x600
  PICO_CORE1_STACK_SIZE=0x400
  ENABLE_R4_MODE
  # ENABLE_UART_LOG - UART init plus BOOT-TIME logging only (SD bring-up,
  # f_mount, prewarm). It must NOT enable any runtime tracing: once the NDS
  # starts talking to the cartridge the build has to behave exactly like
  # ENABLE_UART_LOG=OFF. In particular it must not add work to PIO0_IRQ_0,
  # the GPIO/DMA/timer IRQs or the E3/E4/E5 command handlers.
  $<$<BOOL:${ENABLE_UART_LOG}>:ENABLE_UART_LOG>
  # ENABLE_CART_TRACE - non-intrusive cartridge-runtime diagnostics: a PIO0
  # input listener observes the physical CEB/WREB/D0..D7 bus, an IRQ-less DMA
  # channel stores the raw command words in an SRAM ring, and core1 decodes
  # them and emits bounded non-blocking UART text. It adds nothing to
  # PIO0_IRQ_0, the GPIO/DMA/timer IRQs or the E3/E4/E5 handlers. Off by
  # default and never implied by ENABLE_UART_LOG.
  $<$<BOOL:${ENABLE_CART_TRACE}>:ENABLE_CART_TRACE>
  # ENABLE_SD_WRITE_PROTECT - SD card write protection: every SD write (DS-side
  # SD/R4 commands and the FatFs disk_write path) is silently swallowed so the
  # card contents cannot be modified, while reads keep working normally.
  $<$<BOOL:${ENABLE_SD_WRITE_PROTECT}>:ENABLE_SD_WRITE_PROTECT>
  #DSPICO_ENABLE_WRFUXXED # This macro enables the WRFUXXED exploit to work.
  # ENABLE_PREVENT_DSI_AUTOBOOT # meant to be used with WRFU, doesn't work properly on the 3ds
)

# Runtime-diagnostic switches. Both default OFF and are independent: a plain
# UART log build (ENABLE_UART_LOG=ON, ENABLE_CART_TRACE=OFF) only emits
# boot-time output and leaves the real-time cartridge path untouched.
option(ENABLE_UART_LOG "Boot-time UART logging (no runtime tracing)" OFF)
option(ENABLE_CART_TRACE "Cartridge runtime diagnostics (requires ENABLE_UART_LOG)" OFF)
if(ENABLE_CART_TRACE AND NOT ENABLE_UART_LOG)
  message(FATAL_ERROR
    "ENABLE_CART_TRACE requires ENABLE_UART_LOG=ON: the runtime tracers emit "
    "their output through the UART log, which is compiled in only with "
    "ENABLE_UART_LOG=ON.")
endif()

# Unique per-build tag printed on the UART boot banner so each flashed build is
# verifiable. Pass -DUART_LOG_BUILD_TAG=... at configure time.
set(UART_LOG_BUILD_TAG "" CACHE STRING "Boot banner build tag override")

add_executable(DSpico
  src/main.cpp
  src/uartLog.c
  src/uartLog.h
  src/common.h
  src/romData.S
  src/romData.h
  src/blowfish.c
  src/blowfish.h
  src/powerSaving.c
  src/powerSaving.h
  src/scrambler.c
  src/scrambler.h
  src/scramblerRing.c
  src/scramblerRing.h
  src/ntrCardRom.c
  src/ntrCardRom.h
  src/ntrCardRomNorm.c
  src/ntrCardRomSecure.c
  src/ntrCardRomGame.c
  src/ntrCardRomGame.h
  src/ntrCardRomGameNoScramble.c
  src/ntrCardRomGameNoScramble.h
  src/ntrCardRomGameSd.cpp
  src/ntrCardRomGameR4.cpp
  src/ntrCardSpiUart.c
  src/ntrCardSpiUart.h
  src/wrfuxxed.h
  src/wrfuxxed.S
  src/xor.S
  src/xor.h
  src/ntrCardIrq.S
  src/ntrCardTrace.c
  src/ntrCardTrace.h
  src/ntrCardTrace.pio
  src/r4.h
  src/sd/rp2350_sdio.cpp
  src/sd/rp2350_sdio.h
  src/sd/SdCardInfo.h
  src/sd/SdCardInfo.h
  src/sd/SdCard.cpp
  src/sd/fatfs/diskio.cpp
  src/sd/fatfs/diskio.h
  src/sd/fatfs/ff.c
  src/sd/fatfs/ff.h
  src/sd/fatfs/ffconf.h
  src/sd/fatfs/ffunicode.c
  src/sd/fatfs/ffsystem.c
)

if(EXISTS "${CMAKE_SOURCE_DIR}/roms/default.nds" AND EXISTS "${CMAKE_SOURCE_DIR}/roms/dsimode.nds")
  target_compile_definitions(DSpico PRIVATE
    DETECT_CONSOLE_TYPE # This macro enables the firmware to switch the rom based on which console is detected. You shouldn't change this.
  )
endif()

# Unique per-build tag printed on the UART boot banner so each flashed build is
# verifiable. Pass -DUART_LOG_BUILD_TAG=... at configure time.
if(UART_LOG_BUILD_TAG)
  target_compile_definitions(DSpico PRIVATE UART_LOG_BUILD_TAG="${UART_LOG_BUILD_TAG}")
endif()

pico_generate_pio_header(DSpico ${CMAKE_CURRENT_LIST_DIR}/src/ntrCard.pio)
pico_generate_pio_header(DSpico ${CMAKE_CURRENT_LIST_DIR}/src/ntrCardTrace.pio)
pico_generate_pio_header(DSpico ${CMAKE_CURRENT_LIST_DIR}/src/sd/rp2350_sdio.pio)

pico_enable_stdio_uart(DSpico 0)
pico_add_extra_outputs(DSpico)
target_link_libraries(DSpico pico_stdlib hardware_pio hardware_dma pico_multicore)
