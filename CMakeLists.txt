cmake_minimum_required(VERSION 3.13)
#set(PICO_SDK_PATH "../FIRMWARE/pico-sdk") # enable this if vscode or cmake complains about PICO_SDK_PATH not being set
include(pico_sdk_import.cmake)
project(DSpico_project C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()
add_compile_definitions(DSpico
  PICO_STACK_SIZE=0x600
  PICO_CORE1_STACK_SIZE=0x400
  #DSPICO_ENABLE_WRFUXXED # This macro enables the WRFUXXED exploit to work.
  # ENABLE_PREVENT_DSI_AUTOBOOT # meant to be used with WRFU, doesn't work properly on the 3ds
)

option(ENABLE_R4_MODE "Enable R4 emulation" ON)
option(ENABLE_PSRAM_CACHE "Cache SD sectors (E3/E5 path) in the external APS6404 PSRAM (GPIO22-26/29)" ON)
option(ENABLE_UART_LOG "Print firmware logs on the debug UART (GPIO0/1)" OFF)
option(PSRAM_USE_PIO "Use the experimental PIO1 SM2 PSRAM path with dynamic SDIO RX/TX program sharing" OFF)
option(PSRAM_CACHE_ENABLE_ON_PROBE "Enable E3/E5 cache after PSRAM probe; OFF is a probe-only diagnostic" ON)

add_executable(DSpico
  src/main.cpp
  src/common.h
  src/pioUtil.h
  src/uartLog.c
  src/uartLog.h
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
  src/r4.h
  src/sd/rp2040_sdio.cpp
  src/sd/rp2040_sdio.h
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
  src/psram.c
  src/psram.h
  src/romCache.c
  src/romCache.h
)

if(EXISTS "${CMAKE_SOURCE_DIR}/roms/default.nds" AND EXISTS "${CMAKE_SOURCE_DIR}/roms/dsimode.nds")
  target_compile_definitions(DSpico PRIVATE
    DETECT_CONSOLE_TYPE # This macro enables the firmware to switch the rom based on which console is detected. You shouldn't change this.
  )
endif()

if(ENABLE_PSRAM_CACHE)
  target_compile_definitions(DSpico PRIVATE
    ENABLE_PSRAM_CACHE
  )
endif()

if(PSRAM_CACHE_ENABLE_ON_PROBE)
  target_compile_definitions(DSpico PRIVATE PSRAM_CACHE_ENABLE_ON_PROBE=1)
else()
  target_compile_definitions(DSpico PRIVATE PSRAM_CACHE_ENABLE_ON_PROBE=0)
endif()

if(PSRAM_USE_PIO)
  target_compile_definitions(DSpico PRIVATE PSRAM_FORCE_BITBANG=0)
else()
  target_compile_definitions(DSpico PRIVATE PSRAM_FORCE_BITBANG=1)
endif()

if(ENABLE_R4_MODE)
  target_compile_definitions(DSpico PRIVATE
    ENABLE_R4_MODE
  )
endif()

if(ENABLE_UART_LOG)
  target_compile_definitions(DSpico PRIVATE
    ENABLE_UART_LOG
  )
  pico_enable_stdio_uart(DSpico 1)
else()
  pico_enable_stdio_uart(DSpico 0)
endif()

pico_generate_pio_header(DSpico ${CMAKE_CURRENT_LIST_DIR}/src/ntrCard.pio)
pico_generate_pio_header(DSpico ${CMAKE_CURRENT_LIST_DIR}/src/sd/rp2040_sdio.pio)
if(PSRAM_USE_PIO)
  pico_generate_pio_header(DSpico ${CMAKE_CURRENT_LIST_DIR}/src/psram.pio)
endif()

pico_add_extra_outputs(DSpico)
target_link_libraries(DSpico pico_stdlib hardware_pio hardware_dma pico_multicore)

# Pico SDK's RP2040 linker script places core stacks at the tops of the two
# 4 KB scratch banks but does not assert that user scratch sections stay below
# them. Make overlap a hard link error instead of a low-probability runtime
# corruption of the cartridge IRQ code.
set(DSPICO_SCRATCH_ASSERT_SCRIPT
  ${CMAKE_CURRENT_LIST_DIR}/src/scratch_stack_assert.ld)
target_link_options(DSpico PRIVATE
  "-Wl,-T${DSPICO_SCRATCH_ASSERT_SCRIPT}"
)
set_property(TARGET DSpico APPEND PROPERTY LINK_DEPENDS
  ${DSPICO_SCRATCH_ASSERT_SCRIPT})
