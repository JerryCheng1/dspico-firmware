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
option(ENABLE_SD_WRITE_PROTECT "Discard SD writes while reporting success to the host" OFF)
# Stage S2: build the standalone APS6404L qualification firmware. It is a
# separate executable and never links the cartridge runtime.
option(PSRAM_QUAL_FIRMWARE "Build the standalone APS6404L (PSRAM) qualification firmware" OFF)
option(PSRAM_QUAL_FULL "Qualification firmware: walk the full 8 MiB of every chip" OFF)
option(PSRAM_QUAL_FAULT_TEST "Qualification firmware: inject bounded PIO/DMA faults and verify recovery" OFF)
set(PSRAM_QUAL_WARM_CYCLES 0 CACHE STRING "Qualification firmware: successful automatic watchdog warm-reset cycles (requires fault test)")

# Design section 11.1 stage switches. CACHE_STAGE gates which cache modules are
# linked at all; CACHE_INIT_LOG/CACHE_STATS_LOG add bounded diagnostics and are
# independent of each other (stats requires the init log). Inconsistent
# combinations are rejected here rather than silently compiling a stage that
# the design does not allow.
set(CACHE_STAGE "0" CACHE STRING "Cache development stage (0..10)")
option(CACHE_INIT_LOG "Bounded asynchronous init log (requires ENABLE_UART_LOG)" OFF)
option(CACHE_STATS_LOG "Low-rate cache statistics log (requires CACHE_INIT_LOG)" OFF)
option(CACHE_L2_DISABLED "Force the PSRAM L2 read cache off (SRAM front-end regression)" OFF)
# Design section 10 diagnostic mode. M0..M5 isolate bring-up, snapshot cost,
# backfill coexistence, transfer/generation and the real hit path. This is the
# build knob the staged comparison in the r3 fix design requires; it is NOT a
# claim that any mode has been validated on hardware.
set(L2_DIAG_MODE "M5" CACHE STRING "L2 diagnostic mode M0..M5 (design section 10)")
# M3 contrast build only (design section 10): reproduce the old "publish then
# fill in the same round" behaviour so the new post-handover admission can be
# compared with ONE variable changed. Must stay OFF and must never become the
# default after acceptance.
option(L2_LEGACY_PUBLISH_FILL "Contrast build: old publish-then-fill behaviour (M3 only)" OFF)
option(CACHE_WATCH_LOG "Core1 WFE/SysTick cache diagnostics with idle gating" OFF)
option(CACHE_SUMMARY_LOG "Quiet one-line cache capacity, occupancy and hit-rate log" OFF)
option(PSRAM_BOOT_AUTOSWEEP "Sweep PSRAM PIO divider from 1.0 to 3.0 in 0.1 steps at boot" OFF)
option(CACHE_WATCH_ACTIVE_MARKERS "Intrusive core0 phase/gate/recovery markers (diagnostic experiments only)" OFF)
option(CACHE_FILL_QUIET_EXPERIMENT "Experimental M3/M4 fill or probe during a quiet acknowledged next-sector request" OFF)
option(CACHE_M4_PROMOTE_M5_AFTER_PROBES "Experimental M4-to-M5 transition after 32 clean full-sector probes; promoted M5 defaults to read-only" OFF)
option(CACHE_M5_QUIET_FILL_AFTER_PROMOTION "Experimental continued M5 quiet-window fill after M4 probe qualification" OFF)
option(CACHE_SHADOW_STRIPES_4CHIP "Experimental SD-served M4 shadow: rotate fills through 16 address stripes per PSRAM and verify every fill" OFF)
option(CACHE_FULL_PAGE_4CHIP "Experimental 4-chip 4 KiB page directory and checksum-table backend" OFF)
option(CACHE_PAGE_M5_EXPERIMENT "Allow M5 demand supply from the full-page backend after M4 hardware validation" OFF)
set(CACHE_PAGE_M5_SET_LIMIT 2031 CACHE STRING "M5 physical 4 KiB page sets per chip (1..2031; default uses full cache capacity)")
if(CACHE_PAGE_M5_EXPERIMENT AND NOT CACHE_FULL_PAGE_4CHIP)
  message(FATAL_ERROR "CACHE_PAGE_M5_EXPERIMENT requires CACHE_FULL_PAGE_4CHIP")
endif()
option(CACHE_BOOT_SECTOR0_SEED "Serve the first stage-3 sector-0 demand from the boot prewarm buffer" OFF)
option(CACHE_PROBE_MISMATCH_DIAG "Capture the first M4 PSRAM-vs-SD mismatch for idle UART diagnostics" OFF)
option(CACHE_PSRAM_BOOT_DIAG "Test PSRAM cache address zero via PIO and SIO before cartridge service" OFF)
option(CACHE_PSRAM_SECTOR0_DIAG "Test real SD sector zero at PSRAM cache address zero during prewarm" OFF)
set(CACHE_BUILD_GIT_REV "" CACHE STRING "Git revision embedded in the cache boot banner")

if(CACHE_STAGE LESS 0 OR CACHE_STAGE GREATER 10)
  message(FATAL_ERROR "CACHE_STAGE must be 0..10")
endif()
# Only S0..S3 are implemented. Refuse to build a later stage so a stage
# number can never be mistaken for a passed hardware validation.
if(CACHE_STAGE GREATER 3)
  message(FATAL_ERROR
    "CACHE_STAGE=${CACHE_STAGE} is not implemented yet; S0..S3 are the "
    "only buildable stages (design section 11.1).")
endif()
if(CACHE_INIT_LOG AND NOT ENABLE_UART_LOG)
  message(FATAL_ERROR "CACHE_INIT_LOG requires ENABLE_UART_LOG=ON")
endif()
if(CACHE_INIT_LOG AND CACHE_STAGE LESS 1)
  message(FATAL_ERROR "CACHE_INIT_LOG requires CACHE_STAGE>=1")
endif()
if(CACHE_STATS_LOG AND NOT CACHE_INIT_LOG)
  message(FATAL_ERROR "CACHE_STATS_LOG requires CACHE_INIT_LOG=ON")
endif()
if(CACHE_WATCH_ACTIVE_MARKERS AND NOT CACHE_WATCH_LOG)
  message(FATAL_ERROR "CACHE_WATCH_ACTIVE_MARKERS requires CACHE_WATCH_LOG")
endif()
if(CACHE_SUMMARY_LOG AND (NOT CACHE_WATCH_LOG OR NOT CACHE_FULL_PAGE_4CHIP))
  message(FATAL_ERROR "CACHE_SUMMARY_LOG requires CACHE_WATCH_LOG and CACHE_FULL_PAGE_4CHIP")
endif()
if(CACHE_WATCH_LOG)
  if(CACHE_STAGE LESS 3 OR NOT ENABLE_UART_LOG)
    message(FATAL_ERROR "CACHE_WATCH_LOG requires CACHE_STAGE>=3 and ENABLE_UART_LOG=ON")
  endif()
  if(CACHE_INIT_LOG OR CACHE_STATS_LOG OR ENABLE_CART_TRACE)
    message(FATAL_ERROR "CACHE_WATCH_LOG owns runtime UART: disable CACHE_INIT_LOG, CACHE_STATS_LOG and ENABLE_CART_TRACE")
  endif()
endif()
if(PSRAM_BOOT_AUTOSWEEP)
  string(TOUPPER "${L2_DIAG_MODE}" _sweep_diag_mode)
  if(CACHE_STAGE LESS 3 OR CACHE_L2_DISABLED OR
     NOT _sweep_diag_mode STREQUAL "M5" OR NOT ENABLE_UART_LOG)
    message(FATAL_ERROR "PSRAM_BOOT_AUTOSWEEP requires CACHE_STAGE>=3, enabled M5 and ENABLE_UART_LOG=ON")
  endif()
endif()
if(ENABLE_CART_TRACE AND NOT ENABLE_UART_LOG)
  message(FATAL_ERROR
    "ENABLE_CART_TRACE requires ENABLE_UART_LOG=ON: the runtime tracers emit "
    "their output through the UART log, which is compiled in only with "
    "ENABLE_UART_LOG=ON.")
endif()

# Unique per-build tag printed on the UART boot banner so each flashed build is
# verifiable. Pass -DUART_LOG_BUILD_TAG=... at configure time.
set(UART_LOG_BUILD_TAG "cache-send-r5" CACHE STRING "Boot banner build tag override")

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

target_compile_definitions(DSpico PRIVATE CACHE_STAGE=${CACHE_STAGE})
target_compile_definitions(DSpico PRIVATE CART_RECOVERY_DIAG=1)
if(PSRAM_BOOT_AUTOSWEEP)
  target_compile_definitions(DSpico PRIVATE PSRAM_BOOT_AUTOSWEEP=1)
endif()
if(CACHE_INIT_LOG)
  target_compile_definitions(DSpico PRIVATE CACHE_INIT_LOG=1)
endif()
if(CACHE_STATS_LOG)
  target_compile_definitions(DSpico PRIVATE CACHE_STATS_LOG=1)
endif()
if(CACHE_WATCH_LOG)
  target_compile_definitions(DSpico PRIVATE CACHE_WATCH_LOG=1)
  if(CACHE_SUMMARY_LOG)
    target_compile_definitions(DSpico PRIVATE CACHE_SUMMARY_LOG=1)
  endif()
  target_sources(DSpico PRIVATE src/cacheDiagLog.c)
  target_link_libraries(DSpico hardware_exception)
  if(CACHE_WATCH_ACTIVE_MARKERS)
    target_compile_definitions(DSpico PRIVATE CACHE_WATCH_ACTIVE_MARKERS=1)
  endif()
endif()
if(CACHE_STAGE GREATER_EQUAL 3)
  if(CACHE_L2_DISABLED)
    # F1/F2 regression: keep the front-end/job/slot state machine but never
    # enable the PSRAM read cache, so the same build can be compared against the
    # SD-only path (design section 11.3). A forced-off build overrides any
    # diagnostic mode (r3 fix design section 10).
    target_compile_definitions(DSpico PRIVATE CACHE_L2_DISABLED=1)
    target_compile_definitions(DSpico PRIVATE CACHE_L2_MODE=0)
  else()
    string(TOUPPER "${L2_DIAG_MODE}" _l2diag_upper)
    if(_l2diag_upper STREQUAL "M0")
      set(_l2diag_num 0)
    elseif(_l2diag_upper STREQUAL "M1")
      set(_l2diag_num 1)
    elseif(_l2diag_upper STREQUAL "M2")
      set(_l2diag_num 2)
    elseif(_l2diag_upper STREQUAL "M3")
      set(_l2diag_num 3)
    elseif(_l2diag_upper STREQUAL "M4")
      set(_l2diag_num 4)
    elseif(_l2diag_upper STREQUAL "M5")
      set(_l2diag_num 5)
    else()
      message(FATAL_ERROR "L2_DIAG_MODE must be M0..M5, got '${L2_DIAG_MODE}'")
    endif()
    target_compile_definitions(DSpico PRIVATE CACHE_L2_MODE=${_l2diag_num})
  endif()
endif()
if(CACHE_FILL_QUIET_EXPERIMENT)
  string(TOUPPER "${L2_DIAG_MODE}" _fill_diag_mode)
  if(CACHE_STAGE LESS 3 OR CACHE_L2_DISABLED OR
     (NOT _fill_diag_mode STREQUAL "M3" AND NOT _fill_diag_mode STREQUAL "M4" AND
      NOT (CACHE_FULL_PAGE_4CHIP AND CACHE_PAGE_M5_EXPERIMENT AND
           _fill_diag_mode STREQUAL "M5")) OR
     L2_LEGACY_PUBLISH_FILL OR NOT ENABLE_SD_WRITE_PROTECT)
    message(FATAL_ERROR "CACHE_FILL_QUIET_EXPERIMENT requires stage 3, M3/M4 (or explicit full-page M5), write protection and no legacy fill")
  endif()
  target_compile_definitions(DSpico PRIVATE CACHE_FILL_QUIET_EXPERIMENT=1)
endif()
if(CACHE_M4_PROMOTE_M5_AFTER_PROBES)
  string(TOUPPER "${L2_DIAG_MODE}" _promote_diag_mode)
  if(CACHE_STAGE LESS 3 OR CACHE_L2_DISABLED OR
     NOT _promote_diag_mode STREQUAL "M4" OR
     NOT CACHE_FILL_QUIET_EXPERIMENT OR
     L2_LEGACY_PUBLISH_FILL OR NOT ENABLE_SD_WRITE_PROTECT)
    message(FATAL_ERROR "CACHE_M4_PROMOTE_M5_AFTER_PROBES requires stage 3, M4, quiet experiment, write protection and no legacy fill")
  endif()
  target_compile_definitions(DSpico PRIVATE CACHE_M4_PROMOTE_M5_AFTER_PROBES=1)
endif()
if(CACHE_M5_QUIET_FILL_AFTER_PROMOTION)
  if(NOT CACHE_M4_PROMOTE_M5_AFTER_PROBES)
    message(FATAL_ERROR "CACHE_M5_QUIET_FILL_AFTER_PROMOTION requires CACHE_M4_PROMOTE_M5_AFTER_PROBES")
  endif()
  target_compile_definitions(DSpico PRIVATE CACHE_M5_QUIET_FILL_AFTER_PROMOTION=1)
endif()
if(CACHE_SHADOW_STRIPES_4CHIP)
  string(TOUPPER "${L2_DIAG_MODE}" _shadow_diag_mode)
  if(NOT CACHE_STAGE EQUAL 3 OR CACHE_L2_DISABLED OR
     NOT _shadow_diag_mode STREQUAL "M4" OR
     NOT CACHE_FILL_QUIET_EXPERIMENT OR
     CACHE_M4_PROMOTE_M5_AFTER_PROBES OR
     CACHE_M5_QUIET_FILL_AFTER_PROMOTION OR
     NOT ENABLE_SD_WRITE_PROTECT OR NOT CACHE_WATCH_LOG OR
     NOT CACHE_PROBE_MISMATCH_DIAG)
    message(FATAL_ERROR "CACHE_SHADOW_STRIPES_4CHIP requires stage 3 M4 quiet-fill, SD write protection, watch/probe diagnostics and no M5 promotion")
  endif()
  target_compile_definitions(DSpico PRIVATE CACHE_SHADOW_STRIPES_4CHIP=1)
endif()
if(CACHE_FULL_PAGE_4CHIP)
  string(TOUPPER "${L2_DIAG_MODE}" _page_diag_mode)
  if(NOT CACHE_STAGE EQUAL 3 OR CACHE_L2_DISABLED OR
     CACHE_SHADOW_STRIPES_4CHIP OR NOT CACHE_FILL_QUIET_EXPERIMENT OR
     NOT ENABLE_SD_WRITE_PROTECT OR NOT CACHE_WATCH_LOG OR
     (CACHE_M4_PROMOTE_M5_AFTER_PROBES AND NOT CACHE_PAGE_M5_EXPERIMENT) OR
     (NOT _page_diag_mode STREQUAL "M4" AND
      NOT _page_diag_mode STREQUAL "M5") OR
     (_page_diag_mode STREQUAL "M5" AND NOT CACHE_PAGE_M5_EXPERIMENT))
    message(FATAL_ERROR "CACHE_FULL_PAGE_4CHIP requires stage 3, M4 (or explicitly allowed M5), quiet-fill, SD write protection, watch logging and no stripe shadow")
  endif()
  target_compile_definitions(DSpico PRIVATE CACHE_FULL_PAGE_4CHIP=1)
  if(CACHE_PAGE_M5_EXPERIMENT)
    if(NOT CACHE_PAGE_M5_SET_LIMIT MATCHES "^[0-9]+$" OR
       CACHE_PAGE_M5_SET_LIMIT LESS 1 OR CACHE_PAGE_M5_SET_LIMIT GREATER 2031)
      message(FATAL_ERROR "CACHE_PAGE_M5_SET_LIMIT must be 1..2031")
    endif()
    target_compile_definitions(DSpico PRIVATE CACHE_PAGE_M5_EXPERIMENT=1)
    target_compile_definitions(DSpico PRIVATE CACHE_PAGE_ACTIVE_SETS=${CACHE_PAGE_M5_SET_LIMIT})
  endif()
  target_sources(DSpico PRIVATE src/cachePageMap.c src/cachePageStore.c)
endif()
if(CACHE_BOOT_SECTOR0_SEED)
  if(CACHE_STAGE LESS 3)
    message(FATAL_ERROR "CACHE_BOOT_SECTOR0_SEED requires CACHE_STAGE>=3")
  endif()
  target_compile_definitions(DSpico PRIVATE CACHE_BOOT_SECTOR0_SEED=1)
endif()
if(CACHE_PROBE_MISMATCH_DIAG)
  string(TOUPPER "${L2_DIAG_MODE}" _probe_diag_mode)
  if(CACHE_STAGE LESS 3 OR CACHE_L2_DISABLED OR
     (NOT _probe_diag_mode STREQUAL "M4" AND
      NOT (CACHE_FULL_PAGE_4CHIP AND CACHE_PAGE_M5_EXPERIMENT AND
           _probe_diag_mode STREQUAL "M5")) OR NOT CACHE_WATCH_LOG)
    message(FATAL_ERROR "CACHE_PROBE_MISMATCH_DIAG requires stage 3, watch log and M4 (or explicit full-page M5)")
  endif()
  target_compile_definitions(DSpico PRIVATE CACHE_PROBE_MISMATCH_DIAG=1)
endif()
if(CACHE_PSRAM_BOOT_DIAG)
  string(TOUPPER "${L2_DIAG_MODE}" _boot_diag_mode)
  if(CACHE_STAGE LESS 3 OR CACHE_L2_DISABLED OR
     NOT _boot_diag_mode STREQUAL "M2" OR CACHE_FILL_QUIET_EXPERIMENT OR
     NOT ENABLE_SD_WRITE_PROTECT OR NOT ENABLE_UART_LOG)
    message(FATAL_ERROR "CACHE_PSRAM_BOOT_DIAG requires CACHE_STAGE=3, L2 enabled, M2, quiet fill OFF, SD write protect and UART log")
  endif()
  target_compile_definitions(DSpico PRIVATE CACHE_PSRAM_BOOT_DIAG=1)
endif()
if(CACHE_PSRAM_SECTOR0_DIAG)
  if(NOT CACHE_PSRAM_BOOT_DIAG OR NOT CACHE_BOOT_SECTOR0_SEED)
    message(FATAL_ERROR "CACHE_PSRAM_SECTOR0_DIAG requires CACHE_PSRAM_BOOT_DIAG and CACHE_BOOT_SECTOR0_SEED")
  endif()
  target_compile_definitions(DSpico PRIVATE CACHE_PSRAM_SECTOR0_DIAG=1)
endif()
if(L2_LEGACY_PUBLISH_FILL)
  if(CACHE_STAGE LESS 3)
    message(FATAL_ERROR "L2_LEGACY_PUBLISH_FILL requires CACHE_STAGE>=3")
  endif()
  message(WARNING "L2_LEGACY_PUBLISH_FILL=ON: contrast-only build, not for acceptance")
  target_compile_definitions(DSpico PRIVATE CACHE_L2_LEGACY_PUBLISH_FILL=1)
endif()
if(CACHE_BUILD_GIT_REV)
  target_compile_definitions(DSpico PRIVATE CACHE_BUILD_GIT_REV="${CACHE_BUILD_GIT_REV}")
endif()
if(CACHE_STAGE GREATER_EQUAL 1)
  # S1 resource registry and bounded init log. Not linked at stage 0 so the
  # baseline image stays byte-identical (design section 11.3 rollback).
  target_sources(DSpico PRIVATE
    src/cacheResourcePlan.c
    src/cacheResourcePlan.h
    src/cacheInitLog.c
    src/cacheInitLog.h
    src/cacheInitLogRing.h
  )
endif()

if(CACHE_STAGE GREATER_EQUAL 3)
  # S3: PSRAM sector cache for the E3/E4/E5 path. Adds the PSRAM engine
  # (PIO2 SM0 + DMA4/5) and the cache service; stage 0..2 images are unchanged.
  target_sources(DSpico PRIVATE
    src/cacheSd.c
    src/cacheSd.h
    src/cacheSlots.c
    src/cacheSlots.h
    src/psram.c
    src/psram.h
  )
  pico_generate_pio_header(DSpico ${CMAKE_CURRENT_LIST_DIR}/src/psram.pio)
endif()

pico_generate_pio_header(DSpico ${CMAKE_CURRENT_LIST_DIR}/src/ntrCard.pio)
pico_generate_pio_header(DSpico ${CMAKE_CURRENT_LIST_DIR}/src/ntrCardTrace.pio)
pico_generate_pio_header(DSpico ${CMAKE_CURRENT_LIST_DIR}/src/sd/rp2350_sdio.pio)

if(PSRAM_QUAL_FIRMWARE)
  if(PSRAM_QUAL_WARM_CYCLES GREATER 0 AND NOT PSRAM_QUAL_FAULT_TEST)
    message(FATAL_ERROR "PSRAM_QUAL_WARM_CYCLES requires PSRAM_QUAL_FAULT_TEST=ON")
  endif()
  if(PSRAM_QUAL_WARM_CYCLES GREATER 0 AND PSRAM_QUAL_FULL)
    message(FATAL_ERROR "PSRAM_QUAL_WARM_CYCLES uses short regional/fault tests; set PSRAM_QUAL_FULL=OFF")
  endif()
  if(PSRAM_QUAL_WARM_CYCLES LESS 0 OR PSRAM_QUAL_WARM_CYCLES GREATER 100)
    message(FATAL_ERROR "PSRAM_QUAL_WARM_CYCLES must be in 0..100")
  endif()
  add_executable(PsramQual
    src/psramQualMain.c
    src/psram.c
    src/psram.h
    src/uartLog.c
    src/uartLog.h
    src/psram.pio
  )
  # The qualification boot always logs over UART1.
  target_compile_definitions(PsramQual PRIVATE ENABLE_UART_LOG=1)
  if(PSRAM_QUAL_FULL)
    target_compile_definitions(PsramQual PRIVATE PSRAM_QUAL_FULL=1)
  endif()
  if(PSRAM_QUAL_FAULT_TEST)
    target_compile_definitions(PsramQual PRIVATE PSRAM_QUAL_FAULT_TEST=1)
  endif()
  if(PSRAM_QUAL_WARM_CYCLES GREATER 0)
    target_compile_definitions(PsramQual PRIVATE PSRAM_QUAL_WARM_CYCLES=${PSRAM_QUAL_WARM_CYCLES})
  endif()
  if(UART_LOG_BUILD_TAG)
    target_compile_definitions(PsramQual PRIVATE UART_LOG_BUILD_TAG="${UART_LOG_BUILD_TAG}")
  endif()
  pico_generate_pio_header(PsramQual ${CMAKE_CURRENT_LIST_DIR}/src/psram.pio)
  pico_enable_stdio_uart(PsramQual 0)
  pico_add_extra_outputs(PsramQual)
  target_link_libraries(PsramQual pico_stdlib hardware_pio hardware_dma hardware_watchdog)
endif()

# DSpico is the flash image. Keep host tests, design documents, and the
# separate PSRAM qualification entry point out of its source list.
get_target_property(_dspico_sources DSpico SOURCES)
foreach(_dspico_source IN LISTS _dspico_sources)
  if(_dspico_source MATCHES "(^|/)(docs|tests)/" OR
     _dspico_source MATCHES "(^|/)psramQualMain\\.c$")
    message(FATAL_ERROR "Non-firmware source in DSpico: ${_dspico_source}")
  endif()
endforeach()

pico_enable_stdio_uart(DSpico 0)
pico_add_extra_outputs(DSpico)
target_link_libraries(DSpico pico_stdlib hardware_pio hardware_dma pico_multicore)
