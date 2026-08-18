#pragma once

// Trimmed common header for the dspico-debug test firmware.
// Mirrors the subset of dspico-firmware/src/common.h that the reused PSRAM
// and SDIO/SdCard/FatFs drivers depend on (types, pin map, log, millis).
// The NTR cartridge bus definitions are intentionally omitted: this firmware
// only exercises the PSRAM (GPIO0 + GPIO22-29) and MicroSD (GPIO3-8).
//
// Pin map source: references/hardware/RP2354A_GPIO分配总结.md
// (Netlist_Schematic1_1_2026-08-18.tel, RP2354A board).
//
// Debug output: UART1 muxed onto the Bank-1 USB_DP/USB_DM pads
// (FUNCSEL 0x02, see §6 of the GPIO summary). It is NOT on a Bank 0 GPIO, so
// it is set up explicitly in main.cpp, not via stdio_init_all().

#include "pico/stdlib.h"
#include "pico/time.h"

// Register headers for the SdCard driver's direct hardware pokes
// (TIMER0 alarm IRQ, TIMER0 clock-gate bits in CLOCKS_SLEEP_EN1/WAKE_EN1).
#include "hardware/regs/intctrl.h"
#include "hardware/clocks.h"

typedef uint8_t u8;
typedef int8_t s8;
typedef uint16_t u16;
typedef int16_t s16;
typedef uint32_t u32;
typedef int32_t s32;
typedef uint64_t u64;
typedef int64_t s64;

typedef volatile uint8_t vu8;
typedef volatile int8_t vs8;
typedef volatile uint16_t vu16;
typedef volatile int16_t vs16;
typedef volatile uint32_t vu32;
typedef volatile int32_t vs32;
typedef volatile uint64_t vu64;
typedef volatile int64_t vs64;

#define SD_USE_SDIO

// MicroSD card socket (SIM1), software-driven SDIO on GPIO3-8.
#define SDIO_CLK 3
#define SDIO_CMD 4
#define SDIO_D0  5
#define SDIO_D1  6
#define SDIO_D2  7
#define SDIO_D3  8
#define SDIO_PIN_MASK  ((1u << SDIO_D3) | (1u << SDIO_D2) | (1u << SDIO_D1) | (1u << SDIO_D0) | \
                        (1u << SDIO_CMD) | (1u << SDIO_CLK))

// Firmware log output on the debug UART (UART1 on the USB_DP/DM pads). The
// debug firmware always enables ENABLE_UART_LOG at build time so the reused
// drivers' LOG() calls (e.g. the PSRAM data-path line) reach the UART.
#ifdef ENABLE_UART_LOG
#include <stdio.h>
#define LOG(...)    printf(__VA_ARGS__)
#else
#define LOG(...)    ((void)0)
#endif

static inline uint millis(void)
{
    return us_to_ms(time_us_64());
}

#ifdef __cplusplus
#include "sd/SdCard.h"
extern SdCard gSdCard;
#endif
