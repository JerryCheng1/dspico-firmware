/*
 * DSpico RP2354A board definition.
 *
 * RP2354A uses the RP2350A QFN-60 pinout and contains a stacked 2 MiB
 * Winbond W25Q16JV-compatible flash. The firmware uses a 12 MHz crystal.
 */

#ifndef _BOARDS_DSPICO_RP2354A_H
#define _BOARDS_DSPICO_RP2354A_H

pico_board_cmake_set(PICO_PLATFORM, rp2350)

#define DSPICO_RP2354A 1
#define PICO_RP2350A 1

// The stacked W25Q16JV supports the Winbond quad-I/O boot2 sequence.
#define PICO_BOOT_STAGE2_CHOOSE_W25Q080 1

#ifndef PICO_FLASH_SPI_CLKDIV
#define PICO_FLASH_SPI_CLKDIV 2
#endif

pico_board_cmake_set_default(PICO_FLASH_SIZE_BYTES, (2 * 1024 * 1024))
#ifndef PICO_FLASH_SIZE_BYTES
#define PICO_FLASH_SIZE_BYTES (2 * 1024 * 1024)
#endif

#ifndef PICO_XOSC_STARTUP_DELAY_MULTIPLIER
#define PICO_XOSC_STARTUP_DELAY_MULTIPLIER 1
#endif

// Production RP2354A devices use the fixed A4 silicon revision.
pico_board_cmake_set_default(PICO_RP2350_A2_SUPPORTED, 0)
#ifndef PICO_RP2350_A2_SUPPORTED
#define PICO_RP2350_A2_SUPPORTED 0
#endif

#endif
