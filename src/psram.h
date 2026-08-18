#pragma once
#include "common.h"

#ifdef __cplusplus
extern "C" {
#endif

// 4x APS6404L QSPI PSRAM (U2/U4/U5/U6), software driven.
// See references/hardware/RP2354A_GPIO分配总结.md (Netlist_Schematic1_1_2026-08-18).
// The four chips share SCLK and the 4-bit SIO bus; each has its own CE#.
#define PSRAM_PIN_CLK   0               // SCLK -> pin 6 of all four chips (R12, 22R)
#define PSRAM_PIN_IO0   22              // SIO0 -> pin 5 of all four chips
#define PSRAM_PIN_IO1   23              // SIO1 -> pin 2 of all four chips
#define PSRAM_PIN_IO2   24              // SIO2 -> pin 3 of all four chips
#define PSRAM_PIN_IO3   25              // SIO3 -> pin 7 of all four chips
#define PSRAM_PIN_CE0   26              // CE# of U2 (R3, 4.7k pull-up)
#define PSRAM_PIN_CE1   27              // CE# of U4 (R4, 4.7k pull-up)
#define PSRAM_PIN_CE2   28              // CE# of U5 (R5, 4.7k pull-up)
#define PSRAM_PIN_CE3   29              // CE# of U6 (R13, 4.7k pull-up)

#define PSRAM_CHIP_COUNT            4
#define PSRAM_CE_ALL_MASK   ((0xFu << PSRAM_PIN_CE0))

#define PSRAM_PIN_MASK  (PSRAM_CE_ALL_MASK | \
                         (1u << PSRAM_PIN_IO0) | (1u << PSRAM_PIN_IO1) | \
                         (1u << PSRAM_PIN_IO2) | (1u << PSRAM_PIN_IO3) | \
                         (1u << PSRAM_PIN_CLK))

// Per-chip capacity: each APS6404L-3SQR is 64 Mbit = 8 MiB.
#define PSRAM_CHIP_SIZE_BYTES   (8 * 1024 * 1024)
// Total linear address space: chip N occupies [N * 8 MiB, (N+1) * 8 MiB).
#define PSRAM_SIZE_BYTES        (PSRAM_CHIP_COUNT * PSRAM_CHIP_SIZE_BYTES)

#define PSRAM_CHIP_OF(addr)     ((addr) / PSRAM_CHIP_SIZE_BYTES)
#define PSRAM_ADDR_IN_CHIP(addr) ((addr) % PSRAM_CHIP_SIZE_BYTES)

/// @brief Initializes all PSRAM chips and verifies them with write/read
///        self-tests. Every chip gets its own reset, EID read and self-test.
/// @return true when at least one chip responds (see psram_chip_ok_mask).
bool psram_init(void);

/// @brief Bitmask of chips that passed the self-test (bit0 = CE0 ... bit3 = CE3).
///        Valid after psram_init(). 0xF = the full 32 MiB array is healthy.
u8 psram_chip_ok_mask(void);

/// @brief Reads \p len bytes from PSRAM \p addr into \p buf.
///        \p addr addresses the linear 32 MiB space (chip = addr / 8 MiB).
void psram_read(u32 addr, void* buf, u32 len);

/// @brief Writes \p len bytes from \p buf to PSRAM \p addr.
void psram_write(u32 addr, const void* buf, u32 len);

/// @brief Rework aid: walks a static probe level across the PSRAM pins when no
///        chip responds. Call repeatedly from the main loop (no-op timing-wise
///        between steps). See implementation for details.
void psram_pin_probe_step(void);

#ifdef __cplusplus
}
#endif
