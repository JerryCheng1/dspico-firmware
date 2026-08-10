#pragma once
#include "common.h"

// The production configuration uses only SIO bit-banging for external PSRAM.
// CMake defines this explicitly; keep the safe default for standalone builds.
#ifndef PSRAM_FORCE_BITBANG
#define PSRAM_FORCE_BITBANG 1
#endif

#ifdef __cplusplus
extern "C" {
#endif

// APS6404L QSPI PSRAM, software driven on GPIO22-26/29.
// See reference/hardware/GPIO.md (DSPICOwithPSRAM).
#define PSRAM_PIN_IO0   22
#define PSRAM_PIN_IO1   23
#define PSRAM_PIN_IO2   24
#define PSRAM_PIN_IO3   25
#define PSRAM_PIN_CLK   26
#define PSRAM_PIN_CE    29

#define PSRAM_PIN_MASK  ((1u << PSRAM_PIN_IO0) | (1u << PSRAM_PIN_IO1) | \
                         (1u << PSRAM_PIN_IO2) | (1u << PSRAM_PIN_IO3) | \
                         (1u << PSRAM_PIN_CLK) | (1u << PSRAM_PIN_CE))

// Capacity of the fitted PSRAM: APS6404L-3SQR-ZR = 64 Mbit = 8 MB.
#define PSRAM_SIZE_BYTES    (8 * 1024 * 1024)

/// @brief Hardware init only (GPIO and reset). No bursts/probe.
///        Safe during NDS boot. PIO setup exists only in experimental builds.
void psram_init_hw(void);

/// @brief Initializes the pio0 serialization lock in experimental PIO builds.
///        The production bit-bang build does not call these helpers.
void psram_init_lock(void);

/// @brief Experimental PIO-build serialization helpers.
uint32_t psramPioLock(void);
void psramPioUnlock(uint32_t save);

/// @brief Probes PSRAM with bit-bang bursts. Run on core1.
bool psram_probe(void);

/// @brief hw init + probe. Only when probe's bursts are safe (not during boot).
bool psram_init(void);

/// @brief Reads \p len bytes from PSRAM \p addr into \p buf.
void psram_read(u32 addr, void* buf, u32 len);

/// @brief Writes \p len bytes from \p buf to PSRAM \p addr.
void psram_write(u32 addr, const void* buf, u32 len);

#ifdef __cplusplus
}
#endif
