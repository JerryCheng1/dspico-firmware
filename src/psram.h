#pragma once
#include "common.h"

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

/// @brief Initializes the PSRAM and verifies it with a write/read self-test.
/// @return true when the PSRAM is present and functional.
bool psram_init(void);

/// @brief Reads \p len bytes from PSRAM \p addr into \p buf.
void psram_read(u32 addr, void* buf, u32 len);

/// @brief Writes \p len bytes from \p buf to PSRAM \p addr.
void psram_write(u32 addr, const void* buf, u32 len);

#ifdef __cplusplus
}
#endif
