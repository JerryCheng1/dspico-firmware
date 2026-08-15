#pragma once
#include "common.h"

// Production and standalone builds use the proven bit-bang backend. CMake can
// explicitly select the experimental PIO backend for diagnostics.
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

/// @brief Hardware init only (GPIO and reset). No bursts/probe. Bit-bang is
///        independent and may initialize before cartridge/SDIO. The PIO1
///        backend must initialize after physical SD setup establishes its
///        final SM/instruction layout, and falls back if SM2 is unavailable.
void psram_init_hw(void);

/// @brief Probes PSRAM through the selected backend. Run on core1.
bool psram_probe(void);

/// @brief hw init + probe. Only when probe's bursts are safe (not during boot).
bool psram_init(void);

/// @brief Reads \p len bytes from PSRAM \p addr into \p buf.
void psram_read(u32 addr, void* buf, u32 len);

/// @brief Writes \p len bytes from \p buf to PSRAM \p addr.
void psram_write(u32 addr, const void* buf, u32 len);

/// @brief Starts one non-blocking core1 write. The source buffer must remain
///        valid until psram_core1_async_finish() returns true.
/// @return true when the request was accepted; false while the worker is busy
///         or has not started yet.
bool psram_core1_async_write(u32 addr, const void* buf, u32 len);

/// @brief Completes an accepted asynchronous request and releases the worker.
/// @return true once core1 has finished the transfer; false while still busy.
bool psram_core1_async_finish(void);

/// @brief Returns whether no asynchronous/synchronous core1 request is active.
bool psram_core1_async_idle(void);

/// @brief Number of electrical bursts completed by the most recently
///        submitted core1 request. Intended for first-transfer diagnostics.
u32 psram_core1_last_burst_count(void);

/// @brief Core1 worker for all queued runtime transfers, including bit-bang.
///        Each call performs at most one bus burst so game-mode callers can
///        refill other timing-critical core1 work between bursts.
/// @return true if one burst of a queued transfer was serviced.
bool psram_core1_service(void);

#ifdef __cplusplus
}
#endif
