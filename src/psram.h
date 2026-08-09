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

/// @brief Hardware init only (GPIO, reset, PIO config). No bursts/probe.
///        Safe during NDS boot. Also initializes the pio0 ctrl spinlock.
void psram_init_hw(void);

/// @brief Claims the pio0 ctrl spinlock used to serialize pio0->ctrl
///        read-modify-writes across cores. Must be called once before any
///        psramPioLock()/psram_init_hw() use - main() does this up front,
///        before resetNtrCard()/gpioIrq() (which take the lock) and before
///        core1 starts.
void psram_init_lock(void);

/// @brief Claims the pio0 ctrl spinlock (shared with core0's SM0 restarts).
///        Call before any pio_sm_set_enabled/restart on core0 IRQ paths.
uint32_t psramPioLock(void);
void psramPioUnlock(uint32_t save);

/// @brief Probes PSRAM with bursts. Run on core1 (bursts use the pio0
///        spinlock, not IRQ shielding, but still take core time).
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
