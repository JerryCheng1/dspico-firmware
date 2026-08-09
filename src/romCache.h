#pragma once
#include "common.h"
#include "psram.h"

#ifdef __cplusplus
extern "C" {
#endif

/// @brief Initializes the PSRAM hardware only (no probe). The probe and
///        full-chip test run on core1 via romCacheCore1Poll().
void romCacheInit(void);

/// @brief core1 entry: run the PSRAM probe + full-chip test one step per call,
///        in scrambler-idle time. Returns true if it did PSRAM work.
bool romCacheCore1Poll(void);

/// @brief Returns true once the PSRAM probe passed (and the full-chip test, if
///        enabled). Until then (or when no PSRAM is fitted) all SD cache
///        lookups miss and sectors are served from the SD card directly.
bool romCacheIsAvailable(void);


// ---------------------------------------------------------------------------
// SD sector cache (for the E3/E4/E5 block-device path used by pico-loader).
// pico-loader reads SD sectors directly via the E3/E4/E5 commands. This
// direct-mapped 512 B/line cache fills the whole 8 MB PSRAM and is consulted
// on E3 (hit -> serve from PSRAM, skip the SD read) and backfilled on E5 (a
// sector served from the SD is stored for re-reads).
//
// All PSRAM access is ASYNC: the E5 IRQ handler only does a ~1 us memcpy and
// sets a pending flag; the actual psram_write runs in romCacheSdStoreDrain()
// on the core0 main loop, which PIO0_IRQ_0 can preempt. Doing the ~112 us /
// 1 ms PSRAM access inside the IRQ handler blocks PIO0_IRQ_0 (SM0 stages the
// next command word in its RX FIFO but the handler has not returned) and the
// NDS stalls -> "failed to mount SD card". The bit-bang path (no pio0 SM,
// no spinlock, no IRQ disable) is safe on core0 and is used for ALL cache
// access; the PIO pump is used only for the core1 probe (where the spinlock
// disabling interrupts is harmless - it disables core1's IRQs, not core0's).
// See romCache.c for the full rationale.
//
// The HIT path is async too: romCacheSdCheckHit() (E3 IRQ) only reads the tag
// array and increments counters (~1 us, no PSRAM access); the actual
// psram_read runs in ntrc_sdCacheFetchDrain() on the main loop. The STORE
// path is async via romCacheSdStore()/romCacheSdStoreDrain() as below.
//
// The tag table (sSdTags) lives in SRAM, NOT PSRAM: the E3 IRQ handler must
// resolve a hit/miss in nanoseconds (one LDR). A PSRAM tag read would cost a
// full ~10-20 us bit-bang transaction (cmd+addr+dummy) inside PIO0_IRQ_0 -
// exactly the blackout that breaks the cart protocol. 16384 lines * 4 B =
// 64 KB of SRAM; the data (16384 * 512 B = 8 MB) fills the PSRAM.
#define SD_CACHE_LINE_SIZE   512
#define SD_CACHE_LINE_SHIFT  9
#define SD_CACHE_NUM_LINES   (PSRAM_SIZE_BYTES / SD_CACHE_LINE_SIZE) // 16384

/// @brief Zeros the SD sector cache tags. Called once at boot after romCacheInit.
void romCacheSdInit(void);

/// @brief Tag-only check for the SD sector at \p sector. Safe to call from the
///        E3 PIO0_IRQ_0 handler: reads only the tag array and increments
///        sSdHits/sSdMisses - NO PSRAM access. On a hit the caller must arrange
///        for the data to be fetched async (see ntrc_sdCacheFetchDrain).
/// @param sector The SD LBA.
/// @return true if the sector is cached (hit), false on a miss.
bool romCacheSdCheckHit(u32 sector);

/// @brief Reads the cached 512 B SD sector at \p sector from PSRAM into \p dst.
///        Main-loop ONLY (bit-bang psram_read, ~64 us, preemptible, no lock);
///        must NOT be called from PIO0_IRQ_0. The caller must have confirmed a
///        hit via romCacheSdCheckHit() immediately before (single sector in
///        flight on the main loop guarantees the tag stays valid).
/// @param sector The SD LBA.
/// @param dst Destination buffer of SD_CACHE_LINE_SIZE bytes.
/// @return true on success, false if the tag was evicted (should not happen).
bool romCacheSdReadCached(u32 sector, u8* dst);

/// @brief Queues the 512 B SD sector at \p sector for async storage.
///        Called from the E5 PIO0_IRQ_0 handler: copies \p src into an
///        internal buffer and marks it pending. The actual psram_write is
///        performed later by romCacheSdStoreDrain() on the main loop. If a
///        previous store is still pending the new one is dropped.
/// @param sector The SD LBA.
/// @param src Source buffer of SD_CACHE_LINE_SIZE bytes.
void romCacheSdStore(u32 sector, const u8* src);

/// @brief Performs any pending async SD-sector store (psram_write). Call from
///        the core0 main loop (e.g. romCacheUpdate); must NOT be called from
///        PIO0_IRQ_0. Preemptible by the cart IRQ.
void romCacheSdStoreDrain(void);

/// @brief Invalidates all SD sector cache lines.
void romCacheSdInvalidate(void);

/// @brief Prints hit/miss statistics to the UART when they changed.
///        Rate limited; call repeatedly from the main loop.
void romCacheUpdate(void);

#ifdef __cplusplus
}
#endif
