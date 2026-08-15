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

/// @brief Returns true when the startup PSRAM probe/qualification has reached
///        a terminal result (available, absent, failed, or probe-only). The
///        bit-bang build waits for this before enabling the cartridge PIO so
///        startup PSRAM traffic can never overlap the NDSL mount handshake.
bool romCacheQualificationFinished(void);


// ---------------------------------------------------------------------------
// SD sector cache (for the E3/E4/E5 block-device path used by pico-loader).
// pico-loader reads SD sectors directly via the E3/E4/E5 commands. This
// direct-mapped 512 B/line cache fills the whole 8 MB PSRAM and is consulted
// on E3 (hit -> serve from PSRAM, skip the SD read) and backfilled on E5 (a
// sector served from the SD is stored for re-reads).
//
// All PSRAM access is ASYNC: the E5 IRQ handler only does a short memcpy and
// sets a pending flag; romCacheSdStoreDrain() submits the actual write to
// core1 without waiting. Core0 continues advancing the SDIO state machine and
// publishes the cache tag only after a later completion poll. Doing the slow
// PSRAM access inside the IRQ handler, or synchronously waiting for it on the
// main loop, starves loader-time SD progress and can cause "failed to mount SD
// card". See romCache.c and PSRAM-PIO-HANDOVER.md for the full rationale.
//
// The HIT path is async too: romCacheSdCheckHit() (E3 IRQ) only reads the tag
// array and increments counters (~1 us, no PSRAM access); the actual
// psram_read runs in ntrc_sdCacheFetchDrain() on the main loop. The STORE
// path is async via romCacheSdStore()/romCacheSdStoreDrain() as below.
//
// The tag table (sSdTags) lives in SRAM, NOT PSRAM: the E3 IRQ handler must
// resolve a hit/miss in nanoseconds (one LDR). A PSRAM tag read would cost a
// full external-memory transaction inside PIO0_IRQ_0 -
// exactly the blackout that breaks the cart protocol. 16384 lines * 4 B =
// 64 KB of SRAM, plus a 32 KB 16-bit integrity-digest table. The data
// (16384 * 512 B = 8 MB) fills the PSRAM. A digest mismatch invalidates the
// line and forces a physical-SD fallback before E4 reports it ready.
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
///        Main-loop ONLY (the transfer runs on core1 while core0 remains
///        IRQ-ready). Returns false to use physical SD if an async backfill
///        already owns the core1 worker;
///        must NOT be called from PIO0_IRQ_0. The caller must have confirmed a
///        hit via romCacheSdCheckHit() immediately before (single sector in
///        flight on the main loop guarantees the tag stays valid).
/// @param sector The SD LBA.
/// @param dst Destination buffer of SD_CACHE_LINE_SIZE bytes.
/// @return true on success, false if the tag was evicted (should not happen).
bool romCacheSdReadCached(u32 sector, u8* dst);

/// @brief Queues the 512 B SD sector at \p sector for async storage.
///        Called from the E5 PIO0_IRQ_0 handler: copies \p src into an
///        internal buffer and marks it pending. romCacheSdStoreDrain() later
///        submits a non-blocking core1 write. If a previous store is still
///        pending or in flight the new one is dropped.
/// @param sector The SD LBA.
/// @param src Source buffer of SD_CACHE_LINE_SIZE bytes.
void romCacheSdStore(u32 sector, const u8* src);

/// @brief Submits or polls a pending asynchronous core1 SD-sector store. Call
///        from the core0 main loop (e.g. romCacheUpdate); never waits and must
///        NOT be called from PIO0_IRQ_0.
void romCacheSdStoreDrain(void);

/// @brief Begins an SD write transaction. Safe from PIO0_IRQ_0. New cache
///        hits/backfills are blocked immediately and a queued, not-yet-started
///        backfill is cancelled. The caller must wait for
///        romCacheSdWriteBarrierReady() before starting the physical SD write.
void romCacheSdWriteBegin(void);

/// @brief Polls the SD write barrier. Returns true only after every PSRAM
///        operation that was already running at romCacheSdWriteBegin() has
///        completed and any unpublished backfill has been discarded. Never
///        consumes a synchronous core1 cache-read completion owned by another
///        context. Safe to poll from PIO0_IRQ_0 or the core0 main loop.
bool romCacheSdWriteBarrierReady(void);

/// @brief Ends an SD write transaction and allows cache hits/backfills again.
///        Call only after the physical SD write (including a sequential chain)
///        has completed.
void romCacheSdWriteEnd(void);

/// @brief Invalidates all SD sector cache lines.
void romCacheSdInvalidate(void);

/// @brief Invalidates the single cache line that maps to \p sector. Safe to
///        call from PIO0_IRQ_0 (two short SRAM stores, no PSRAM access). It
///        also prevents an in-flight read/backfill from republishing the line
///        after invalidation. MUST be called whenever a sector is written
///        out-of-band (the F6 and R4 write paths write the SD directly), so a
///        subsequent E3 read cannot hit stale pre-write data.
/// @param sector The SD LBA that was (or is being) written.
void romCacheSdInvalidateSector(u32 sector);

/// @brief Prints hit/miss statistics to the UART when they changed.
///        Rate limited; call repeatedly from the main loop.
void romCacheUpdate(void);

#ifdef __cplusplus
}
#endif
