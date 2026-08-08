#pragma once
#include "common.h"
#include "psram.h"

#ifdef __cplusplus
extern "C" {
#endif

// The cache mirrors the granularity of the R4 ROM read path: one line holds
// one 16 KB ROM block (the size of sR4RomBlockLarge).
#define ROM_CACHE_LINE_SHIFT    14
#define ROM_CACHE_LINE_SIZE     (1u << ROM_CACHE_LINE_SHIFT)
#define ROM_CACHE_NUM_LINES     (PSRAM_SIZE_BYTES / ROM_CACHE_LINE_SIZE)

/// @brief Initializes the PSRAM and starts the background full-chip test.
///        When no working PSRAM is detected the cache stays unavailable and
///        all fetches miss.
void romCacheInit(void);

/// @brief Returns true once the background full-chip test has passed.
///        Until then (or when no PSRAM is fitted) all fetches miss and ROM
///        data is served from the SD card directly.
bool romCacheIsAvailable(void);

/// @brief Invalidates all cache lines. Must be called when the ROM changes.
void romCacheInvalidate(void);

/// @brief Tries to serve the 16 KB ROM block at \p blockAddr from the cache.
/// @param blockAddr The ROM address of the block. Must be 16 KB aligned.
/// @param dst Destination buffer of ROM_CACHE_LINE_SIZE bytes.
/// @return true on a cache hit (\p dst filled), false on a miss.
bool romCacheFetch(u32 blockAddr, u8* dst);

/// @brief Stores the 16 KB ROM block at \p blockAddr in the cache.
/// @param blockAddr The ROM address of the block. Must be 16 KB aligned.
/// @param src Source buffer of ROM_CACHE_LINE_SIZE bytes.
void romCacheStore(u32 blockAddr, const u8* src);

/// @brief Prints hit/miss statistics to the UART when they changed.
///        Rate limited; call repeatedly from the main loop.
void romCacheUpdate(void);

#ifdef __cplusplus
}
#endif
