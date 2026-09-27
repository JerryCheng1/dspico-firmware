#pragma once

// Four-chip 4 KiB L2 page directory. Each physical page reserves a following
// 32-byte CRC table (8 x CRC32). The last 4640 bytes of each 8 MiB chip remain
// outside the directory for qualification. This module owns metadata only;
// callers must complete and verify PSRAM data/table I/O before commit.

#include <stdbool.h>
#include <stdint.h>

#ifdef CACHE_SD_HOST
#define CACHE_PAGE_RAM_CODE
#else
#define CACHE_PAGE_RAM_CODE __attribute__((section(".time_critical.cache")))
#endif

#define CACHE_PAGE_CHIPS 4u
#define CACHE_PAGE_SETS 2031u
#ifndef CACHE_PAGE_ACTIVE_SETS
#define CACHE_PAGE_ACTIVE_SETS CACHE_PAGE_SETS
#endif
#define CACHE_PAGE_COUNT (CACHE_PAGE_CHIPS * CACHE_PAGE_SETS)
#define CACHE_PAGE_DATA_BYTES 4096u
#define CACHE_PAGE_TABLE_BYTES 32u
#define CACHE_PAGE_STRIDE_BYTES (CACHE_PAGE_DATA_BYTES + CACHE_PAGE_TABLE_BYTES)
#define CACHE_PAGE_CHIP_BYTES 8388608u
#define CACHE_PAGE_RESERVED_BYTES (CACHE_PAGE_CHIP_BYTES - CACHE_PAGE_SETS * CACHE_PAGE_STRIDE_BYTES)

typedef struct {
    uint32_t tag;       // logical SD page (sector >> 3)
    uint32_t epoch;     // media/write namespace
    uint32_t version;   // bumped on allocation or replacement
    uint32_t table_crc; // CRC32 of the 32-byte PSRAM checksum table
    uint32_t state;     // valid [7:0], busy [15:8], pin [16], CLOCK ref [17]
} cachePageEntry;

typedef struct {
    cachePageEntry entry[CACHE_PAGE_COUNT];
    uint8_t hand[(CACHE_PAGE_SETS + 3u) / 4u]; // packed 2-bit CLOCK hand/set
    uint32_t count_epoch;
    uint32_t valid_count;
    uint8_t usable_mask; // fixed at initialization; bit i enables physical chip i
} cachePageMap;

typedef struct {
    uint16_t slot;
    uint8_t lane;
    uint8_t had_page; // 1 if a previously valid table must be read/checked
    uint8_t is_pin;   // read pin, never a fill lease
    uint32_t version;
    uint32_t tag;
    uint32_t epoch;
} cachePageLease;

#ifdef __cplusplus
extern "C" {
#endif

CACHE_PAGE_RAM_CODE void cachePageMapInit(cachePageMap* m, uint8_t usable_mask);
// Returns only a fully committed, unbusy sector and pins the physical page.
// One reader per page is allowed; a successful lookup requires exactly one
// Unpin after PSRAM I/O. Other lookups and fills defer while pinned.
CACHE_PAGE_RAM_CODE bool cachePageMapLookup(cachePageMap* m, uint32_t sector, uint32_t epoch,
                        cachePageLease* out, uint32_t* table_crc);
CACHE_PAGE_RAM_CODE bool cachePageMapUnpin(cachePageMap* m, const cachePageLease* lease);
// Starts a fill, clearing the target lane before the caller touches PSRAM.
// A busy or pinned page cannot be selected for another operation.
CACHE_PAGE_RAM_CODE bool cachePageMapBegin(cachePageMap* m, uint32_t sector, uint32_t epoch,
                       cachePageLease* out);
CACHE_PAGE_RAM_CODE bool cachePageMapCommit(cachePageMap* m, const cachePageLease* lease,
                        uint32_t table_crc);
// A failed or cancelled table update invalidates the whole page; old CRCs
// must not make other lanes appear valid after partial table transport.
CACHE_PAGE_RAM_CODE bool cachePageMapAbort(cachePageMap* m, const cachePageLease* lease);
CACHE_PAGE_RAM_CODE bool cachePageMapLeaseCurrent(const cachePageMap* m, const cachePageLease* lease);
// Invalidates a committed page after read/table verification fails. A read
// lease must be unpinned first; no other reader may be active.
CACHE_PAGE_RAM_CODE bool cachePageMapInvalidate(cachePageMap* m, const cachePageLease* lease);
CACHE_PAGE_RAM_CODE uint32_t cachePageMapDataAddr(uint16_t slot, uint8_t lane);
CACHE_PAGE_RAM_CODE uint32_t cachePageMapTableAddr(uint16_t slot);
CACHE_PAGE_RAM_CODE uint8_t cachePageMapChip(uint16_t slot);

#ifdef __cplusplus
}
#endif
