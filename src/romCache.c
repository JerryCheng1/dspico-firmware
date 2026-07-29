#include "common.h"
#include <stdio.h>
#include "romCache.h"

#ifdef ENABLE_PSRAM_ROM_CACHE

static_assert((ROM_CACHE_NUM_LINES & (ROM_CACHE_NUM_LINES - 1)) == 0,
    "ROM_CACHE_NUM_LINES must be a power of two");

// Direct mapped cache: block address b goes to line (b >> 14) % NUM_LINES,
// the line data lives at PSRAM offset lineIdx * 16 KB.
static bool sCacheAvailable;
static u32 sTags[ROM_CACHE_NUM_LINES];
static u32 sHits;
static u32 sMisses;

void romCacheInit(void)
{
    romCacheInvalidate();
    sHits = 0;
    sMisses = 0;
    sCacheAvailable = psram_init();
    printf("PSRAM ROM cache: %s (%u KB, %u lines)\n",
        sCacheAvailable ? "enabled" : "not detected, disabled",
        PSRAM_SIZE_BYTES / 1024, ROM_CACHE_NUM_LINES);
}

bool romCacheIsAvailable(void)
{
    return sCacheAvailable;
}

void romCacheInvalidate(void)
{
    for (u32 i = 0; i < ROM_CACHE_NUM_LINES; i++)
        sTags[i] = 0xFFFFFFFF;
}

bool romCacheFetch(u32 blockAddr, u8* dst)
{
    if (!sCacheAvailable)
        return false;

    u32 lineIdx = (blockAddr >> ROM_CACHE_LINE_SHIFT) & (ROM_CACHE_NUM_LINES - 1);
    if (sTags[lineIdx] != blockAddr)
    {
        sMisses++;
        return false;
    }

    psram_read(lineIdx * ROM_CACHE_LINE_SIZE, dst, ROM_CACHE_LINE_SIZE);
    sHits++;
    return true;
}

void romCacheStore(u32 blockAddr, const u8* src)
{
    if (!sCacheAvailable)
        return;

    u32 lineIdx = (blockAddr >> ROM_CACHE_LINE_SHIFT) & (ROM_CACHE_NUM_LINES - 1);
    psram_write(lineIdx * ROM_CACHE_LINE_SIZE, src, ROM_CACHE_LINE_SIZE);
    sTags[lineIdx] = blockAddr;
}

void romCacheUpdate(void)
{
    if (!sCacheAvailable)
        return;

    // print at most once every 5 seconds, and only when the counters moved
    static u32 sLastPrintedHits = 0;
    static u32 sLastPrintedMisses = 0;
    static u64 sLastPrintTime = 0;
    if ((sHits == sLastPrintedHits && sMisses == sLastPrintedMisses) ||
        time_us_64() - sLastPrintTime < 5000000)
    {
        return;
    }

    u32 total = sHits + sMisses;
    // hit rate in permille, printed with one decimal
    u32 permille = total != 0 ? (sHits * 1000 + total / 2) / total : 0;
    printf("ROM cache: hits=%u misses=%u hitrate=%u.%u%%\n",
        sHits, sMisses, permille / 10, permille % 10);

    sLastPrintedHits = sHits;
    sLastPrintedMisses = sMisses;
    sLastPrintTime = time_us_64();
}

#endif
