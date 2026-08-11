#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "romCache.h"

static uint8_t sPsram[PSRAM_SIZE_BYTES];
static bool sInvalidateDuringRead;
static bool sInvalidateDuringWrite;
static bool sCorruptDuringRead;
static u32 sInvalidateSector;
static uint64_t sNowUs;
static bool sInterruptsDisabled;
static bool sAsyncPsramPending;
static bool sAsyncPsramCompletionReady;
static u32 sAsyncPsramAddr;
static u32 sAsyncPsramLen;
static const void* sAsyncPsramBuf;

// Loader-protocol counters normally owned by ntrCardRomGameSd.cpp. The host
// cache test links romCache.c without the cartridge emulation translation unit.
volatile u32 gCartSdE4Polls;
volatile u32 gCartSdE4Ready;
volatile u32 gCartSdE5Reads;

void uartLogPrintf(const char* format, ...)
{
    (void)format;
}

bool psram_core1_async_write(u32 addr, const void* buf, u32 len)
{
    if (sAsyncPsramPending)
        return false;
    sAsyncPsramAddr = addr;
    sAsyncPsramBuf = buf;
    sAsyncPsramLen = len;
    sAsyncPsramCompletionReady = false;
    sAsyncPsramPending = true;
    return true;
}

bool psram_core1_async_finish(void)
{
    if (!sAsyncPsramPending)
        return false;
    // Model a real worker: the first main-loop poll observes it in flight;
    // the second completes the transfer and releases the source buffer.
    if (!sAsyncPsramCompletionReady)
    {
        sAsyncPsramCompletionReady = true;
        return false;
    }
    psram_write(sAsyncPsramAddr, sAsyncPsramBuf, sAsyncPsramLen);
    sAsyncPsramPending = false;
    return true;
}

bool psram_core1_async_idle(void)
{
    return !sAsyncPsramPending;
}

static void require(bool condition, const char* message)
{
    if (!condition)
    {
        fprintf(stderr, "FAIL: %s\n", message);
        exit(1);
    }
}

uint64_t time_us_64(void)
{
    sNowUs += 1000;
    return sNowUs;
}

uint32_t save_and_disable_interrupts(void)
{
    require(!sInterruptsDisabled, "interrupt mask unexpectedly nested");
    sInterruptsDisabled = true;
    return 0;
}

void restore_interrupts(uint32_t state)
{
    (void)state;
    require(sInterruptsDisabled, "interrupt mask restored without save");
    sInterruptsDisabled = false;
}

void psram_init_hw(void)
{
}

bool psram_probe(void)
{
    return true;
}

void psram_read(u32 addr, void* dst, u32 len)
{
    require(addr + len <= PSRAM_SIZE_BYTES, "PSRAM read out of range");
    memcpy(dst, &sPsram[addr], len);
    if (sCorruptDuringRead)
    {
        sCorruptDuringRead = false;
        ((uint8_t*)dst)[0] ^= 0x80;
    }
    if (sInvalidateDuringRead)
    {
        sInvalidateDuringRead = false;
        romCacheSdInvalidateSector(sInvalidateSector);
    }
}

void psram_write(u32 addr, const void* src, u32 len)
{
    require(addr + len <= PSRAM_SIZE_BYTES, "PSRAM write out of range");
    memcpy(&sPsram[addr], src, len);
    if (sInvalidateDuringWrite)
    {
        sInvalidateDuringWrite = false;
        romCacheSdInvalidateSector(sInvalidateSector);
    }
}

static void fillAndDrain(u32 sector, uint8_t value)
{
    uint8_t data[SD_CACHE_LINE_SIZE];
    memset(data, value, sizeof(data));
    romCacheSdStore(sector, data);
    romCacheSdStoreDrain();
    romCacheSdStoreDrain();
}

int main(void)
{
    uint8_t out[SD_CACHE_LINE_SIZE];

    romCacheInit();
    romCacheSdInit();
    require(romCacheCore1Poll(), "PSRAM probe did not run");
    require(romCacheIsAvailable(), "cache not enabled after successful probe");

    // Startup must be genuinely PSRAM-quiet: sectors offered during the
    // mount/open window are dropped, not backfilled behind the hit gate.
    fillAndDrain(9, 0x09);
    for (int i = 0; i < 4000; i++)
        romCacheUpdate();
    require(!romCacheSdCheckHit(9),
            "startup quiet window unexpectedly backfilled PSRAM");

    fillAndDrain(10, 0x10);
    require(romCacheSdCheckHit(10), "stored sector was not a cache hit");
    require(romCacheSdReadCached(10, out), "stored sector could not be read");
    require(out[0] == 0x10 && out[sizeof(out) - 1] == 0x10,
            "stored sector data was corrupted");

    // Regression: invalidating after E5 queued a backfill but before the main
    // loop drained it must not let that old backfill resurrect the cache tag.
    uint8_t queued[SD_CACHE_LINE_SIZE];
    memset(queued, 0x20, sizeof(queued));
    romCacheSdStore(20, queued);
    romCacheSdInvalidateSector(20);
    romCacheSdStoreDrain();
    romCacheSdStoreDrain();
    require(!romCacheSdCheckHit(20),
            "pre-drain invalidation was overwritten by stale backfill");

    // Regression: the write IRQ may arrive while the bit-bang store drain is
    // in progress. The drain must not publish its tag after returning.
    memset(queued, 0x21, sizeof(queued));
    romCacheSdStore(21, queued);
    sInvalidateSector = 21;
    sInvalidateDuringWrite = true;
    romCacheSdStoreDrain();
    romCacheSdStoreDrain();
    require(!romCacheSdCheckHit(21),
            "mid-drain invalidation was overwritten by stale backfill");

    // Regression: a cached read invalidated during the PSRAM transfer must be
    // rejected instead of being exposed to E4/E5 as ready data.
    fillAndDrain(30, 0x30);
    sInvalidateSector = 30;
    sInvalidateDuringRead = true;
    require(!romCacheSdReadCached(30, out),
            "read invalidated in flight was incorrectly published");

    // A PSRAM transfer that returns corrupt data must invalidate the line and
    // force the cartridge path to fall back to physical SD.
    fillAndDrain(40, 0x40);
    require(romCacheSdCheckHit(40), "corruption test line was not cached");
    sCorruptDuringRead = true;
    require(!romCacheSdReadCached(40, out),
            "corrupt PSRAM data was incorrectly accepted");
    require(!romCacheSdCheckHit(40),
            "corrupt PSRAM line was not invalidated");

    puts("rom cache consistency tests passed");
    return 0;
}
