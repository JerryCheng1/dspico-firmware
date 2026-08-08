#include "common.h"
#include <stdio.h>
#include "hardware/structs/scb.h"
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

// Background full-chip PSRAM test. psram_init() only does a quick presence
// probe; the exhaustive test (write + verify the whole 8 MiB with an
// address-dependent pattern) runs here, one chunk per romCacheUpdate() call,
// so boot is not blocked and the cartridge protocol stays responsive. The
// cache stays unavailable until every chunk has verified. The chunk is kept
// small: on the bit-bang fallback path a 4 KiB chunk takes ~1 ms per main
// loop iteration, which starves the file API badly enough that the loader's
// SD mount times out.
#define ROM_CACHE_TEST_CHUNK_BYTES  512

// The test does not start immediately: each step runs PSRAM bursts with
// interrupts briefly disabled, and during the first seconds after boot the
// loader is mounting the SD card over the cart bus - the added IRQ latency
// breaks that. Give the loader time to finish before testing.
#define ROM_CACHE_TEST_DELAY_MS     10000

static bool sTestPending;
static bool sTestRunning;

// The main loop sleeps in __wfi() when the cart bus is idle, so without help
// the test would only advance when the console happens to send commands (at
// the launcher menu: never). A repeating timer on TIMER_IRQ_1 (the SD driver
// owns TIMER_IRQ_0) wakes the loop every 2 ms while the test is pending or
// running.
static alarm_pool_t* sWakePool;
static repeating_timer_t sWakeTimer;

static bool romCacheWakeTick(repeating_timer_t* t)
{
    (void)t;
    return true; // the IRQ itself is the product: it wakes the main loop
}

static void romCacheWakeTimerStart(void)
{
    if (!sWakePool)
        sWakePool = alarm_pool_create(1, 4);
    alarm_pool_add_repeating_timer_ms(sWakePool, 2, romCacheWakeTick, NULL, &sWakeTimer);
    // pwr_initPowerSaving() deep-sleeps with only PIO0 clocked, which stops
    // the hardware timer; drop to light sleep while the test needs wakeups.
    // romCacheInit() must therefore run after pwr_initPowerSaving().
    scb_hw->scr &= ~M0PLUS_SCR_SLEEPDEEP_BITS;
}

static void romCacheWakeTimerStop(void)
{
    if (sWakePool)
        cancel_repeating_timer(&sWakeTimer);
    scb_hw->scr |= M0PLUS_SCR_SLEEPDEEP_BITS; // restore deep sleep
}
static bool sTestWritePhase;
static u32 sTestAddr;
static u32 sTestStart;
static u8 sTestBuf[ROM_CACHE_TEST_CHUNK_BYTES] __attribute__((aligned(4)));

// Address-dependent byte pattern: unique per 4 KiB page, so address-aliasing
// and inter-cell coupling faults surface.
static inline u8 romCacheTestPattern(u32 addr)
{
    return (u8)(addr + (addr & 0xFFu) * 0x9Du + 0x35u) ^ (u8)(addr >> 12);
}

static void romCacheTestStep(void)
{
    if (sTestWritePhase)
    {
        for (u32 i = 0; i < ROM_CACHE_TEST_CHUNK_BYTES; i++)
            sTestBuf[i] = romCacheTestPattern(sTestAddr + i);
        psram_write(sTestAddr, sTestBuf, ROM_CACHE_TEST_CHUNK_BYTES);
    }
    else
    {
        psram_read(sTestAddr, sTestBuf, ROM_CACHE_TEST_CHUNK_BYTES);
        for (u32 i = 0; i < ROM_CACHE_TEST_CHUNK_BYTES; i++)
        {
            if (sTestBuf[i] != romCacheTestPattern(sTestAddr + i))
            {
                LOG("PSRAM: full-chip test FAILED @0x%08lX, ROM cache stays disabled\n",
                    sTestAddr + i);
                sTestRunning = false;
                romCacheWakeTimerStop();
                return;
            }
        }
    }

    sTestAddr += ROM_CACHE_TEST_CHUNK_BYTES;
    if (sTestAddr < PSRAM_SIZE_BYTES)
        return;

    if (sTestWritePhase)
    {
        sTestWritePhase = false; // whole chip written -> verify it
        sTestAddr = 0;
    }
    else
    {
        sTestRunning = false;
        sCacheAvailable = true;
        romCacheWakeTimerStop();
        LOG("PSRAM: full-chip test OK (%lu ms), ROM cache enabled (%u KB, %u lines)\n",
            (u32)(millis() - sTestStart),
            PSRAM_SIZE_BYTES / 1024, (u32)ROM_CACHE_NUM_LINES);
    }
}

void romCacheInit(void)
{
    romCacheInvalidate();
    sHits = 0;
    sMisses = 0;
    sCacheAvailable = false;
    sTestRunning = false;
    sTestPending = false;
    if (psram_init())
    {
        sTestPending = true;
        sTestWritePhase = true;
        sTestAddr = 0;
        sTestStart = millis();
        romCacheWakeTimerStart();
        LOG("PSRAM: detected, full-chip test starts in %u s...\n",
            ROM_CACHE_TEST_DELAY_MS / 1000);
    }
    else
    {
        LOG("PSRAM ROM cache: not detected, disabled\n");
    }
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
    if (sTestPending)
    {
        if ((u32)(millis() - sTestStart) < ROM_CACHE_TEST_DELAY_MS)
            return;
        sTestPending = false;
        sTestRunning = true;
        sTestStart = millis();
        LOG("PSRAM: running background full-chip test...\n");
    }
    if (sTestRunning)
    {
        // Catch up on missed wakeups: spend at most ~1 ms per call so the
        // cart protocol stays responsive.
        u32 until = time_us_32() + 1000;
        do
        {
            romCacheTestStep();
        } while (sTestRunning && (int)(time_us_32() - until) < 0);
    }

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
    LOG("ROM cache: hits=%u misses=%u hitrate=%u.%u%%\n",
        sHits, sMisses, permille / 10, permille % 10);

    sLastPrintedHits = sHits;
    sLastPrintedMisses = sMisses;
    sLastPrintTime = time_us_64();
}

#endif
