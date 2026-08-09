#include "common.h"
#include <stdio.h>
#include <string.h>
#include "hardware/structs/scb.h"
#include "hardware/sync.h"
#include "ntrCardRom.h"
#include "romCache.h"

#ifdef ENABLE_PSRAM_ROM_CACHE

static_assert((ROM_CACHE_NUM_LINES & (ROM_CACHE_NUM_LINES - 1)) == 0,
    "ROM_CACHE_NUM_LINES must be a power of two");

// When 1, core1 runs an 8 MB write+verify full-chip test after the probe
// before enabling the cache. The test's continuous bursts run during gameplay
// and stall core0's cart-reset IRQ (gpioIrq) on the pio0 ctrl spinlock,
// breaking the R4 B6 streaming read (white screen). The probe already
// validates the data path (write+read+compare at 3 addresses with retries),
// so the test is OFF by default and the cache is enabled on probe success.
#ifndef PSRAM_FULL_CHIP_TEST
#define PSRAM_FULL_CHIP_TEST 0
#endif

// When 1 (and PSRAM_FULL_CHIP_TEST is 0), a successful probe immediately
// enables the cache. When 0, the probe still runs (confirming the PSRAM is
// present and the data path works) but sCacheAvailable stays false, so
// romCacheFetch/Store are no-ops and ROM is served purely from SD -
// identical to the nopsram build. This isolates whether breakage comes from
// the probe bursts / core1 activity (breaks even at 0) or from cache use
// (only breaks at 1). Default 1 = cache enabled on probe success.
//
// The earlier "undefined instruction" on game entry with this at 0 was NOT
// cache use - it was a race between core0's USB PIN_IRQ toggle
// (usbEventQueue.c setCartridgeIrqState, the only runtime core0 SIO gpio_out
// write) and core1's non-atomic psramBbSendNibble RMW of gpio_out. USB is now
// removed from the board/build, so that race is gone and the cache is safe to
// enable.
#ifndef PSRAM_CACHE_ENABLE_ON_PROBE
#define PSRAM_CACHE_ENABLE_ON_PROBE 1
#endif

// Direct mapped cache: block address b goes to line (b >> 14) % NUM_LINES,
// the line data lives at PSRAM offset lineIdx * 16 KB.
static volatile bool sCacheAvailable;
static u32 sTags[ROM_CACHE_NUM_LINES];
static u32 sHits;
static u32 sMisses;

// The PSRAM probe + full-chip test run on core1 (see romCacheCore1Poll),
// keeping the IRQ-shielded/locking bursts off core0's cart-protocol path.
// Shared state between core1 (writer) and core0 (reader):
static volatile bool sProbeDone;        // core1 sets when probe finished
static volatile bool sProbeOk;          // core1 sets: probe succeeded
static volatile bool sTestFailed;       // core1 sets: a test chunk mismatched
static volatile u32  sTestFailAddr;     // core1 sets: failing address
#if PSRAM_FULL_CHIP_TEST
static volatile u32  sTestAddr;         // progress, for logging
#endif
static volatile u32  sTestStart;

// core0 sets sHwInitDone once psram_init_hw() has configured the PSRAM GPIO,
// reset the chip, and armed sUsePio. core1 must not probe before this: an
// early probe (core1 starts at multicore_launch_core1, ~300 ms before core0
// reaches romCacheInit past initSd) sees unconfigured pins / an un-reset chip
// and fails for good - sC1Probed latches true and the cache stays disabled.
static volatile bool sHwInitDone;

// One chunk of the full-chip test. Kept small so core1 makes progress in
// short bursts between scrambler ring fills.
#if PSRAM_FULL_CHIP_TEST
#define ROM_CACHE_TEST_CHUNK_BYTES  512

static u8 sTestBuf[ROM_CACHE_TEST_CHUNK_BYTES] __attribute__((aligned(4)));

static inline u8 romCacheTestPattern(u32 addr)
{
    return (u8)(addr + (addr & 0xFFu) * 0x9Du + 0x35u) ^ (u8)(addr >> 12);
}
#endif

void romCacheInit(void)
{
    romCacheInvalidate();
    sHits = 0;
    sMisses = 0;
    sCacheAvailable = false;
    sProbeDone = false;
    sProbeOk = false;
    sTestFailed = false;
    sTestFailAddr = 0;
#if PSRAM_FULL_CHIP_TEST
    sTestAddr = 0;
#endif
    sTestStart = 0;
    sHwInitDone = false;
    // Hardware init + pio0 ctrl spinlock. Safe during boot (busy_wait, no
    // bursts). The probe/test is driven by core1 after this returns.
    psram_init_hw();
    // PSRAM GPIO/reset/PIO are now ready: release core1 to probe. core1 may
    // already be parked in WFE inside romCacheCore1Poll() waiting for this.
    sHwInitDone = true;
    __sev();
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

// ---------------------------------------------------------------------------
// SD sector cache (E3/E4/E5 block-device path; see romCache.h).
// Direct-mapped: SD LBA sector S -> line S % SD_CACHE_NUM_LINES, line data at
// PSRAM offset line * 512. Counters are non-static so romCacheUpdate's beat
// can report them.
//
// SD_CACHE_ENABLE gates the hit path (E3 consults the tag array; on a tag
// match the 512 B is fetched from PSRAM). SD_CACHE_STORE gates the store
// path (E5 backfills a served sector into PSRAM).
//
// IMPORTANT: neither path may do its PSRAM access inside PIO0_IRQ_0. That IRQ
// is triggered by pis_sm0_rx_fifo_not_empty (SM0 pushes each NDS command word
// into its RX FIFO). The PIO pump path's psramPioLock() -> spin_lock_blocking()
// -> save_and_disable_interrupts() disables ALL interrupts on the calling core
// (PRIMASK), and the lock is held across psramTxSmWait() (the whole burst).
// On core0 that blacks out PIO0_IRQ_0 for every 32 B burst (~4 us each, 16x
// per 512 B) -> SM0's command words pile up unstaged -> "failed to mount SD
// card". The bit-bang path takes NO lock and never disables IRQs, so it is
// safe on core0. (The core1 probe can use PIO freely: spin_lock_blocking on
// core1 disables core1's IRQs, not core0's.)
//
// Both cache paths are therefore ASYNC and use the bit-bang data path: the
// E3/E5 IRQ handlers do only a ~1 us tag check / memcpy and set a pending
// flag; the actual psram_read/psram_write runs on the core0 main loop
// (ntrc_sdCacheFetchDrain / romCacheSdStoreDrain), which PIO0_IRQ_0 can
// preempt at any burst boundary. The IRQ handlers do not touch PSRAM, so they
// never contend for the pio0 ctrl spinlock or the async buffers.
#ifndef SD_CACHE_ENABLE
#define SD_CACHE_ENABLE 1
#endif
#ifndef SD_CACHE_STORE
#define SD_CACHE_STORE 1
#endif
static u32 sSdTags[SD_CACHE_NUM_LINES];
volatile u32 sSdHits;
volatile u32 sSdMisses;

void romCacheSdInit(void)
{
    romCacheSdInvalidate();
    sSdHits = 0;
    sSdMisses = 0;
}

void romCacheSdInvalidate(void)
{
    for (u32 i = 0; i < SD_CACHE_NUM_LINES; i++)
        sSdTags[i] = 0xFFFFFFFF;
}

bool romCacheSdCheckHit(u32 sector)
{
#if !SD_CACHE_ENABLE
    sSdMisses++;
    return false;
#else
    if (!sCacheAvailable)
    {
        sSdMisses++;
        return false;
    }

    u32 lineIdx = sector & (SD_CACHE_NUM_LINES - 1);
    if (sSdTags[lineIdx] != sector)
    {
        sSdMisses++;
        return false;
    }

    // Hit: tag matches. Do NOT read PSRAM here - this runs in the E3
    // PIO0_IRQ_0 handler. The caller (ntrc_gameReqSdReadCmd1) records the
    // pending fetch; ntrc_sdCacheFetchDrain() on the main loop does the
    // actual bit-bang psram_read. Only the tag array and counters are touched
    // here (~1 us, IRQ-safe).
    sSdHits++;
    return true;
#endif
}

bool romCacheSdReadCached(u32 sector, u8* dst)
{
#if !SD_CACHE_ENABLE
    return false;
#else
    u32 lineIdx = sector & (SD_CACHE_NUM_LINES - 1);
    // Re-check the tag: the only tag mutator is the store drain, which also
    // runs on the core0 main loop and is serialized with this drain (single
    // thread). With one sector in flight between E3 and E5, no eviction can
    // land between the E3 hit check and this read - but guard anyway.
    if (sSdTags[lineIdx] != sector)
        return false;

    // Bit-bang read (~64 us). Main loop only: preemptible by PIO0_IRQ_0, takes
    // no spinlock, disables no IRQs. The E4 polls that arrive during this read
    // report not-ready (the buffer is still marked invalid until the caller
    // fills sSdSectorBuffersSectors after this returns).
    psram_read(lineIdx * SD_CACHE_LINE_SIZE, dst, SD_CACHE_LINE_SIZE);
    return true;
#endif
}

// Async store state. E5's IRQ handler copies the served sector into
// sAsyncStoreBuf and sets sAsyncStorePending; the main loop drains it with the
// real psram_write. Single-slot: if the main loop has not drained the previous
// store when E5 offers another, the new one is dropped (the line simply stays
// unfilled and is re-served from SD next time - correctness is unaffected).
static u8 sAsyncStoreBuf[SD_CACHE_LINE_SIZE] __attribute__((aligned(4)));
static volatile u32 sAsyncStoreSector;
static volatile bool sAsyncStorePending;

void romCacheSdStore(u32 sector, const u8* src)
{
#if !SD_CACHE_STORE
    return;
#else
    if (!sCacheAvailable)
        return;

    // Drop if the previous store is still being drained: overwriting
    // sAsyncStoreBuf mid-psram_write would corrupt the cached line.
    if (sAsyncStorePending)
        return;

    memcpy(sAsyncStoreBuf, src, SD_CACHE_LINE_SIZE);
    sAsyncStoreSector = sector;
    sAsyncStorePending = true;
#endif
}

void romCacheSdStoreDrain(void)
{
#if !SD_CACHE_STORE
    return;
#else
    if (!sAsyncStorePending)
        return;

    u32 sector = sAsyncStoreSector;
    u32 lineIdx = sector & (SD_CACHE_NUM_LINES - 1);
    // Runs on the core0 main loop, NOT in PIO0_IRQ_0. The next E3/E4/E5 may
    // preempt this at a burst boundary, but those handlers do not touch PSRAM,
    // so they never contend for sAsyncStoreBuf or the pio0 ctrl spinlock.
    psram_write(lineIdx * SD_CACHE_LINE_SIZE, sAsyncStoreBuf, SD_CACHE_LINE_SIZE);
    sSdTags[lineIdx] = sector;
    sAsyncStorePending = false; // release the slot only after the write lands
#endif
}


// Called by core1 in its scrambler-idle time. Runs the PSRAM probe, then the
// full-chip test one chunk per call. Returns true if it did PSRAM work (so
// core1 can decide to yield). All bursts here run on core1; they use the pio0
// ctrl spinlock (not IRQ shielding), so core0's cart IRQ keeps firing.
//
// The probe runs as soon as sHwInitDone goes true (romCacheInit does the
// __sev that wakes core1). It bit-bangs only the PSRAM pins (GPIO22-26/29),
// which do not overlap the cartridge bus (GPIO9-21); the USB SIO gpio_out
// race that previously corrupted the bus during the probe is gone (USB
// removed from the board/build), so no boot delay is needed. (An earlier
// 3 s delay was for a since-refuted "probe corrupts NDS boot" hypothesis and
// had the side effect of gating the probe on cart-bus interrupts, since the
// core0 main loop is __wfi-driven and could not nudge core1 during a quiet
// loader menu - the probe only ran after the user navigated the menu.)
static bool sC1Probed;
#if PSRAM_FULL_CHIP_TEST
static bool sC1TestWritePhase;
#endif

bool romCacheCore1Poll(void)
{
    // Wait for core0 to finish psram_init_hw(): probing before the PSRAM GPIO
    // is configured and the chip reset fails for good (sC1Probed latches).
    if (!sHwInitDone)
        return false;

    if (sTestFailed || sCacheAvailable)
        return false; // terminal: nothing more to do

    if (!sC1Probed)
    {
        // Probe immediately: the PSRAM pins do not overlap the cartridge bus
        // and the USB gpio_out race is gone, so there is no boot window to
        // avoid. romCacheInit()'s __sev woke us; run the probe now.
        sC1Probed = true;
        sProbeOk = psram_probe();
        sProbeDone = true;
        sTestStart = millis();
        if (sProbeOk)
        {
#if PSRAM_FULL_CHIP_TEST
            sC1TestWritePhase = true;
            sTestAddr = 0;
#elif PSRAM_CACHE_ENABLE_ON_PROBE
            // Probe validated the data path (write+read+compare at 3
            // addresses with retries). Skip the full-chip test - its
            // continuous bursts run during gameplay and break the R4 B6
            // streaming read - and enable the cache on probe success alone.
            sCacheAvailable = true;
#else
            // Diagnostic: probe ran (PSRAM confirmed present) but the cache
            // is NOT enabled. romCacheFetch/Store stay no-ops, so the R4 B6
            // path serves ROM purely from SD - identical to the nopsram build.
            // If the menu/game still breaks with this, the culprit is the
            // probe bursts themselves (or core1's activity), not cache use.
#endif
        }
        return true;
    }
    if (!sProbeOk)
        return false; // no PSRAM: give up

#if PSRAM_FULL_CHIP_TEST
    // One test chunk.
    if (sC1TestWritePhase)
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
                sTestFailAddr = sTestAddr + i;
                sTestFailed = true;
                return true;
            }
        }
    }

    sTestAddr += ROM_CACHE_TEST_CHUNK_BYTES;
    if (sTestAddr < PSRAM_SIZE_BYTES)
        return true;

    if (sC1TestWritePhase)
    {
        sC1TestWritePhase = false; // whole chip written -> verify it
        sTestAddr = 0;
        return true;
    }
    // verify pass complete
    sCacheAvailable = true;
    return true;
#else
    return false; // cache enabled on probe success; nothing more to do
#endif
}

// core0: forward core1's probe/test outcome to logs and cache state.
void romCacheUpdate(void)
{
    // Drain any pending async SD-sector store BEFORE the heartbeat/logging so
    // backfills keep up with the E5 stream. This psram_write runs here on the
    // main loop (preemptible by PIO0_IRQ_0); see romCacheSdStoreDrain().
    romCacheSdStoreDrain();

    // The probe runs on core1 immediately when sHwInitDone goes true:
    // romCacheInit() does the __sev that wakes core1, so no periodic nudge is
    // needed here. (The main loop is __wfi-driven, but that only matters for
    // the scrambler-idle parking after the probe, which is woken by
    // secureCmd1Handler's __sev when game mode engages the scrambler.)

    // Heartbeat: report live cache counters. Time-based (not tick-based)
    // because the main loop only runs when an IRQ wakes it (PIO0_IRQ_0 on each
    // cart read), which can be only a few times/sec once a game runs from NDS
    // RAM - a tick counter would take hours to fire.
    extern volatile u32 sSdHits, sSdMisses;
    static u64 sBeatLastUs;
    u64 beatNow = time_us_64();
    if (beatNow - sBeatLastUs >= 3000000)
    {
        sBeatLastUs = beatNow;
        LOG("cache beat: avail=%d rom h/m=%u/%u sd h/m=%u/%u\n",
            (int)sCacheAvailable, (u32)sHits, (u32)sMisses,
            (u32)sSdHits, (u32)sSdMisses);
    }

    static bool sLoggedProbe;
    if (!sLoggedProbe && sProbeDone)
    {
        sLoggedProbe = true;
        if (sProbeOk)
#if PSRAM_FULL_CHIP_TEST
            LOG("PSRAM: detected, full-chip test running on core1...\n");
#elif PSRAM_CACHE_ENABLE_ON_PROBE
            LOG("PSRAM: detected, enabling ROM cache on core1...\n");
#else
            LOG("PSRAM: detected (probe OK), cache NOT enabled (diagnostic)\n");
#endif
        else
            LOG("PSRAM ROM cache: not detected, disabled\n");
    }
    if (sCacheAvailable)
    {
        static bool sLoggedOk;
        if (!sLoggedOk)
        {
            sLoggedOk = true;
#if PSRAM_FULL_CHIP_TEST
            LOG("PSRAM: full-chip test OK (%lu ms), ROM cache enabled (%u KB, %u lines)\n",
                (u32)(millis() - sTestStart),
                PSRAM_SIZE_BYTES / 1024, (u32)ROM_CACHE_NUM_LINES);
#else
            LOG("PSRAM: probe OK (%lu ms), ROM cache enabled (%u KB, %u lines)\n",
                (u32)(millis() - sTestStart),
                PSRAM_SIZE_BYTES / 1024, (u32)ROM_CACHE_NUM_LINES);
#endif
        }

        // Print hit/miss stats when they change. The first change prints
        // immediately: the main loop is __wfi-driven and goes quiet once a
        // game runs from NDS RAM, so a pure 5 s timer can be missed entirely
        // (the stats change during the sub-5 s load, then the loop stops
        // running before the 5 s mark). After the first print, rate-limit to
        // once every 5 s.
        static u32 sLastPrintedHits = 0;
        static u32 sLastPrintedMisses = 0;
        static u64 sLastPrintTime = 0;
        static bool sFirstStatPrint = true;
        if (sHits != sLastPrintedHits || sMisses != sLastPrintedMisses)
        {
            u64 now = time_us_64();
            if (sFirstStatPrint || now - sLastPrintTime >= 5000000)
            {
                sFirstStatPrint = false;
                u32 total = sHits + sMisses;
                u32 permille = total != 0 ? (sHits * 1000 + total / 2) / total : 0;
                LOG("ROM cache: hits=%u misses=%u hitrate=%u.%u%%\n",
                    sHits, sMisses, permille / 10, permille % 10);
                sLastPrintedHits = sHits;
                sLastPrintedMisses = sMisses;
                sLastPrintTime = now;
            }
        }
    }
    else if (sTestFailed)
    {
        static bool sLoggedFail;
        if (!sLoggedFail)
        {
            sLoggedFail = true;
            LOG("PSRAM: full-chip test FAILED @0x%08lX, ROM cache stays disabled\n",
                (u32)sTestFailAddr);
        }
    }
}

#endif

