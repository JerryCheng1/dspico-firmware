#include "common.h"
#include <inttypes.h>
#include <string.h>
#include "hardware/sync.h"
#include "romCache.h"

#ifdef ENABLE_PSRAM_CACHE

// When 1, core1 runs an 8 MB write+verify full-chip test after the probe
// before enabling the cache. The probe already validates the data path
// (write+read+compare at 3 addresses with retries), so the expensive full-chip
// pass is OFF by default and the cache is enabled on probe success.
#ifndef PSRAM_FULL_CHIP_TEST
#define PSRAM_FULL_CHIP_TEST 0
#endif

// When 1 (and PSRAM_FULL_CHIP_TEST is 0), a successful probe immediately
// enables the cache. When 0, the probe still runs (confirming the PSRAM is
// present and the data path works) but sCacheAvailable stays false, so the SD
// cache stays disabled and every sector is served from SD - identical to the
// nopsram build. This isolates whether breakage comes from the probe bursts /
// core1 activity (breaks even at 0) or from cache use (only breaks at 1).
// Default 1 = cache enabled on probe success.
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

// sCacheAvailable gates the SD sector cache (the only cache layer): until the
// core1 probe passes, romCacheSdCheckHit/Store are no-ops and every E3/E5 is
// served from the SD card directly.
static volatile bool sCacheAvailable;

// The PSRAM probe + full-chip test run on core1 (see romCacheCore1Poll).
// Shared state between core1 (writer) and core0 (reader):
static volatile bool sProbeDone;        // core1 sets when probe finished
static volatile bool sProbeOk;          // core1 sets: probe succeeded
static volatile bool sQualificationDone;// core1 sets at terminal qualification
static volatile bool sTestFailed;       // core1 sets: a test chunk mismatched
static volatile u32  sTestFailAddr;     // core1 sets: failing address
#if PSRAM_FULL_CHIP_TEST
static volatile u32  sTestAddr;         // progress, for logging
#endif
static volatile u32  sTestStart;
static volatile u32  sTestElapsedMs;

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
    sCacheAvailable = false;
    sProbeDone = false;
    sProbeOk = false;
    sQualificationDone = false;
    sTestFailed = false;
    sTestFailAddr = 0;
#if PSRAM_FULL_CHIP_TEST
    sTestAddr = 0;
#endif
    sTestStart = 0;
    sTestElapsedMs = 0;
    sHwInitDone = false;
    // Hardware init + selected backend. Safe during boot (busy_wait, no data
    // bursts). The probe/test is driven by core1 after this returns.
    psram_init_hw();
    // PSRAM GPIO/reset/PIO are now ready: release core1 to probe. core1 may
    // already be parked in WFE inside romCacheCore1Poll() waiting for this.
    __dmb();
    sHwInitDone = true;
    __sev();
}

bool romCacheIsAvailable(void)
{
    return sCacheAvailable;
}

bool romCacheQualificationFinished(void)
{
    __dmb();
    return sQualificationDone;
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
// is triggered by pis_sm0_rx_fifo_not_empty and must return quickly enough for
// SM0 to stage the next NDS response. Even with the IRQ-safe PIO backend, a
// synchronous 512 B transfer inside the handler would delay later commands.
//
// Both cache paths are therefore ASYNC: the E3/E5 IRQ handlers do only a
// short tag check / memcpy and set a pending flag. The core0 main loop starts
// the operation from ntrc_sdCacheFetchDrain / romCacheSdStoreDrain. Every
// backend queues the transfer to core1. Hit reads wait with core0 cartridge
// IRQs enabled; backfills are submitted without waiting so core0 keeps
// advancing SDIO. Thus a bit-bang CE#-low burst is never stretched by a
// cartridge interrupt. On PIO1, SDIO SM0/SM1 and PSRAM SM2 enable bits use
// atomic CTRL aliases.
#ifndef SD_CACHE_ENABLE
#define SD_CACHE_ENABLE 1
#endif
#ifndef SD_CACHE_STORE
#define SD_CACHE_STORE 1
#endif

// UART heartbeat is enabled in log builds, but its used-line count is O(1):
// never restore the old 16384-entry volatile tag scan on the core0 main loop.
#ifndef PSRAM_CACHE_HEARTBEAT_LOG
#define PSRAM_CACHE_HEARTBEAT_LOG 1
#endif

// pico-loader's initial mount/open phase has a tight E4 ready deadline. Keep
// BOTH PSRAM reads and backfills completely idle for this startup window. A
// real cold-boot trace showed that even a non-blocking 69 us core1 backfill
// doubled the early E4 poll count (166 -> 308) and the loader stopped before
// the next E5. After this quiet window, stores and hits are enabled together.
#ifndef SD_CACHE_IO_WARMUP_MS
#define SD_CACHE_IO_WARMUP_MS 3000u
#endif
// Tags and generations are touched by both the core0 main loop and
// PIO0_IRQ_0, so they must be volatile. Each write-around SD update bumps the
// mapped line's generation. A PSRAM read/backfill may publish its result only
// if the generation is unchanged across the slow PSRAM operation; otherwise
// it would resurrect data invalidated while that operation was in flight.
// One byte per line keeps the metadata affordable on RP2040; wrapping would
// require 256 writes to the same mapped line before one ~64 us operation can
// finish, which the serialized cartridge protocol cannot produce.
static volatile u32 sSdTags[SD_CACHE_NUM_LINES];
static volatile u8 sSdLineGenerations[SD_CACHE_NUM_LINES];
// A 16-bit content digest prevents a marginal PSRAM read/write from ever
// being served to the loader. 32 KB is the largest integrity table that fits
// comfortably beside the 64 KB tag table in RP2040 SRAM.
static u16 sSdChecksums[SD_CACHE_NUM_LINES];
static volatile u32 sSdUsedLines;
volatile u32 sSdHits;
volatile u32 sSdMisses;
static volatile u32 sSdVerifyFailures;
// Only the one-byte gate is read in PIO0_IRQ_0. It gates both the E3 hit check
// and E5 store queue. The deadline and all timer arithmetic stay on core0.
static volatile bool sCacheIoEnabled;
static u32 sCacheIoEnableAtMs;

// Async store state. E5's IRQ handler copies the served sector into
// sAsyncStoreBuf and sets sAsyncStorePending; the main loop drains it with the
// real psram_write. Single-slot: if the main loop has not drained the previous
// store when E5 offers another, the new one is dropped (the line simply stays
// unfilled and is re-served from SD next time - correctness is unaffected).
static u8 sAsyncStoreBuf[SD_CACHE_LINE_SIZE] __attribute__((aligned(4)));
static volatile u32 sAsyncStoreSector;
static volatile u8 sAsyncStoreGeneration;
static volatile bool sAsyncStorePending;
// The store-drain state is module-wide rather than function-local because an
// SD write barrier must distinguish its own asynchronous backfill from a
// synchronous cache read using the same core1 worker. Only an owned backfill
// may be completed with psram_core1_async_finish(); consuming a synchronous
// read's DONE state would leave psramSubmitCore1() waiting forever.
static volatile bool sAsyncStoreInFlight;
static u16 sAsyncStoreChecksum;
static u64 sFirstStoreStartUs;
static bool sLoggedFirstStore;

// Set before any physical SD write. While active, E3 is forced to miss, E5
// drops backfills, and a write cannot start until the core1 PSRAM worker is
// idle. This makes the cache a read-through/write-around cache with a real
// write transaction boundary rather than tag invalidation alone.
static volatile bool sSdWriteBarrierActive;

// Fast 16-bit folded FNV-1a digest. This runs only on the core0 main loop,
// never in PIO0_IRQ_0. It is not cryptographic; it is an integrity guard for
// PSRAM signal/timing faults before data is exposed through E5.
static u16 sdCacheChecksum(const u8* data)
{
    u32 hash = 2166136261u;
    for (u32 i = 0; i < SD_CACHE_LINE_SIZE; i++)
        hash = (hash ^ data[i]) * 16777619u;
    return (u16)(hash ^ (hash >> 16));
}

void romCacheSdInit(void)
{
    sAsyncStorePending = false;
    sAsyncStoreInFlight = false;
    sAsyncStoreChecksum = 0;
    sFirstStoreStartUs = 0;
    sLoggedFirstStore = false;
    sSdWriteBarrierActive = false;
    for (u32 i = 0; i < SD_CACHE_NUM_LINES; i++)
    {
        sSdTags[i] = 0xFFFFFFFF;
        sSdLineGenerations[i] = 0;
    }
    sSdHits = 0;
    sSdMisses = 0;
    sSdVerifyFailures = 0;
    sCacheIoEnableAtMs = 0;
    sCacheIoEnabled = SD_CACHE_IO_WARMUP_MS == 0;
    sSdUsedLines = 0;
}

void romCacheSdInvalidate(void)
{
    for (u32 i = 0; i < SD_CACHE_NUM_LINES; i++)
    {
        sSdTags[i] = 0xFFFFFFFF;
        sSdLineGenerations[i]++;
    }
    sSdUsedLines = 0;
}

void romCacheSdInvalidateSector(u32 sector)
{
#if !SD_CACHE_ENABLE && !SD_CACHE_STORE
    return;
#else
    // Drop any cached copy and cancel publication by a PSRAM read/backfill
    // already in flight for this direct-mapped line. Both are short SRAM
    // stores, so this remains safe in PIO0_IRQ_0. Without the generation bump,
    // a slow store drain preempted by an SD write could set the old tag again
    // after this function returned, resurrecting pre-write data.
    u32 lineIdx = sector & (SD_CACHE_NUM_LINES - 1);
    if (sSdTags[lineIdx] != 0xFFFFFFFF && sSdUsedLines > 0)
        sSdUsedLines--;
    sSdTags[lineIdx] = 0xFFFFFFFF;
    sSdLineGenerations[lineIdx]++;
#endif
}

// Invalidates a direct-mapped line after an abandoned in-flight backfill. The
// PSRAM data has already been overwritten, so an older tag for the same line
// must not remain valid even though the new tag was never published.
static void sdCacheInvalidateLine(u32 lineIdx)
{
    if (sSdTags[lineIdx] != 0xFFFFFFFF && sSdUsedLines > 0)
        sSdUsedLines--;
    sSdTags[lineIdx] = 0xFFFFFFFF;
    sSdLineGenerations[lineIdx]++;
}

bool romCacheSdCheckHit(u32 sector)
{
#if !SD_CACHE_ENABLE
    sSdMisses++;
    return false;
#else
    // Gate first so the warmup miss path is one SRAM byte load + counter,
    // matching the minimal probe-only path as closely as possible.
    if (!sCacheIoEnabled || !sCacheAvailable || sSdWriteBarrierActive)
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
    // actual psram_read. Only the tag array and counters are touched
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
    if (sSdWriteBarrierActive)
        return false;

    u32 lineIdx = sector & (SD_CACHE_NUM_LINES - 1);
    u8 generation = sSdLineGenerations[lineIdx];
    u16 expectedChecksum = sSdChecksums[lineIdx];
    // Re-check the tag: the only tag mutator is the store drain, which also
    // runs on the core0 main loop and is serialized with this drain (single
    // thread). With one sector in flight between E3 and E5, no eviction can
    // land between the E3 hit check and this read - but guard anyway.
    if (sSdTags[lineIdx] != sector)
        return false;

    // Main loop only. Every backend runs on core1 while core0 waits with
    // PIO0_IRQ_0 enabled.
    // E4 polls report not-ready until the caller publishes the filled buffer.
    // Do not synchronously queue behind a backfill already running on core1.
    // Fall back to physical SD instead; the caller keeps E4 not-ready until
    // that read completes. This prevents a cache hit from stalling core0's SD
    // state machine behind unrelated PSRAM work.
    if (!psram_core1_async_idle())
        return false;

    static bool sLoggedFirstFetch;
    bool traceFirstFetch = !sLoggedFirstFetch;
    u64 fetchStartUs = 0;
    if (traceFirstFetch)
    {
        sLoggedFirstFetch = true;
        fetchStartUs = time_us_64();
        LOG("[cache] first hit fetch begin sector=%" PRIu32 " on core1\n",
            sector);
    }
    psram_read(lineIdx * SD_CACHE_LINE_SIZE, dst, SD_CACHE_LINE_SIZE);
    u16 actualChecksum = sdCacheChecksum(dst);
    if (traceFirstFetch)
        LOG("[cache] first hit fetch done in %" PRIu64
            " us bursts=%" PRIu32 " checksum=%04" PRIX32 "/%04" PRIX32
            "\n", time_us_64() - fetchStartUs,
            psram_core1_last_burst_count(), (u32)actualChecksum,
            (u32)expectedChecksum);

    // An SD write may have preempted the bit-bang read and invalidated this
    // line. Validate and return while IRQs are briefly masked so the check is
    // a coherent snapshot. The mask covers only SRAM loads, not PSRAM I/O.
    uint32_t irqState = save_and_disable_interrupts();
    bool metadataValid = sSdLineGenerations[lineIdx] == generation &&
                         sSdTags[lineIdx] == sector;
    bool checksumValid = actualChecksum == expectedChecksum;
    bool stillValid = !sSdWriteBarrierActive &&
                      metadataValid && checksumValid;
    if (metadataValid && !checksumValid)
    {
        if (sSdUsedLines > 0)
            sSdUsedLines--;
        sSdTags[lineIdx] = 0xFFFFFFFF;
        sSdLineGenerations[lineIdx]++;
        sSdVerifyFailures++;
    }
    restore_interrupts(irqState);
    if (metadataValid && !checksumValid)
        LOG("[cache] VERIFY FAIL sector=%" PRIu32
            " expected=%04" PRIX32 " actual=%04" PRIX32
            "; invalidated, fallback to SD\n", sector,
            (u32)expectedChecksum, (u32)actualChecksum);
    return stillValid;
#endif
}

void romCacheSdStore(u32 sector, const u8* src)
{
#if !SD_CACHE_STORE
    return;
#else
    // During pico-loader mount/open, make the runtime PSRAM data path exactly
    // as quiet as the proven probe-only firmware. Do not even copy/queue the
    // sector: it can be cached normally if requested again after warmup.
    if (!sCacheIoEnabled || !sCacheAvailable || sSdWriteBarrierActive)
        return;

    // Drop if the previous store is still being drained: overwriting
    // sAsyncStoreBuf mid-psram_write would corrupt the cached line.
    if (sAsyncStorePending)
        return;

    u32 lineIdx = sector & (SD_CACHE_NUM_LINES - 1);
    u8 generation = sSdLineGenerations[lineIdx];
    memcpy(sAsyncStoreBuf, src, SD_CACHE_LINE_SIZE);
    sAsyncStoreSector = sector;
    sAsyncStoreGeneration = generation;
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
    u8 generation = sAsyncStoreGeneration;
    u32 lineIdx = sector & (SD_CACHE_NUM_LINES - 1);
    // Runs on the core0 main loop, NOT in PIO0_IRQ_0. Core0 submits the
    // request to core1 and remains available for the next E3/E4/E5 IRQ while
    // core1 owns the uninterrupted PSRAM transaction.
    if (!sAsyncStoreInFlight)
    {
        if (sSdWriteBarrierActive || !psram_core1_async_idle())
            return;
        u16 checksum = sdCacheChecksum(sAsyncStoreBuf);

        // A write command may preempt checksum calculation. Reserve the
        // worker and mark ownership atomically with respect to that IRQ.
        uint32_t submitIrqState = save_and_disable_interrupts();
        if (sSdWriteBarrierActive || !sAsyncStorePending ||
            !psram_core1_async_idle() ||
            !psram_core1_async_write(lineIdx * SD_CACHE_LINE_SIZE,
                                     sAsyncStoreBuf, SD_CACHE_LINE_SIZE))
        {
            restore_interrupts(submitIrqState);
            return;
        }
        sAsyncStoreChecksum = checksum;
        sAsyncStoreInFlight = true;
        restore_interrupts(submitIrqState);
        if (!sLoggedFirstStore)
        {
            sLoggedFirstStore = true;
            sFirstStoreStartUs = time_us_64();
            LOG("[cache] first async backfill begin sector=%" PRIu32
                " checksum=%04" PRIX32
                " on core1; core0 continues SDIO\n", sector,
                (u32)sAsyncStoreChecksum);
        }
    }

    // The core1 completion event wakes core0. Until then return immediately so
    // the main loop can call gSdCard.Update() for an E3 that arrived while the
    // write was in flight.
    if (!psram_core1_async_finish())
        return;
    sAsyncStoreInFlight = false;
    if (sFirstStoreStartUs != 0)
    {
        LOG("[cache] first async backfill done in %" PRIu64 " us\n",
            time_us_64() - sFirstStoreStartUs);
        sFirstStoreStartUs = 0;
    }

    // Publish the tag only if no write-around path invalidated this line while
    // psram_write was running. Mask IRQs for the generation check + tag store
    // so invalidation cannot land between them and be overwritten. This is a
    // handful of SRAM instructions; the slow PSRAM transfer already completed
    // asynchronously on core1.
    uint32_t irqState = save_and_disable_interrupts();
    if (!sSdWriteBarrierActive &&
        sSdLineGenerations[lineIdx] == generation)
    {
        if (sSdTags[lineIdx] == 0xFFFFFFFF)
            sSdUsedLines++;
        sSdChecksums[lineIdx] = sAsyncStoreChecksum;
        sSdTags[lineIdx] = sector;
    }
    else if (sSdWriteBarrierActive)
    {
        // The backfill overwrote this PSRAM line after the write transaction
        // started, but its tag must never become visible. Also discard any old
        // tag whose data occupied the same direct-mapped line.
        sdCacheInvalidateLine(lineIdx);
    }
    sAsyncStorePending = false; // release only after write/publication finishes
    restore_interrupts(irqState);
#endif
}

void romCacheSdWriteBegin(void)
{
#if !SD_CACHE_ENABLE && !SD_CACHE_STORE
    return;
#else
    uint32_t irqState = save_and_disable_interrupts();
    sSdWriteBarrierActive = true;
    __dmb();
    // A queued store has not touched PSRAM and can be dropped immediately.
    // An in-flight store retains its source buffer and is drained/discarded by
    // romCacheSdWriteBarrierReady().
    if (sAsyncStorePending && !sAsyncStoreInFlight)
        sAsyncStorePending = false;
    restore_interrupts(irqState);
#endif
}

bool romCacheSdWriteBarrierReady(void)
{
#if !SD_CACHE_ENABLE && !SD_CACHE_STORE
    return true;
#else
    if (!sSdWriteBarrierActive)
        return true;

    if (sAsyncStoreInFlight)
    {
        // This completion belongs to our asynchronous backfill, so it is safe
        // to consume. Do not use async_finish() for an unowned synchronous
        // read: its waiting psramSubmitCore1() must release that request.
        if (!psram_core1_async_finish())
            return false;

        u32 lineIdx = sAsyncStoreSector & (SD_CACHE_NUM_LINES - 1);
        uint32_t irqState = save_and_disable_interrupts();
        sdCacheInvalidateLine(lineIdx);
        sAsyncStoreInFlight = false;
        sAsyncStorePending = false;
        sFirstStoreStartUs = 0;
        restore_interrupts(irqState);
    }

    // If this is false with no owned async store, a synchronous cache fetch
    // was preempted by the write command. Leave DONE untouched; after its main
    // loop waiter resumes it will set the worker to IDLE and this becomes true.
    return !sAsyncStoreInFlight && psram_core1_async_idle();
#endif
}

void romCacheSdWriteEnd(void)
{
#if !SD_CACHE_ENABLE && !SD_CACHE_STORE
    return;
#else
    __dmb();
    sSdWriteBarrierActive = false;
#endif
}


// Called by core1 in its scrambler-idle time. Runs the PSRAM probe, then the
// full-chip test one chunk per call. Returns true if it did PSRAM work (so
// core1 can decide to yield). All bursts here run on core1. The PIO transaction
// lock never changes either core's PRIMASK, so core0's cart IRQ keeps firing.
//
// The probe runs as soon as sHwInitDone goes true (romCacheInit does the SEV
// that wakes core1). In the production bit-bang build, main calls romCacheInit
// and waits for qualification before enabling cartridge PIO0: no startup PSRAM
// burst can then overlap NDSL's mount handshake. The experimental PIO1 backend
// still initializes after SDIO has established its final instruction layout.
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
    __dmb(); // acquire psram_init_hw() state published by core0

    if (sTestFailed || sCacheAvailable)
        return false; // terminal: nothing more to do

    if (!sC1Probed)
    {
        // romCacheInit() has configured the selected backend and explicitly
        // released core1. Production main waits for this call to finish before
        // cartridge PIO0 becomes active.
        sC1Probed = true;
        sTestStart = millis();
        sProbeOk = psram_probe();
        __dmb();
        sProbeDone = true;
        __sev();
        if (sProbeOk)
        {
#if PSRAM_FULL_CHIP_TEST
            sC1TestWritePhase = true;
            sTestAddr = 0;
#elif PSRAM_CACHE_ENABLE_ON_PROBE
            // Probe validated the data path (write+read+compare at 3
            // addresses with retries). Skip the redundant full-chip pass and
            // enable the cache on probe success alone.
            sCacheAvailable = true;
#else
            // Diagnostic: probe ran (PSRAM confirmed present) but the cache
            // is NOT enabled. The SD cache stays disabled, so every sector is
            // served from SD - identical to the nopsram build. If the
            // menu/game still breaks with this, the culprit is the probe
            // bursts themselves (or core1's activity), not cache use.
#endif
        }
#if !PSRAM_FULL_CHIP_TEST
        sTestElapsedMs = millis() - sTestStart;
        __dmb();
        sQualificationDone = true;
        __sev();
#else
        else
        {
            sTestElapsedMs = millis() - sTestStart;
            __dmb();
            sQualificationDone = true;
            __sev();
        }
#endif
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
                sTestElapsedMs = millis() - sTestStart;
                __dmb();
                sQualificationDone = true;
                __sev();
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
    sTestElapsedMs = millis() - sTestStart;
    __dmb();
    sQualificationDone = true;
    __sev();
    return true;
#else
    return false; // cache enabled on probe success; nothing more to do
#endif
}

#if defined(ENABLE_UART_LOG) && PSRAM_CACHE_HEARTBEAT_LOG
// Hit rate as permille (0..1000) so it can be printed as N.N%.
static u32 hitRatePermille(u32 hits, u32 misses)
{
    u32 total = hits + misses;
    return total != 0 ? (hits * 1000 + total / 2) / total : 0;
}

#endif

// core0: forward core1's probe/test outcome to logs and cache state.
void romCacheUpdate(void)
{
    // Keep timer reads and 64-bit microsecond-to-millisecond conversion out of
    // PIO0_IRQ_0. E3/E5 see only sCacheIoEnabled and remain the same
    // constant-time no-cache paths as the proven probe-only firmware.
    if (!sCacheIoEnabled && sCacheAvailable)
    {
        u32 nowMs = millis();
        if (sCacheIoEnableAtMs == 0)
            sCacheIoEnableAtMs = nowMs + SD_CACHE_IO_WARMUP_MS;
        else if ((s32)(nowMs - sCacheIoEnableAtMs) >= 0)
            sCacheIoEnabled = true;
    }

    // Submit/poll one pending SD-sector store after updating the gate. The
    // transfer runs asynchronously on core1; this call never waits for it.
    romCacheSdStoreDrain();

#ifdef ENABLE_UART_LOG
    // The probe runs on core1 immediately when sHwInitDone goes true:
    // romCacheInit() does the __sev that wakes core1, so no periodic nudge is
    // needed here. (The main loop is __wfi-driven, but that only matters for
    // the scrambler-idle parking after the probe, which is woken by
    // secureCmd1Handler's __sev when game mode engages the scrambler.)

#if PSRAM_CACHE_HEARTBEAT_LOG
    // Optional heartbeat: report live cache counters. Time-based (not
    // tick-based)
    // because the main loop only runs when an IRQ wakes it (PIO0_IRQ_0 on each
    // cart read), which can be only a few times/sec once a game runs from NDS
    // RAM - a tick counter would take hours to fire.
    //
    // The SD sector cache is the only cache layer: 512 B/line, 16384 lines,
    // filling the whole 8 MB PSRAM. ACTIVE under pico-loader (its hits/misses
    // grow as it re-reads FAT/dir/font/etc.).
    extern volatile u32 sSdHits, sSdMisses;
    static u64 sBeatLastUs;
    static bool sLoggedIoEnable;
    u64 beatNow = time_us_64();

    if (!sLoggedIoEnable && sCacheIoEnableAtMs != 0 && sCacheIoEnabled)
    {
        sLoggedIoEnable = true;
        LOG("[cache] read/backfill paths enabled after %" PRIu32
            " ms mount/open quiet window\n", (u32)SD_CACHE_IO_WARMUP_MS);
    }

    if (beatNow - sBeatLastUs >= 3000000)
    {
        sBeatLastUs = beatNow;

        u32 sdRate = hitRatePermille(sSdHits, sSdMisses);
        // O(1) snapshot maintained when tags become valid/invalid. Do not scan
        // the 16384-entry volatile tag table from this timing-sensitive loop.
        u32 usedLines = (u32)sSdUsedLines;
        u32 capKb = (SD_CACHE_NUM_LINES * SD_CACHE_LINE_SIZE) / 1024; // 8 MB
        u32 usedKb = (usedLines * SD_CACHE_LINE_SIZE) / 1024;
        u32 freeKb = capKb - usedKb;
        // Keep UART diagnostics ASCII-only. Many serial terminals default to
        // a legacy Windows code page and render UTF-8 Chinese as mojibake.
        LOG("[cache] SD hit-rate=%" PRIu32 ".%" PRIu32
            "%% verifyfail=%" PRIu32 " | used=%" PRIu32 "/%" PRIu32
            " lines (%" PRIu32 "KB) free=%" PRIu32 "KB/%" PRIu32 "KB\n",
            sdRate / 10, sdRate % 10,
            (u32)sSdVerifyFailures, usedLines, (u32)SD_CACHE_NUM_LINES,
            usedKb, freeKb, capKb);
    }
#endif

    static bool sLoggedProbe;
    if (!sLoggedProbe && sProbeDone)
    {
        sLoggedProbe = true;
        if (sProbeOk)
#if PSRAM_FULL_CHIP_TEST
            LOG("PSRAM: detected, full-chip test running on core1...\n");
#elif PSRAM_CACHE_ENABLE_ON_PROBE
            LOG("PSRAM: detected (startup probe complete on core1)\n");
#else
            LOG("PSRAM: detected (probe OK), cache NOT enabled (diagnostic)\n");
#endif
        else
            LOG("PSRAM: not detected, cache disabled\n");
    }
    if (sCacheAvailable)
    {
        static bool sLoggedOk;
        if (!sLoggedOk)
        {
            sLoggedOk = true;
            // One-time summary of what the cache uses and how to read the
            // heartbeat. The SD cache is the only layer; it fills the PSRAM.
            u32 sdCapKb = (SD_CACHE_NUM_LINES * SD_CACHE_LINE_SIZE) / 1024;
            LOG("PSRAM: probe OK (%lu ms), %" PRIu32 " KB chip\n",
                (unsigned long)sTestElapsedMs,
                (u32)(PSRAM_SIZE_BYTES / 1024));
            LOG("PSRAM: SD cache enabled - %" PRIu32 " lines x %" PRIu32
                "B = %" PRIu32 "KB (E3/E5 sector cache, fills chip)\n",
                (u32)SD_CACHE_NUM_LINES, (u32)SD_CACHE_LINE_SIZE, sdCapKb);
            LOG("[cache] startup policy: no reads/backfills for %" PRIu32
                " ms mount/open quiet window\n", (u32)SD_CACHE_IO_WARMUP_MS);
            LOG("[cache] legend: hit/miss=E3/E5; used=valid lines; verifyfail=digest mismatch -> SD fallback\n");
        }
    }
    else if (sTestFailed)
    {
        static bool sLoggedFail;
        if (!sLoggedFail)
        {
            sLoggedFail = true;
            LOG("PSRAM: full-chip test FAILED @0x%08" PRIX32
                ", cache stays disabled\n", (u32)sTestFailAddr);
        }
    }
#endif
}

#endif
