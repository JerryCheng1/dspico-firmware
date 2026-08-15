#include "common.h"
#include <stdio.h>
#include "hardware/sync.h"
#include "r4.h"
#include "ntrCardRom.h"
#include "ntrCardRomGameNoScramble.h"
#ifdef ENABLE_PSRAM_CACHE
#include "romCache.h"
#endif

// Physical SD reads and cartridge writes must never share storage. A cache-hit
// fetch runs synchronously on the core0 main loop but remains IRQ-preemptible;
// F6/R4 can therefore receive a write payload while that fetch is still using
// the read buffer. Keeping a dedicated double-buffer prevents the resumed
// PSRAM read from overwriting bytes that are about to be committed to SD.
static u8 sSdSectorBuf[1024] __attribute__((aligned(4)));
static u8 sSdWriteSectorBuf[1024] __attribute__((aligned(4)));
static u32 sCurSdSector = 0xFFFFFFFF;
static u32 sSdSectorBuffersSectors[2] = { 0xFFFFFFFF, 0xFFFFFFFF };
static u32 sBufferIndex = 0;
static u32 sWriteBufferIndex = 0;
static u32 sReadSector;
static bool sReadBusy = false;
// Own the LBA and destination buffer captured when a physical SDIO read is
// submitted. sReadSector/sBufferIndex describe the cartridge protocol and can
// advance in an IRQ while SDIO is still active; using either of those mutable
// values to label a completed DMA can publish sector A under sector B's tag.
static u32 sPhysicalReadSector = 0xFFFFFFFF;
static u32 sPhysicalReadBufferIndex = 0;
static bool sWriteBusy = false;
static bool sWritePendingStart = false;
static bool sCurrentWriteIsLast = false;
static u32 sCurrentWriteSector = 0xFFFFFFFF;
static u32 sCurrentWriteBufferIndex = 0;
static bool sNextWriteBlockQueued = false;
static bool sNextWriteIsLast = false;
static u32 sNextWriteSector = 0xFFFFFFFF;
static u32 sNextWriteBufferIndex = 0;

extern "C"
{
volatile u32 gCartSdE4Polls;
volatile u32 gCartSdE4Ready;
volatile u32 gCartSdE5Reads;
}

#ifdef ENABLE_PSRAM_CACHE
// The SD sector cache backfills sectors served from the SD card. E3 sets this
// to the sector it requested on a miss so E5 (which serves the data) knows
// which sector to store. 0xFFFFFFFF = no store pending (cache hit, or a write
// path). Kept separate from sReadSector because E5 serves sReadSector and then
// bumps it before we store, and double-buffering can interleave.
static volatile u32 sPendingStoreSector = 0xFFFFFFFF;

// Async cache-HIT state. On a hit E3 (PIO0_IRQ_0) only does a tag check - it
// must NOT psram_read there (that keeps the handler busy too long). It records
// sector here and leaves buffer 0 invalid; ntrc_sdCacheFetchDrain() on the
// main loop does the PSRAM read into buffer 0 and marks it valid, so
// E4 reports not-ready until the drain completes, then ready.
static volatile u32 sPendingFetchSector = 0xFFFFFFFF;
static volatile bool sPendingFetch;
#endif

static bool __time_critical_func(beginPhysicalSdRead)(u32 bufferIndex,
                                                      u32 sector)
{
    // The request and its ownership record form one publication. A cartridge
    // write IRQ must not cancel the SdCard request in the few instructions
    // before sReadBusy/LBA/buffer become visible.
    uint32_t irqState = save_and_disable_interrupts();
    bool accepted = gSdCard.TryBeginReadSectors(
        &sSdSectorBuf[bufferIndex * 512], sector, 1);
    if (!accepted)
    {
        restore_interrupts(irqState);
        return false;
    }

    sPhysicalReadSector = sector;
    sPhysicalReadBufferIndex = bufferIndex;
    sReadBusy = true;
    restore_interrupts(irqState);
    return true;
}

static void __time_critical_func(cancelPhysicalSdRead)(void)
{
    if (sReadBusy)
        gSdCard.Cancel();
    sReadBusy = false;
    sPhysicalReadSector = 0xFFFFFFFF;
    sPhysicalReadBufferIndex = 0;
}

// This translation unit used to live in SCRATCH_Y together with the core0
// stack. The cache/write state machines grew large enough to overlap that
// stack, corrupting the cartridge handlers before NDSL reached its first E3.
// Keep these routines deterministic and out of XIP, but place them in the
// larger striped SRAM region via __time_critical_func instead.
static void __time_critical_func(beginSdWriteTransaction)(void)
{
    // Stop publishing any read that was requested before this write. The
    // physical read buffer is separate from the write payload, but the SDIO
    // state machine still has to become idle before the write can start.
#ifdef ENABLE_PSRAM_CACHE
    sPendingFetch = false;
    sPendingFetchSector = 0xFFFFFFFF;
    sPendingStoreSector = 0xFFFFFFFF;
    romCacheSdWriteBegin();
#endif
    cancelPhysicalSdRead();
}

// Main-loop drain for an async SD cache hit. Fills buffer 0 from PSRAM and
// marks it valid so E4 reports ready. Runs on core0 and remains preemptible by
// PIO0_IRQ_0 with either backend. Must NOT be called from IRQ context.
extern "C" void ntrc_sdCacheFetchDrain(void)
{
#ifdef ENABLE_PSRAM_CACHE
    if (!sPendingFetch)
        return;

    // A cache request may have replaced a speculative physical read. Wait for
    // SdCard::Cancel() to finish before PSRAM writes the same double-buffer.
    if (!gSdCard.IsReady())
        return;

    u32 sector = sPendingFetchSector;
    // Fill buffer 0 from the PSRAM cache. A write may have
    // invalidated the line while it was in flight. In that case fall back to
    // the SD card; if a write currently owns the SD interface, retain the
    // pending request and retry from the main loop once it becomes idle.
    bool cacheRead = romCacheSdReadCached(sector, &sSdSectorBuf[0]);
    if (cacheRead)
    {
        // F6/R4 may have cancelled this fetch while psram_read was waiting on
        // core1. Publish only if the exact request is still current.
        uint32_t irqState = save_and_disable_interrupts();
        if (sPendingFetch && sPendingFetchSector == sector)
        {
            sSdSectorBuffersSectors[0] = sector;
            sPendingFetch = false;
            sPendingFetchSector = 0xFFFFFFFF;
        }
        restore_interrupts(irqState);
        return;
    }

    if (!sPendingFetch || sPendingFetchSector != sector)
        return;

    if (beginPhysicalSdRead(0, sector))
    {
        bool cancelled;
        uint32_t irqState = save_and_disable_interrupts();
        cancelled = !sPendingFetch || sPendingFetchSector != sector;
        if (!cancelled)
        {
            sPendingStoreSector = sector;
            sPendingFetch = false;
            sPendingFetchSector = 0xFFFFFFFF;
        }
        restore_interrupts(irqState);
        if (cancelled)
            gSdCard.Cancel();
    }
#endif
}

extern "C" void __time_critical_func(ntrc_gameReqSdReadCmd1)(ntr_rom_emu_t* romEmu, u32 word, pio_hw_t* pio)
{
    ntrc_noPayload(pio);
    sCurSdSector = 0xFFFFFFFF;
    sReadSector = word;
#ifdef ENABLE_PSRAM_CACHE
    // Cache hit: the sector is in PSRAM, but we must NOT read it here (a
    // psram_read in PIO0_IRQ_0 keeps the handler busy and breaks the loader).
    // tag-only check; on a hit record the sector for the main-loop drain
    // (ntrc_sdCacheFetchDrain) and leave both buffers invalid so E4 reports
    // not-ready until the drain fills buffer 0. No SD read is started.
    if (romCacheSdCheckHit(word))
    {
        // Normally pico-loader's final E4 has already completed the
        // speculative next-sector read. Still cancel it explicitly if another
        // caller starts a new E3 early; otherwise its DMA can overwrite the
        // PSRAM hit buffer after the cache checksum has passed.
        cancelPhysicalSdRead();
        sSdSectorBuffersSectors[0] = 0xFFFFFFFF;
        sSdSectorBuffersSectors[1] = 0xFFFFFFFF;
        sBufferIndex = 0;
        sPendingStoreSector = 0xFFFFFFFF; // hit: already cached, no backfill
        sPendingFetchSector = word;
        sPendingFetch = true;
        ntrc_finishGameNoScrambleCmd1(romEmu);
        return;
    }
    sPendingFetch = false;
    sPendingStoreSector = word;
#endif
    if (sSdSectorBuffersSectors[sBufferIndex] != word)
    {
        sSdSectorBuffersSectors[0] = 0xFFFFFFFF;
        sSdSectorBuffersSectors[1] = 0xFFFFFFFF;
        sBufferIndex = 0;
        if (!beginPhysicalSdRead(0, sReadSector))
        {
            __breakpoint();
        }
    }
    ntrc_finishGameNoScrambleCmd1(romEmu);
}

extern "C" void __time_critical_func(ntrc_gameGetSdStatCmd0)(ntr_rom_emu_t* romEmu, u32 word, pio_hw_t* pio)
{
    gCartSdE4Polls++;
    ntrc_beginWrite(pio, 4);

    bool sdReady;
    bool startCurrentWrite = false;
    bool startNextWrite = false;
    bool endWriteTransaction = false;
    if (sWriteBusy)
    {
        if (sWritePendingStart)
        {
            // Do not expose ready until the physical write has completed. The
            // first poll that sees both SDIO and PSRAM idle starts it after the
            // E4 response, then a later poll reports completion.
            sdReady = false;
#ifdef ENABLE_PSRAM_CACHE
            startCurrentWrite = gSdCard.IsReady() &&
                                romCacheSdWriteBarrierReady();
#else
            startCurrentWrite = gSdCard.IsReady();
#endif
        }
        else
        {
            sdReady = gSdCard.IsReady();
            if (sdReady)
            {
                if (sNextWriteBlockQueued)
                    startNextWrite = true;
                else
                {
                    sWriteBusy = false;
                    endWriteTransaction = true;
                }
            }
        }
    }
    else
    {
        // Retire the exact physical transaction that completed. Never infer
        // its LBA or destination from the protocol cursor: both can be changed
        // by E3/E5 IRQs independently of the SDIO DMA.
        if (sReadBusy && gSdCard.IsReady())
        {
            sSdSectorBuffersSectors[sPhysicalReadBufferIndex] =
                sPhysicalReadSector;
            sReadBusy = false;
            sPhysicalReadSector = 0xFFFFFFFF;
            sPhysicalReadBufferIndex = 0;
        }
        sdReady = sSdSectorBuffersSectors[sBufferIndex] == sReadSector;
    }

    ntrc_writeWord(pio, sdReady ? 1 : 0);
    if (sdReady)
        gCartSdE4Ready++;
    ntrc_finishGameNoScrambleCmd0(romEmu);

    if (startCurrentWrite)
    {
        if (!gSdCard.TryBeginWriteSectors(
                &sSdWriteSectorBuf[sCurrentWriteBufferIndex * 512],
                sCurrentWriteSector, 1, !sCurrentWriteIsLast))
        {
            __breakpoint();
        }
        sWritePendingStart = false;
    }
    else if (startNextWrite)
    {
        if (!gSdCard.TryBeginWriteSectors(
                &sSdWriteSectorBuf[sNextWriteBufferIndex * 512],
                sNextWriteSector, 1, !sNextWriteIsLast))
        {
            __breakpoint();
        }

        sNextWriteBlockQueued = false;
        sNextWriteIsLast = false;
        sNextWriteSector = 0xFFFFFFFF;
    }
#ifdef ENABLE_PSRAM_CACHE
    else if (endWriteTransaction)
    {
        romCacheSdWriteEnd();
    }
#endif
}

extern "C" void __time_critical_func(ntrc_gameGetSdDataCmd0)(ntr_rom_emu_t* romEmu, u32 word, pio_hw_t* pio)
{
    gCartSdE5Reads++;
    ntrc_beginWrite(pio, 512);

    // without scrambling to save time
    ntrc_dmaToBus(&sSdSectorBuf[sBufferIndex * 512], 512);

#ifdef ENABLE_PSRAM_CACHE
    // Backfill the sector we just served into the PSRAM cache (if this was a
    // cache miss served from the SD). Capture the sector index BEFORE the
    // sReadSector++/buffer flip below. romCacheSdStore() is ASYNC: it only
    // memcpy's the 512 B into an internal buffer and sets a pending flag (~1
    // us, no PSRAM access), so it does not stall PIO0_IRQ_0. The actual
    // psram_write runs later in romCacheSdStoreDrain() on the main loop.
    // (A synchronous psram_write here blocks the IRQ: SM0 stages the next
    // command word but the handler has not returned -> "failed to mount SD".)
    u32 servedSector = sSdSectorBuffersSectors[sBufferIndex];
    if (servedSector != 0xFFFFFFFF && servedSector == sPendingStoreSector)
    {
        romCacheSdStore(servedSector, &sSdSectorBuf[sBufferIndex * 512]);
        sPendingStoreSector = 0xFFFFFFFF;
    }
#endif

    sReadSector++;

    sBufferIndex = 1 - sBufferIndex;

    if (!sReadBusy)
    {
        if (!beginPhysicalSdRead(sBufferIndex, sReadSector))
        {
            __breakpoint();
        }
    }

    ntrc_finishGameNoScrambleCmd0(romEmu);
}

extern "C" void __time_critical_func(ntrc_gameGetSdDataCmd1)(ntr_rom_emu_t* romEmu, u32 word, pio_hw_t* pio)
{
    // we still need to advance the state
    ntrc_finishGameNoScrambleCmd1(romEmu);
}

extern "C" void __time_critical_func(ntrc_gameWriteSdDataCmd0)(ntr_rom_emu_t* romEmu, u32 word, pio_hw_t* pio)
{
    if ((word & ~WRITE_SD_DATA_FLAGS_MASK) != NTR_CMD_ID_GAME_WRITE_SD_DATA)
    {
        ntrc_gameCmd0Dummy(romEmu, word, pio);
        return;
    }

    ntrc_beginRead(pio, 512);
    sCurSdSector = 0xFFFFFFFF;
    sSdSectorBuffersSectors[0] = 0xFFFFFFFF;
    sSdSectorBuffersSectors[1] = 0xFFFFFFFF;
    if ((word & WRITE_SD_DATA_IS_FIRST_FLAG) != 0)
        beginSdWriteTransaction();

    ntrc_finishGameNoScrambleCmd0(romEmu);
}

static void __time_critical_func(sdWritePayloadComplete)(ntr_rom_emu_t* romEmu)
{
    bool isFirst = (romEmu->cmd0 & WRITE_SD_DATA_IS_FIRST_FLAG) != 0;
    bool isLast = (romEmu->cmd0 & WRITE_SD_DATA_IS_LAST_FLAG) != 0;
    // The SD is written directly (write-around), so invalidate any cached copy
    // of the sector BEFORE the write lands - otherwise a later E3 read of this
    // sector could hit the stale pre-write data (lost save). Single SRAM tag
    // store, IRQ-safe. Done for every written sector (isFirst and chained).
#ifdef ENABLE_PSRAM_CACHE
    romCacheSdInvalidateSector(romEmu->cmd1);
#endif
    if (isFirst)
    {
        sWriteBusy = true;
        sWritePendingStart = true;
        sCurrentWriteIsLast = isLast;
        sCurrentWriteSector = romEmu->cmd1;
        sCurrentWriteBufferIndex = sWriteBufferIndex;
    }
    else
    {
        sNextWriteBlockQueued = true;
        sNextWriteIsLast = isLast;
        sNextWriteSector = romEmu->cmd1;
        sNextWriteBufferIndex = sWriteBufferIndex;
    }
}

extern "C" void __time_critical_func(ntrc_gameWriteSdDataCmd1)(ntr_rom_emu_t* romEmu, u32 word, pio_hw_t* pio)
{
    if ((romEmu->cmd0 & ~WRITE_SD_DATA_FLAGS_MASK) != NTR_CMD_ID_GAME_WRITE_SD_DATA)
    {
        ntrc_gameCmd1Unknown(romEmu, word, pio);
        return;
    }

    bool isFirst = (romEmu->cmd0 & WRITE_SD_DATA_IS_FIRST_FLAG) != 0;
    if (isFirst)
    {
        sNextWriteBlockQueued = false;
        sNextWriteIsLast = false;
        sNextWriteSector = 0xFFFFFFFF;
        sWritePendingStart = false;
        sCurrentWriteIsLast = false;
        sCurrentWriteSector = 0xFFFFFFFF;
        sWriteBufferIndex = 0;
    }
    else
    {
        sWriteBufferIndex = 1 - sWriteBufferIndex;
    }

    ntrc_finishGameNoScrambleCmd1WithReadPayload(romEmu,
        (u32*)&sSdWriteSectorBuf[sWriteBufferIndex * 512], 512,
        sdWritePayloadComplete);
}

#ifdef ENABLE_R4_MODE

static bool sR4WritePendingStart;
static bool sR4WriteInProgress;
static u32 sR4WriteSector = 0xFFFFFFFF;

extern "C" void __time_critical_func(ntrc_gameR4StartSdReadCmd0)(ntr_rom_emu_t* romEmu, u32 word, pio_hw_t* pio)
{
    ntrc_beginWrite(pio, 4);
    u32 sector = (romEmu->cmd0 << 8) >> 9;
    u32 result = 0x1F4;
    if (gSdCard.IsReady())
    {
        if (sector == sCurSdSector)
        {
            result = 0;
        }
        else
        {
            if (!gSdCard.TryBeginReadSectors(sSdSectorBuf, sector, 1))
            {
                __breakpoint();
            }
            sCurSdSector = sector;
            sSdSectorBuffersSectors[0] = 0xFFFFFFFF;
            sSdSectorBuffersSectors[1] = 0xFFFFFFFF;
        }
    }
    ntrc_writeWord(pio, result);
    ntrc_finishGameNoScrambleCmd0(romEmu);
}

extern "C" void __time_critical_func(ntrc_gameR4GetSdDataCmd1)(ntr_rom_emu_t* romEmu, u32 word, pio_hw_t* pio)
{
    ntrc_beginWrite(pio, 512);
    ntrc_dmaToBus(sSdSectorBuf, 512);
    romEmu->wordIdx = 0;
}

extern "C" void __time_critical_func(ntrc_gameR4StartSdWriteCmd0)(ntr_rom_emu_t* romEmu, u32 word, pio_hw_t* pio)
{
    ntrc_beginRead(pio, 512);
    sCurSdSector = 0xFFFFFFFF;
    sSdSectorBuffersSectors[0] = 0xFFFFFFFF;
    sSdSectorBuffersSectors[1] = 0xFFFFFFFF;
    sNextWriteBlockQueued = false;
    sNextWriteIsLast = false;
    sNextWriteSector = 0xFFFFFFFF;
    sR4WritePendingStart = false;
    sR4WriteInProgress = false;
    sR4WriteSector = 0xFFFFFFFF;
    beginSdWriteTransaction();
    ntrc_finishGameNoScrambleCmd0(romEmu);
}

static void __time_critical_func(r4SdWritePayloadComplete)(ntr_rom_emu_t* romEmu)
{
    u32 sector = (romEmu->cmd0 << 8) >> 9;
#ifdef ENABLE_PSRAM_CACHE
    // R4 and pico-loader share the SD card. Invalidate the E3/E5 cache just
    // like the F6 write path, including any async backfill already in flight.
    romCacheSdInvalidateSector(sector);
#endif
    sR4WriteSector = sector;
    sR4WritePendingStart = true;
}

extern "C" void __time_critical_func(ntrc_gameR4StartSdWriteCmd1)(ntr_rom_emu_t* romEmu, u32 word, pio_hw_t* pio)
{
    ntrc_finishGameNoScrambleCmd1WithReadPayload(
        romEmu, (u32*)sSdWriteSectorBuf, 512, r4SdWritePayloadComplete);
}

extern "C" void __time_critical_func(ntrc_gameR4GetSdWriteStatCmd0)(ntr_rom_emu_t* romEmu, u32 word, pio_hw_t* pio)
{
    ntrc_beginWrite(pio, 4);
    u32 sdStat = 1;
    bool startWrite = false;
    bool endWriteTransaction = false;
    if (sR4WritePendingStart)
    {
#ifdef ENABLE_PSRAM_CACHE
        startWrite = gSdCard.IsReady() && romCacheSdWriteBarrierReady();
#else
        startWrite = gSdCard.IsReady();
#endif
    }
    else if (sR4WriteInProgress && gSdCard.IsReady())
    {
        sR4WriteInProgress = false;
        sdStat = 0;
        endWriteTransaction = true;
    }
    else if (!sR4WriteInProgress && gSdCard.IsReady())
    {
        sdStat = 0;
    }
    ntrc_writeWord(pio, sdStat);
    ntrc_finishGameNoScrambleCmd0(romEmu);

    if (startWrite)
    {
        if (__builtin_expect(!gSdCard.TryBeginWriteSectors(
                sSdWriteSectorBuf, sR4WriteSector, 1, false), false))
        {
            __breakpoint();
        }
        sR4WritePendingStart = false;
        sR4WriteInProgress = true;
    }
#ifdef ENABLE_PSRAM_CACHE
    else if (endWriteTransaction)
    {
        romCacheSdWriteEnd();
    }
#endif
}

#endif
