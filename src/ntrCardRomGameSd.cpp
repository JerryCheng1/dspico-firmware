#include "common.h"
#include <stdio.h>
#include "r4.h"
#include "ntrCardRom.h"
#include "ntrCardRomGameNoScramble.h"
#if CACHE_STAGE >= 3
#include "cacheSd.h"
#endif
#ifdef CACHE_PSRAM_SECTOR0_DIAG
#include "psram.h"
#include <string.h>
#endif
#ifdef ENABLE_UART_LOG
#include "uartLog.h"
#endif

#ifdef CACHE_PSRAM_SECTOR0_DIAG
static void psramBootReadStartProbe(u32 chip, u32 addr, u8 expected)
{
    u8 sio = 0, pio = 0;
    bool sioRead = psramBitBangRead(chip, addr, &sio, 1);
    bool pioRead = psramRead(chip, addr, &pio, 1);
    uartLogPrintfBlocking("[psram-start] chip=%lu addr=%lu exp=%02X status=%u/%u sio=%02X pio=%02X\n",
        (unsigned long)chip, (unsigned long)addr, (unsigned)expected,
        (unsigned)sioRead, (unsigned)pioRead,
        (unsigned)sio, (unsigned)pio);
}

static void psramBootAddressProbe(u32 chip)
{
    static u8 source[32] __attribute__((aligned(4)));
    static u8 sioBlock[32] __attribute__((aligned(4)));
    static u8 pioBlock[32] __attribute__((aligned(4)));
    const u32 base = 416;
    for (u32 i = 0; i < sizeof(source); i++)
        source[i] = (u8)(0xA5u ^ (u8)(i * 37u));
    source[440 - base] = 0x52;

    bool wrote = psramBitBangWrite(chip, base, source, sizeof(source));
    bool sioRead = wrote && psramBitBangRead(chip, base, sioBlock, sizeof(sioBlock));
    bool pioRead = wrote && psramRead(chip, base, pioBlock, sizeof(pioBlock));
    uartLogPrintfBlocking("[psram-block] chip=%lu base=%lu status=%u/%u/%u exp440=%02X sio440=%02X pio440=%02X equal=%u/%u\n",
        (unsigned long)chip, (unsigned long)base,
        (unsigned)wrote, (unsigned)sioRead, (unsigned)pioRead,
        (unsigned)source[440 - base], (unsigned)sioBlock[440 - base],
        (unsigned)pioBlock[440 - base],
        (unsigned)(sioRead && memcmp(source, sioBlock, sizeof(source)) == 0),
        (unsigned)(pioRead && memcmp(source, pioBlock, sizeof(source)) == 0));

    if (!wrote || !sioRead || !pioRead)
        return;
    psramBootReadStartProbe(chip, 416, source[0]);
    psramBootReadStartProbe(chip, 417, source[1]);
    psramBootReadStartProbe(chip, 432, source[16]);
    psramBootReadStartProbe(chip, 440, source[24]);
    psramBootReadStartProbe(chip, 441, source[25]);

    u8 replacement = 0x0A;
    bool writeOne = psramBitBangWrite(chip, 440, &replacement, 1);
    sioRead = writeOne && psramBitBangRead(chip, base, sioBlock, sizeof(sioBlock));
    pioRead = writeOne && psramRead(chip, base, pioBlock, sizeof(pioBlock));
    uartLogPrintfBlocking("[psram-write1] chip=%lu addr=440 wr=0A status=%u/%u/%u sio440=%02X pio440=%02X sio441=%02X pio441=%02X\n",
        (unsigned long)chip,
        (unsigned)writeOne, (unsigned)sioRead, (unsigned)pioRead,
        (unsigned)sioBlock[440 - base], (unsigned)pioBlock[440 - base],
        (unsigned)sioBlock[441 - base], (unsigned)pioBlock[441 - base]);
}
#endif

// E4 poll pre-arm: while a status poll storm runs, the length word for the
// NEXT poll is pushed the moment the current answer is queued, so the SM
// never stalls on the length autopull waiting for the CPU. The hard response
// deadline (length word before the console's first status strobe, ~600-800 ns
// after the last command byte) then does not depend on the dispatch IRQ at
// all; only the data word does, and that has ~3x the margin. Any other
// command arriving with a pre-armed word pending must drain it first (see
// E3/E5/F6 + the CEB-rise recovery in main.cpp).
volatile u32 gCartSdE4LenArmed;
// Transaction generation (design section 6.3). An armed length word is only
// valid for the SAME cartridge transaction (the E3 -> E4 storm); a new E3/E5/F6
// command, a conducted reset or the CEB-rise recovery opens a new generation
// and invalidates it. This makes "the next E4 length of a transaction" explicit
// instead of relying on the bare flag.
static u32 sCartTxGen = 1;
static u32 sCartSdE4LenGen;
#if CACHE_STAGE >= 3
// The next E4 response is queued while the preceding command is still active.
static u32 sCartSdE4StatusPipelined;
static u32 sCartSdE4PipelinedReady;
#endif
// Diagnostics: times a pre-armed length had to be dropped while the RX FIFO
// was not empty (design section 6.3 / L06).
volatile u32 gCartSdFifoRecovery;
volatile u32 gCartSdRecoveryPending;
#ifdef CART_RECOVERY_DIAG
extern "C" {
volatile u32 gCartFirstPrearmCmd;
volatile u32 gCartFirstPrearmWord;
volatile u32 gCartFirstPrearmPc;
volatile u32 gCartFirstPrearmTx;
}
// This runs only after a response-queue conflict. Keep the snapshot outside
// the small scratch-Y IRQ code section used by the normal command handlers.
static __attribute__((noinline)) void cartLogFirstPrearm(void)
{
    if (gCartSdFifoRecovery)
        return;
    gCartFirstPrearmCmd = gNtrRomEmu.cmd0;
    gCartFirstPrearmWord = gNtrRomEmu.wordIdx;
    gCartFirstPrearmPc = pio_sm_get_pc(pio0, 0);
    gCartFirstPrearmTx = pio_sm_get_tx_fifo_level(pio0, 0);
}
#endif

static inline void cartTxBegin(void)
{
    sCartTxGen++;
    if (sCartTxGen == 0)
        sCartTxGen = 1;
}

// Called by the reset path and the CEB-rise recovery (design 6.3).
extern "C" void ntrc_cartBumpTxGen(void)
{
    cartTxBegin();
    gCartSdE4LenArmed = 0;
    gCartSdRecoveryPending = 0;
#if CACHE_STAGE >= 3
    sCartSdE4StatusPipelined = 0;
#endif
}

extern "C" u32 ntrc_cartTxGen(void) { return sCartTxGen; }

// A non-E4 command cannot safely reuse a pre-armed E4 length. Do not
// clear a running SM's FIFOs: RX can change after an empty check, and the
// old length may already be in OSR. Abandon this entire transaction; only
// the CEB-high recovery may clear/restart the SM and parser together.
static inline bool cartRejectPrearm(void)
{
    if (gCartSdRecoveryPending)
        return true;
    if (!gCartSdE4LenArmed)
        return false;
#ifdef CART_RECOVERY_DIAG
    cartLogFirstPrearm();
#endif
    gCartSdRecoveryPending = 1;
    gCartSdFifoRecovery++;
    return true;
}

static u8 sSdSectorBuf[1024];
static u32 sCurSdSector = 0xFFFFFFFF;
// Cache-slot state. Read cross-TU by the non-intrusive trace snapshot only
// (core0 main loop, never an IRQ). At CACHE_STAGE >= 3 the cache service in
// the main loop and the E3/E4/E5 handlers share it across the IRQ boundary, so
// it is volatile there; stages 0..2 keep the original non-volatile layout so
// the baseline image stays byte-identical.
#if CACHE_STAGE >= 3
volatile u32 sSdSectorBuffersSectors[2] = { 0xFFFFFFFF, 0xFFFFFFFF };
volatile u32 sBufferIndex = 0;
static u32 sReadSector;
volatile bool sReadBusy = false;
#else
u32 sSdSectorBuffersSectors[2] = { 0xFFFFFFFF, 0xFFFFFFFF };
u32 sBufferIndex = 0;
static u32 sReadSector;
bool sReadBusy = false;
#endif
bool sWriteBusy = false;
static bool sNextWriteBlockQueued = false;
static bool sNextWriteIsLast = false;
static u32 sNextWriteSector = 0xFFFFFFFF;
// A received F6 first block waiting for the main loop to start the SD write.
// The payload callback runs in IRQ context and must not start storage I/O (the
// cache backend may still own the SD engine).
static volatile bool sPendingWrite = false;
static volatile u32 sPendingWriteSector = 0xFFFFFFFF;
static volatile bool sPendingWriteKeepOpen = false;

// Write lifecycle token (design section 8.1). The barrier is accepted when the
// first F6 block arrives (before its payload), reused for every continuation
// block, and released only once the whole sequence is durable.
#if CACHE_STAGE >= 3
static u32 sWriteToken = 0;
static bool sWriteSequenceLast = false;
#ifdef ENABLE_R4_MODE
static u32 sR4WriteToken = 0;
static bool sR4WriteStarted = false;
#endif
#endif

extern "C" void __scratch_y("cpu0")(ntrc_gameReqSdReadCmd0)(ntr_rom_emu_t* romEmu, u32 word, pio_hw_t* pio)
{
    // E3 opens a new transaction: any pre-armed E4 length from a previous
    // transaction is stale by generation (design section 6.3). Writing the length at the cmd0 half (byte 4)
    // instead of cmd1 (byte 8) stops the SM stalling on the length autopull
    // for the IRQ latency - during which the first E4 poll's command bytes
    // were clocked past the deaf SM and lost (E4=0 after E3 on the clean
    // upstream port).
    if (cartRejectPrearm())
    {
        ntrc_finishGameNoScrambleCmd0(romEmu);
        return;
    }
    cartTxBegin();
    ntrc_noPayload(pio);
    ntrc_finishGameNoScrambleCmd0(romEmu);
#ifdef CACHE_FILL_QUIET_EXPERIMENT
    cacheSdNoteHostCommandFromIrq(CACHE_HOST_CMD_E3);
#endif
}

extern "C" void __scratch_y("cpu0")(ntrc_gameReqSdReadCmd1)(ntr_rom_emu_t* romEmu, u32 word, pio_hw_t* pio)
{
    if (gCartSdRecoveryPending)
    {
        ntrc_finishGameNoScrambleCmd1(romEmu);
        return;
    }

#if CACHE_STAGE >= 3
    // The first E4 may follow this final E3 command word before a 150 MHz
    // dispatch can supply its status byte. A conservative busy response is
    // already valid; the E4 handler will pipeline the next sampled status.
    if (!sWriteBusy)
    {
        ntrc_beginWrite(pio, 4);
        ntrc_writeWord(pio, 0);
        sCartSdE4LenGen = sCartTxGen;
        sCartSdE4PipelinedReady = 0;
        sCartSdE4StatusPipelined = 1;
        gCartSdE4LenArmed = 1;
    }
#endif

    sCurSdSector = 0xFFFFFFFF;
    sReadSector = word;
#if CACHE_STAGE >= 3
    // Publish the latest read intent. The state machine merges a same-identity
    // re-issue and replaces a different one without touching the in-flight job
    // or any slot it owns.
    cacheSdRequest(word);
#else
    if (sSdSectorBuffersSectors[sBufferIndex] != word)
    {
        sSdSectorBuffersSectors[0] = 0xFFFFFFFF;
        sSdSectorBuffersSectors[1] = 0xFFFFFFFF;
        sBufferIndex = 0;
        if (!gSdCard.TryBeginReadSectors(&sSdSectorBuf[0], sReadSector, 1))
        {
            __breakpoint();
        }
        sReadBusy = true;
    }
#endif
    ntrc_finishGameNoScrambleCmd1(romEmu);
}

extern "C" void __scratch_y("cpu0")(ntrc_gameGetSdStatCmd0)(ntr_rom_emu_t* romEmu, u32 word, pio_hw_t* pio)
{
    // Mid-storm the length word is already queued (pre-armed by the previous
    // poll, see gCartSdE4LenArmed) - the SM's length autopull never waits on
    // this IRQ. Only the first poll of a storm pushes its own length. A word
    // armed in a different transaction is stale and must not be reused.
    if (gCartSdRecoveryPending ||
        (gCartSdE4LenArmed && sCartSdE4LenGen != sCartTxGen && cartRejectPrearm()))
    {
        ntrc_finishGameNoScrambleCmd0(romEmu);
        return;
    }
#if CACHE_STAGE >= 3
    if (sCartSdE4StatusPipelined && !sWriteBusy)
    {
        // This poll's byte was queued by E3 or the preceding busy poll.
        // Bind only the ready value actually offered to the host, while the
        // next poll's length and status are placed in FIFO before bookkeeping.
        const bool queuedReady = sCartSdE4PipelinedReady != 0;
        if (queuedReady)
        {
            gCartSdE4LenArmed = 0;
            sCartSdE4StatusPipelined = 0;
        }
        else
        {
            const bool nextReady = gCacheSdReadReady != 0;
            ntrc_beginWrite(pio, 4);
            ntrc_writeWord(pio, nextReady ? 1 : 0);
            sCartSdE4PipelinedReady = nextReady ? 1u : 0u;
            sCartSdE4LenGen = sCartTxGen;
            gCartSdE4LenArmed = 1;
        }
        (void)cacheSdPollReadySampled(queuedReady);
        ntrc_finishGameNoScrambleCmd0(romEmu);
#ifdef CACHE_FILL_QUIET_EXPERIMENT
        cacheSdNoteHostCommandFromIrq(CACHE_HOST_CMD_E4);
#endif
        return;
    }
#endif
    if (!gCartSdE4LenArmed)
        ntrc_beginWrite(pio, 4);

    bool sdReady;
#if CACHE_STAGE >= 3
    const bool readStatus = !sWriteBusy;
#endif
    if (sWriteBusy)
    {
        sdReady = gSdCard.IsReady() && !sPendingWrite;
        if (sdReady && !sNextWriteBlockQueued)
        {
            sWriteBusy = false;
#if CACHE_STAGE >= 3
            // Whole sequential write is durable: release the write barrier so
            // reads may be served (from the new medium generation) again.
            if (sWriteSequenceLast && sWriteToken)
            {
                cacheSdWriteEnd(sWriteToken);
                sWriteToken = 0;
                sWriteSequenceLast = false;
            }
#endif
        }
    }
    else
    {
#if CACHE_STAGE >= 3
        // Queue the time-critical status before identity checks/binding copies.
        // The backend publishes this word only for a verified completion, and
        // the ready value is DERIVED from the descriptor (design 6.2).
        sdReady = gCacheSdReadReady != 0;
#else
        if (sSdSectorBuffersSectors[sBufferIndex] == sReadSector)
        {
            sdReady = true;
        }
        else if (sReadBusy && gSdCard.IsReady())
        {
            sSdSectorBuffersSectors[sBufferIndex] = sReadSector;
            sdReady = true;
            sReadBusy = false;
        }
        else
        {
            sdReady = false;
        }
#endif
    }

    ntrc_writeWord(pio, sdReady ? 1 : 0);
    if (sdReady)
    {
        // The console now fetches the sector (E5), which pushes its own 512
        // byte length - do not leave a stale E4 length in front of it.
        gCartSdE4LenArmed = 0;
    }
    else
    {
        // Storm continues: arm the next poll's length word immediately. The
        // SM pulls it long after this push (next command, byte 8), so this is
        // never on a deadline. Record the transaction generation it belongs to.
        ntrc_beginWrite(pio, 4);
        sCartSdE4LenGen = sCartTxGen;
        gCartSdE4LenArmed = 1;
    }
#if CACHE_STAGE >= 3
    // Main-loop publishers cannot run here, and E3/E5/write/reset IRQs cannot
    // preempt this highest-priority cartridge handler. Record the value that
    // was ALREADY queued into the FIFO and let the state machine bind the same
    // offer; a queued ready that cannot bind freezes E4_ACK_MISMATCH (design
    // section 6.1). No sector data is copied in this handler.
    if (readStatus)
        (void)cacheSdPollReadySampled(sdReady != 0);
#endif
    ntrc_finishGameNoScrambleCmd0(romEmu);
#ifdef CACHE_FILL_QUIET_EXPERIMENT
    cacheSdNoteHostCommandFromIrq(CACHE_HOST_CMD_E4);
#endif

    if (sWriteBusy && sdReady && sNextWriteBlockQueued)
    {
        if (!gSdCard.TryBeginWriteSectors(&sSdSectorBuf[sBufferIndex * 512], sNextWriteSector, 1, !sNextWriteIsLast))
        {
            __breakpoint();
        }

        sNextWriteBlockQueued = false;
        sNextWriteIsLast = false;
        sNextWriteSector = 0xFFFFFFFF;
    }
}

extern "C" void __scratch_y("cpu0")(ntrc_gameGetSdDataCmd0)(ntr_rom_emu_t* romEmu, u32 word, pio_hw_t* pio)
{
    if (cartRejectPrearm())
    {
        ntrc_finishGameNoScrambleCmd0(romEmu);
        return;
    }

    // Queue length before ownership checks, as in the working r3 OFF path.
    // Length alone does not send data: only a valid consume starts DMA0.
    ntrc_beginWrite(pio, 512);

#if CACHE_STAGE >= 3
    {
        u32 sector;
        u8* data = cacheSdTakeAckedFromIrq(&sector);
        if (data)
        {
            // Seed the first word before DMA configuration. The aligned slot
            // stays pinned while DMA supplies exactly the remaining 508 B.
            ntrc_writeWord(pio, *(const u32*)data);
            ntrc_dmaToBus(data + 4, 508);
            cacheSdRecordSendFromIrq(sector, data);
            // Queue the next sector only after the binding is consumed.
            sReadSector = sector + 1;
            cacheSdRequest(sReadSector);
        }
        else
        {
            // No legal binding: no data is fed. The
            // precise reason is frozen by cs_consume_read(); mark the bounded
            // transaction recovery that the CEB rise will complete.
            cacheSdRejectSendFromIrq();
            cacheSdMarkE5Recovery();
        }
    }
#else
    ntrc_dmaToBus(&sSdSectorBuf[sBufferIndex * 512], 512);
    sReadSector++;
    sBufferIndex = 1 - sBufferIndex;
    if (!sReadBusy)
    {
        if (!gSdCard.TryBeginReadSectors(&sSdSectorBuf[sBufferIndex * 512], sReadSector, 1))
        {
            __breakpoint();
        }
        sReadBusy = true;
    }
#endif

    cartTxBegin();
    ntrc_finishGameNoScrambleCmd0(romEmu);
#ifdef CACHE_FILL_QUIET_EXPERIMENT
    cacheSdNoteHostCommandFromIrq(CACHE_HOST_CMD_E5);
#endif
}

extern "C" void __scratch_y("cpu0")(ntrc_gameGetSdDataCmd1)(ntr_rom_emu_t* romEmu, u32 word, pio_hw_t* pio)
{
    // we still need to advance the state
    ntrc_finishGameNoScrambleCmd1(romEmu);
}

extern "C" void __scratch_y("cpu0")(ntrc_gameWriteSdDataCmd0)(ntr_rom_emu_t* romEmu, u32 word, pio_hw_t* pio)
{
    if ((word & ~WRITE_SD_DATA_FLAGS_MASK) != NTR_CMD_ID_GAME_WRITE_SD_DATA)
    {
        ntrc_gameCmd0Dummy(romEmu, word, pio);
        return;
    }

    if (cartRejectPrearm())
    {
        ntrc_finishGameNoScrambleCmd0(romEmu);
        return;
    }
    cartTxBegin();
    ntrc_beginRead(pio, 512);
    sCurSdSector = 0xFFFFFFFF;
    sSdSectorBuffersSectors[0] = 0xFFFFFFFF;
    sSdSectorBuffersSectors[1] = 0xFFFFFFFF;
#if CACHE_STAGE >= 3
    // Register the write intent before its payload is received. Continuation
    // blocks of a sequential write reuse the same token/barrier; only a first
    // block opens a new barrier generation (design section 8.1).
    if ((word & WRITE_SD_DATA_IS_FIRST_FLAG) != 0)
    {
        sWriteToken = cacheSdWriteBegin();
        sWriteSequenceLast = false;
    }
    else if (sWriteToken == 0)
    {
        sWriteToken = cacheSdWriteBegin();
    }
#endif

    ntrc_finishGameNoScrambleCmd0(romEmu);
#ifdef CACHE_FILL_QUIET_EXPERIMENT
    cacheSdNoteHostCommandFromIrq(CACHE_HOST_CMD_F6);
#endif
}

static void __scratch_y("cpu0")(sdWritePayloadComplete)(ntr_rom_emu_t* romEmu)
{
    bool isFirst = (romEmu->cmd0 & WRITE_SD_DATA_IS_FIRST_FLAG) != 0;
    bool isLast = (romEmu->cmd0 & WRITE_SD_DATA_IS_LAST_FLAG) != 0;
    if (isFirst)
    {
#if CACHE_STAGE >= 3
        // Defer the actual SD write to the main loop (design section 8.1).
        sPendingWriteSector = romEmu->cmd1;
        sPendingWriteKeepOpen = !isLast;
        sPendingWrite = true;
        sWriteBusy = true;
        if (isLast)
            sWriteSequenceLast = true;
#else
        // Stages 0..2 keep the original immediate submit so the baseline write
        // pipeline is unchanged (fixes the R5 staging regression).
        if (!gSdCard.TryBeginWriteSectors(sSdSectorBuf, romEmu->cmd1, 1, !isLast))
        {
            __breakpoint();
        }
        sWriteBusy = true;
#endif
    }
    else
    {
        sNextWriteBlockQueued = true;
        sNextWriteIsLast = isLast;
        sNextWriteSector = romEmu->cmd1;
#if CACHE_STAGE >= 3
        if (isLast)
            sWriteSequenceLast = true;
#endif
    }
}

extern "C" void ntrc_gameSdWriteService(void)
{
#if CACHE_STAGE >= 3
    if (sPendingWrite && gSdCard.IsReady())
    {
        if (gSdCard.TryBeginWriteSectors(sSdSectorBuf, sPendingWriteSector, 1, sPendingWriteKeepOpen))
            sPendingWrite = false;
    }
#endif
}

extern "C" void __scratch_y("cpu0")(ntrc_gameWriteSdDataCmd1)(ntr_rom_emu_t* romEmu, u32 word, pio_hw_t* pio)
{
    if (gCartSdRecoveryPending)
    {
        ntrc_finishGameNoScrambleCmd1(romEmu);
        return;
    }

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
        sBufferIndex = 0;
    }
    else
    {
        sBufferIndex = 1 - sBufferIndex;
    }

    ntrc_finishGameNoScrambleCmd1WithReadPayload(romEmu,
        (u32*)&sSdSectorBuf[sBufferIndex * 512], 512, sdWritePayloadComplete);
}

// Cold-boot pre-warm. The loader's first read (sector 0, MBR) would
// otherwise race the SD card's power-on housekeeping (FTL init/GC), which
// stretches the E4 busy-poll storm from tens to thousands of polls. Every
// poll is a fresh chance for a wire-level desync, and the DS-side DLDI
// driver treats ANY nonzero status word as ready, so storm length translates
// directly into mount-failure probability. Reading sector 0 here - at boot,
// where the 1 s block timeout + retry path absorbs card housekeeping with no
// host poll deadline - forces the card through that phase before the loader
// starts, and publishing it in the buffer cache makes the first E3/E4 answer
// ready immediately.
extern "C" void ntrc_gameSdPrewarm(void)
{
    if (!gSdCard.TryReadSectorsSync(&sSdSectorBuf[0], 0, 1) ||
        gSdCard.GetSectorsCompleted() != 1)
    {
#ifdef ENABLE_UART_LOG
        uartLogPrintfBlocking("[sd] prewarm sec0 FAIL\n");
#endif
        return;
    }
    sSdSectorBuffersSectors[0] = 0;
    sBufferIndex = 0;
    sReadBusy = false;
#if CACHE_STAGE >= 3 && defined(CACHE_BOOT_SECTOR0_SEED)
    cacheSdSeedBootSector0(&sSdSectorBuf[0]);
#endif
#ifdef CACHE_PSRAM_SECTOR0_DIAG
    static u8 sec0Snapshot[512] __attribute__((aligned(4)));
    static u8 sparse52[512] __attribute__((aligned(4)));
    for (u32 i = 0; i < 512u; i++)
        sec0Snapshot[i] = sSdSectorBuf[i];
    sparse52[440] = 0x52;
    uartLogPrintfBlocking("[psram-src] snap440=%02X sd440=%02X\n",
        (unsigned)sec0Snapshot[440], (unsigned)sSdSectorBuf[440]);
    psramCacheWindowDiag lowDiag = psramCacheWindowDataTest(sec0Snapshot);
    uartLogPrintfBlocking("[psram-sec0-pio-wr] flags=%02lX pioOff=%lu pio=%02X/%02X sioOff=%lu sio=%02X/%02X\n",
        (unsigned long)lowDiag.flags, (unsigned long)lowDiag.pioOff,
        (unsigned)lowDiag.expectedPio, (unsigned)lowDiag.actualPio,
        (unsigned long)lowDiag.sioOff,
        (unsigned)lowDiag.expectedSio, (unsigned)lowDiag.actualSio);
    lowDiag = psramCacheWindowSioWriteTest(sec0Snapshot);
    uartLogPrintfBlocking("[psram-sec0-sio-wr] flags=%02lX pioOff=%lu pio=%02X/%02X sioOff=%lu sio=%02X/%02X\n",
        (unsigned long)lowDiag.flags, (unsigned long)lowDiag.pioOff,
        (unsigned)lowDiag.expectedPio, (unsigned)lowDiag.actualPio,
        (unsigned long)lowDiag.sioOff,
        (unsigned)lowDiag.expectedSio, (unsigned)lowDiag.actualSio);
    uartLogPrintfBlocking("[psram-src-after] snap440=%02X sd440=%02X\n",
        (unsigned)sec0Snapshot[440], (unsigned)sSdSectorBuf[440]);
    lowDiag = psramCacheWindowDataTest(sparse52);
    uartLogPrintfBlocking("[psram-zero52-pio-wr] flags=%02lX pioOff=%lu pio=%02X/%02X sioOff=%lu sio=%02X/%02X\n",
        (unsigned long)lowDiag.flags, (unsigned long)lowDiag.pioOff,
        (unsigned)lowDiag.expectedPio, (unsigned)lowDiag.actualPio,
        (unsigned long)lowDiag.sioOff,
        (unsigned)lowDiag.expectedSio, (unsigned)lowDiag.actualSio);
    lowDiag = psramCacheWindowSioWriteTest(sparse52);
    uartLogPrintfBlocking("[psram-zero52-sio-wr] flags=%02lX pioOff=%lu pio=%02X/%02X sioOff=%lu sio=%02X/%02X\n",
        (unsigned long)lowDiag.flags, (unsigned long)lowDiag.pioOff,
        (unsigned)lowDiag.expectedPio, (unsigned)lowDiag.actualPio,
        (unsigned long)lowDiag.sioOff,
        (unsigned)lowDiag.expectedSio, (unsigned)lowDiag.actualSio);
    for (u32 chip = 0; chip < PSRAM_CHIP_COUNT; chip++)
        psramBootAddressProbe(chip);
#endif
#if defined(ENABLE_UART_LOG) && !defined(CACHE_SUMMARY_LOG)
    const u32* p = (const u32*)&sSdSectorBuf[0];
    uartLogPrintfBlocking("[sd] prewarm sec0 OK head=%08lX %08lX sig=%04lX\n",
        (unsigned long)p[0], (unsigned long)p[1],
        (unsigned long)((p[127] >> 16) & 0xFFFFu));
#endif
}

#ifdef ENABLE_R4_MODE

extern "C" void __scratch_y("cpu0")(ntrc_gameR4StartSdReadCmd0)(ntr_rom_emu_t* romEmu, u32 word, pio_hw_t* pio)
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

extern "C" void __scratch_y("cpu0")(ntrc_gameR4GetSdDataCmd1)(ntr_rom_emu_t* romEmu, u32 word, pio_hw_t* pio)
{
    ntrc_beginWrite(pio, 512);
    ntrc_dmaToBus(sSdSectorBuf, 512);
    romEmu->wordIdx = 0;
}

extern "C" void __scratch_y("cpu0")(ntrc_gameR4StartSdWriteCmd0)(ntr_rom_emu_t* romEmu, u32 word, pio_hw_t* pio)
{
    ntrc_beginRead(pio, 512);
    sCurSdSector = 0xFFFFFFFF;
    sSdSectorBuffersSectors[0] = 0xFFFFFFFF;
    sSdSectorBuffersSectors[1] = 0xFFFFFFFF;
    sNextWriteBlockQueued = false;
    sNextWriteIsLast = false;
    sNextWriteSector = 0xFFFFFFFF;
#if CACHE_STAGE >= 3
    sR4WriteToken = cacheSdWriteBegin();
    sR4WriteStarted = false;
#endif
    ntrc_finishGameNoScrambleCmd0(romEmu);
}

static void __scratch_y("cpu0")(r4SdWritePayloadComplete)(ntr_rom_emu_t* romEmu)
{
    if (__builtin_expect(!gSdCard.TryBeginWriteSectors(sSdSectorBuf, (romEmu->cmd0 << 8) >> 9, 1, false), false))
    {
        __breakpoint();
    }
#if CACHE_STAGE >= 3
    sR4WriteStarted = true;
#endif
}

extern "C" void __scratch_y("cpu0")(ntrc_gameR4StartSdWriteCmd1)(ntr_rom_emu_t* romEmu, u32 word, pio_hw_t* pio)
{
    ntrc_finishGameNoScrambleCmd1WithReadPayload(romEmu, (u32*)sSdSectorBuf, 512, r4SdWritePayloadComplete);
}

extern "C" void __scratch_y("cpu0")(ntrc_gameR4GetSdWriteStatCmd0)(ntr_rom_emu_t* romEmu, u32 word, pio_hw_t* pio)
{
    ntrc_beginWrite(pio, 4);
    u32 sdStat = 1;
    if (gSdCard.IsReady())
        sdStat = 0;
#if CACHE_STAGE >= 3
    if (sdStat == 0 && sR4WriteStarted && sR4WriteToken)
    {
        cacheSdWriteEnd(sR4WriteToken);
        sR4WriteToken = 0;
        sR4WriteStarted = false;
    }
#endif
    ntrc_writeWord(pio, sdStat);
    ntrc_finishGameNoScrambleCmd0(romEmu);
}

#endif
#if CACHE_STAGE >= 3
// Hooks consumed by the PSRAM sector cache (src/cacheSd.c).
extern "C" bool ntrc_cacheSdBegin(u8* dst, u32 sector)
{
    return gSdCard.TryBeginReadSectors(dst, sector, 1);
}

extern "C" bool ntrc_cacheSdReady(void)
{
    return gSdCard.IsReady();
}

extern "C" u32 ntrc_cacheSdTransferId(void)
{
    return gSdCard.GetTransferId();
}
#endif

#if CACHE_STAGE >= 3
extern "C" bool ntrc_cacheSdError(void)
{
    // Idle also occurs on cancel/out-of-range. Only a complete one-sector
    // transfer is a successful cache demand. CRC retries remain internal.
    return gSdCard.GetSectorCount() != 1 || gSdCard.GetSectorsCompleted() != 1;
}
#endif
