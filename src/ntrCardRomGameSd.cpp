#include "common.h"
#include <stdio.h>
#include "r4.h"
#include "ntrCardRom.h"
#include "ntrCardRomGameNoScramble.h"
#ifdef ENABLE_UART_LOG
#include "uartLog.h"
extern "C" {
volatile u32 gCartSdE3Requests;
volatile u32 gCartSdE4Polls;
volatile u32 gCartSdE4Ready;
volatile u32 gCartSdE5Reads;
// E5 fetches whose buffer was never latched by an E4 ready answer: the
// console mistook a garbage status word for ready (the DS-side DLDI driver
// polls until ANY nonzero word) and is fetching an unfilled buffer.
volatile u32 gCartSdE5Unready;
volatile u32 gCartSdDummyCmd0;
volatile u32 gCartSdUnknownCmd1;
volatile u32 gCartSdUnknownWord;
volatile u32 gCartSdUnknownCmd0;
volatile u32 gCartSdLastE4Us;
// E5 payload evidence: raw buffer content as served to the console, captured
// at fetch time (IRQ - just a few word copies; the main loop prints it).
volatile u32 gE5TraceCount;
u32 gE5TraceSector[8];
u32 gE5TraceHead[8][2];
u32 gE5TraceTail[8][2];
}
#endif

// E4 poll pre-arm: while a status poll storm runs, the length word for the
// NEXT poll is pushed the moment the current answer is queued, so the SM
// never stalls on the length autopull waiting for the CPU. The hard response
// deadline (length word before the console's first status strobe, ~600-800 ns
// after the last command byte) then does not depend on the dispatch IRQ at
// all; only the data word does, and that has ~3x the margin. Measured on
// wire: mid-storm the TX FIFO is already empty at dispatch (e4fl ring) - the
// SM's end-of-response autopull has sucked the pre-armed word into the OSR
// one transaction early, which is exactly the decoupling wanted. Any other
// command arriving with a pre-armed word pending must drain it first (see
// E3/E5/F6 + the CEB-rise recovery in main.cpp).
volatile u32 gCartSdE4LenArmed;
#ifdef ENABLE_UART_LOG
// TX FIFO level + pre-arm flag sampled at E4 cmd0 dispatch. Healthy mid-storm
// reads "level 0, armed": the SM already holds the length in its OSR.
volatile u32 gE4FlIdx;
u8 gE4FlRing[16];
#endif

static u8 sSdSectorBuf[1024];
static u32 sCurSdSector = 0xFFFFFFFF;
static u32 sSdSectorBuffersSectors[2] = { 0xFFFFFFFF, 0xFFFFFFFF };
static u32 sBufferIndex = 0;
static u32 sReadSector;
static bool sReadBusy = false;
static bool sWriteBusy = false;
static bool sNextWriteBlockQueued = false;
static bool sNextWriteIsLast = false;
static u32 sNextWriteSector = 0xFFFFFFFF;

extern "C" void __scratch_y("cpu0")(ntrc_gameReqSdReadCmd0)(ntr_rom_emu_t* romEmu, u32 word, pio_hw_t* pio)
{
    // E3 has no response payload. Writing the length at the cmd0 half (byte 4)
    // instead of cmd1 (byte 8) stops the SM stalling on the length autopull
    // for the IRQ latency - during which the first E4 poll's command bytes
    // were clocked past the deaf SM and lost (E4=0 after E3 on the clean
    // upstream port).
    if (gCartSdE4LenArmed)
    {
        // An abandoned E4 storm left its pre-armed length in the TX FIFO; it
        // would be pulled as THIS command's length. The RX FIFO is empty here
        // (cmd0 word already dispatched, cmd1 bytes still on the wire), so the
        // clear only drops the stale TX word; the ISR is untouched.
        pio_sm_clear_fifos(pio, 0);
        gCartSdE4LenArmed = 0;
    }
    ntrc_noPayload(pio);
    ntrc_finishGameNoScrambleCmd0(romEmu);
}

extern "C" void __scratch_y("cpu0")(ntrc_gameReqSdReadCmd1)(ntr_rom_emu_t* romEmu, u32 word, pio_hw_t* pio)
{
#ifdef ENABLE_UART_LOG
    gCartSdE3Requests++;
#endif
    sCurSdSector = 0xFFFFFFFF;
    sReadSector = word;
    SD_EVT(9, word);
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
    ntrc_finishGameNoScrambleCmd1(romEmu);
}

extern "C" void __scratch_y("cpu0")(ntrc_gameGetSdStatCmd0)(ntr_rom_emu_t* romEmu, u32 word, pio_hw_t* pio)
{
#ifdef ENABLE_UART_LOG
    // Sampled at dispatch, committed after the pushes so the diag build keeps
    // the no-debug build's response timing.
    u8 flAtEntry = (u8)((pio->flevel & 0xFu) | (gCartSdE4LenArmed ? 0x10u : 0));
    u32 readyEvt = 0;
#endif
    // Mid-storm the length word is already queued (pre-armed by the previous
    // poll, see gCartSdE4LenArmed) - the SM's length autopull never waits on
    // this IRQ. Only the first poll of a storm pushes its own length.
    if (!gCartSdE4LenArmed)
        ntrc_beginWrite(pio, 4);

    bool sdReady;
    if (sWriteBusy)
    {
        sdReady = gSdCard.IsReady();
        if (sdReady && !sNextWriteBlockQueued)
        {
            sWriteBusy = false;
        }
    }
    else
    {
        if (sSdSectorBuffersSectors[sBufferIndex] == sReadSector)
        {
            sdReady = true;
#ifdef ENABLE_UART_LOG
            readyEvt = 11; // ready from already-buffered sector
#endif
        }
        else if (sReadBusy && gSdCard.IsReady())
        {
            sSdSectorBuffersSectors[sBufferIndex] = sReadSector;
            sdReady = true;
            sReadBusy = false;
#ifdef ENABLE_UART_LOG
            readyEvt = 10; // ready latched from a finished SD read
#endif
        }
        else
        {
            sdReady = false;
        }
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
        // never on a deadline.
        ntrc_beginWrite(pio, 4);
        gCartSdE4LenArmed = 1;
    }
#ifdef ENABLE_UART_LOG
    gCartSdE4Polls++;
    gCartSdLastE4Us = time_us_32();
    gE4FlRing[gE4FlIdx++ & 15] = flAtEntry;
    if (readyEvt)
        SD_EVT(readyEvt, time_us_32());
    if (sdReady && !gCartSdE4Ready)
    {
        // Logic-analyzer trigger: raised exactly when the first ready=1 answer
        // is queued. The E5 command (if the engine runs it) follows within
        // microseconds, so a capture triggered here covers the boundary.
        gpio_put_masked(1u << 0, 1u << 0);
    }
    if (sdReady) gCartSdE4Ready++;
#endif
    ntrc_finishGameNoScrambleCmd0(romEmu);

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
    if (gCartSdE4LenArmed)
    {
        // E5 straight after a busy poll (protocol violation, but stay
        // coherent): drop the pre-armed E4 length before pushing our own.
        pio_sm_clear_fifos(pio, 0);
        gCartSdE4LenArmed = 0;
    }
    ntrc_beginWrite(pio, 512);

#ifdef ENABLE_UART_LOG
    gpio_put_masked(1u << 0, 0); // boundary covered: drop the trigger line
#endif

    // without scrambling to save time
#ifdef ENABLE_UART_LOG
    gCartSdE5Reads++;
    // Phantom-ready evidence: this E5's buffer was never latched by an E4
    // ready answer, so the DMA below serves an unfilled buffer. Must be
    // checked before sReadSector/sBufferIndex advance.
    if (sSdSectorBuffersSectors[sBufferIndex] != sReadSector)
        gCartSdE5Unready++;
#endif
    ntrc_dmaToBus(&sSdSectorBuf[sBufferIndex * 512], 512);
    // Record the dispatched word itself: sdevt2 showed E5_FETCH with NO
    // matching E5 word in the rx ring - if that reproduces, this arg is the
    // smoking gun (not 0xE5000000 => wild dispatch path).
    SD_EVT(13, word);
#ifdef ENABLE_UART_LOG
    {
        u32 t = gE5TraceCount;
        if (t < 8)
        {
            const u32* p = (const u32*)&sSdSectorBuf[sBufferIndex * 512];
            gE5TraceSector[t] = sReadSector;
            gE5TraceHead[t][0] = p[0];
            gE5TraceHead[t][1] = p[1];
            gE5TraceTail[t][0] = p[126];
            gE5TraceTail[t][1] = p[127];
            gE5TraceCount = t + 1;
        }
    }
#endif
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

    ntrc_finishGameNoScrambleCmd0(romEmu);
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

    if (gCartSdE4LenArmed)
    {
        // See ntrc_gameGetSdDataCmd0: never let a stale E4 length lead.
        pio_sm_clear_fifos(pio, 0);
        gCartSdE4LenArmed = 0;
    }
    ntrc_beginRead(pio, 512);
    sCurSdSector = 0xFFFFFFFF;
    sSdSectorBuffersSectors[0] = 0xFFFFFFFF;
    sSdSectorBuffersSectors[1] = 0xFFFFFFFF;

    ntrc_finishGameNoScrambleCmd0(romEmu);
}

static void __scratch_y("cpu0")(sdWritePayloadComplete)(ntr_rom_emu_t* romEmu)
{
    bool isFirst = (romEmu->cmd0 & WRITE_SD_DATA_IS_FIRST_FLAG) != 0;
    bool isLast = (romEmu->cmd0 & WRITE_SD_DATA_IS_LAST_FLAG) != 0;
    if (isFirst)
    {
        if (!gSdCard.TryBeginWriteSectors(sSdSectorBuf, romEmu->cmd1, 1, !isLast))
        {
            __breakpoint();
        }
        sWriteBusy = true;
    }
    else
    {
        sNextWriteBlockQueued = true;
        sNextWriteIsLast = isLast;
        sNextWriteSector = romEmu->cmd1;
    }
}

extern "C" void __scratch_y("cpu0")(ntrc_gameWriteSdDataCmd1)(ntr_rom_emu_t* romEmu, u32 word, pio_hw_t* pio)
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
    if (!gSdCard.TryReadSectorsSync(&sSdSectorBuf[0], 0, 1))
    {
#ifdef ENABLE_UART_LOG
        uartLogPrintfBlocking("[sd] prewarm sec0 FAIL\n");
#endif
        return;
    }
    sSdSectorBuffersSectors[0] = 0;
    sBufferIndex = 0;
    sReadBusy = false;
#ifdef ENABLE_UART_LOG
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
    ntrc_finishGameNoScrambleCmd0(romEmu);
}

static void __scratch_y("cpu0")(r4SdWritePayloadComplete)(ntr_rom_emu_t* romEmu)
{
    if (__builtin_expect(!gSdCard.TryBeginWriteSectors(sSdSectorBuf, (romEmu->cmd0 << 8) >> 9, 1, false), false))
    {
        __breakpoint();
    }
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
    ntrc_writeWord(pio, sdStat);
    ntrc_finishGameNoScrambleCmd0(romEmu);
}

#endif