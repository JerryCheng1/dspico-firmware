// Non-intrusive cartridge trace implementation.
//
// Layout:  PIO0 SMx listener -> DMA (no IRQ) -> SRAM ring -> core1 decode ->
//          bounded non-blocking UART1 text.
//
// The only writers to the real-time cartridge path remain the service SM and
// its IRQ. This module never registers an interrupt handler, never enables a
// DMA completion IRQ, and never runs on PIO0_IRQ_0 / GPIO / DMA / timer IRQs.

#include "common.h"

#if NTRC_TRACE_ENABLED

#include "pioUtil.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "hardware/structs/dma.h"
#include "pico/time.h"
#include "ntrCardTrace.pio.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

// Dedicated PIO0 state machine. SM0 is the cartridge service, SM1 may be the
// WRF SPI-UART program, SM2/SM3 are free.
#define TRACE_SM            2u

// Raw ring: 2048 32-bit words = 1024 {cmd0,cmd1} records. Aligned to its size
// so the DMA write-address ring can wrap in hardware.
#define TRACE_RING_WORDS    2048u
#define TRACE_RING_MASK     (TRACE_RING_WORDS - 1u)
#define TRACE_RING_BYTES    (TRACE_RING_WORDS * 4u)
#define TRACE_RING_BITS     13u // log2(TRACE_RING_BYTES)

#define TRACE_DMA_COUNT     0x7FFFFFFFu

// Bounded text staging consumed only by core1.
#define TRACE_TX_SIZE       2048u
#define TRACE_TX_MASK       (TRACE_TX_SIZE - 1u)
#define TRACE_TX_BUDGET     32u

// Core1 must remain a bounded observer.  A full trace ring is diagnostic
// loss, never a reason to monopolise the shared SRAM/interconnect while the
// cartridge service core is handling a burst.
#define TRACE_CONSUME_BUDGET_RECORDS 8u

// A batch of diagnostics may only start when the cartridge bus has been idle
// for this long and CEB is high.
#define TRACE_QUIET_US      2000u

#define TRACE_HISTORY       64u
#define TRACE_HISTORY_MASK  (TRACE_HISTORY - 1u)

static uint sTraceSm = TRACE_SM;
static uint sTraceOffset;
static int  sTraceDma = -1;
static volatile bool sTraceReady;

static u32 __attribute__((aligned(TRACE_RING_BYTES))) sTraceRing[TRACE_RING_WORDS];

// ---- core0 -> core1 snapshot (seqlock) ------------------------------------
static volatile u32 sSnapSeq;
static volatile ntrCardTraceSnapshot sSnap;

void ntrCardTracePublishSnapshot(const ntrCardTraceSnapshot* snapshot)
{
    u32 seq = sSnapSeq;
    sSnapSeq = seq + 1u;
    __dmb();
    sSnap = *snapshot;
    __dmb();
    sSnapSeq = seq + 2u;
}

static bool traceReadSnapshot(ntrCardTraceSnapshot* out)
{
    u32 start = sSnapSeq;
    if (start & 1u)
        return false;
    __dmb();
    *out = sSnap;
    __dmb();
    return sSnapSeq == start;
}

// ---- core1 consumer state -------------------------------------------------
typedef struct
{
    u32 cmd0;
    u32 cmd1;
    u32 tUs;
} trace_record_t;

static u32 sConsumed;                 // command words already consumed
static u32 sLastProduced;             // cached DMA produced counter
static u32 sDroppedRecords;
static u32 sLastBusUs;

static trace_record_t sHistory[TRACE_HISTORY];
static u32 sHistoryCount;

static u32 sTotal;
static u32 sCountE3;
static u32 sCountE4;
static u32 sCountE5;
static u32 sCountB7;
static u32 sCountB8;
static u32 sCountFc;
static u32 sCountF6;
static u32 sCountUsb;
static u32 sCountOther;

static u32 sE4SinceE3;
static bool sSeenE3;
static bool sAnomalyActive;

// Frozen anomaly window, emitted one line at a time.
static trace_record_t sDumpRecs[TRACE_HISTORY];
static u32 sDumpCount;
static u32 sDumpPos;
static u32 sDumpAnomId;
static u32 sDumpAnomSeq;

// Last summary values, so a summary line is emitted only when something moved.
static u32 sLastSummaryTotal = 0xFFFFFFFFu;

// ---- bounded text staging -------------------------------------------------
static char sTx[TRACE_TX_SIZE];
static u32 sTxW;
static u32 sTxR;
static u32 sTxDropped;

static u32 traceTxFree(void)
{
    u32 used = sTxW - sTxR;
    if (used >= TRACE_TX_SIZE)
        return 0;
    // Keep one slot empty so the ring's full and empty states differ.
    return TRACE_TX_SIZE - used - 1u;
}

static void traceTxPush(const char* p, u32 n)
{
    // A diagnostic line is indivisible.  Publishing a prefix and dropping
    // its tail makes the UART stream misleading, while dropping the whole
    // line is harmless and explicitly counted.
    if (n > traceTxFree())
    {
        sTxDropped++;
        return;
    }
    for (u32 i = 0; i < n; i++)
    {
        sTx[sTxW & TRACE_TX_MASK] = p[i];
        sTxW++;
    }
}

static void traceTxPrintf(const char* fmt, ...)
{
    char buf[160];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n <= 0)
        return;
    if ((u32)n >= sizeof(buf))
        n = (int)sizeof(buf) - 1;
    traceTxPush(buf, (u32)n);
}

// Non-blocking drain: stops the moment the UART FIFO is full, writing at most
// TRACE_TX_BUDGET bytes per scheduling pass.
static void traceTxPump(void)
{
    u32 budget = TRACE_TX_BUDGET;
    while (sTxR != sTxW && budget-- != 0u)
    {
        if (!uart_is_writable(uart1))
            return;
        uart_putc_raw(uart1, sTx[sTxR & TRACE_TX_MASK]);
        sTxR++;
    }
}

static bool traceTxSpace(u32 needed)
{
    return needed <= traceTxFree();
}

// ---- DMA / ring -----------------------------------------------------------
static u32 traceProduced(void)
{
    // Two samples; the larger remaining count is the earlier (safe) sample, so
    // the last word returned was fully committed before we read it.
    u32 r1 = dma_channel_hw_addr((uint)sTraceDma)->al2_transfer_count;
    u32 r2 = dma_channel_hw_addr((uint)sTraceDma)->al2_transfer_count;
    u32 rem = r1 > r2 ? r1 : r2;
    if (rem > TRACE_DMA_COUNT)
        rem = TRACE_DMA_COUNT;
    sLastProduced = TRACE_DMA_COUNT - rem;
    return sLastProduced;
}

// ---- decode ---------------------------------------------------------------
static const char* traceCmdName(u32 id)
{
    switch (id)
    {
        case 0xB7: return "B7_READ_PAGE";
        case 0xB8: return "B8_READ_ID";
        case 0xE3: return "E3_SD_REQ";
        case 0xE4: return "E4_SD_STAT";
        case 0xE5: return "E5_SD_DATA";
        case 0xF6: return "F6_SD_WRITE";
        case 0xFC: return "FC_NOSCRAMBLE";
        default:   return "UNKNOWN";
    }
}

static bool traceIsKnownDldiCmd(u32 id)
{
    switch (id)
    {
        case 0xB7:
        case 0xB8:
        case 0xE3:
        case 0xE4:
        case 0xE5:
        case 0xE8: // USB command / write / read / get event
        case 0xE9:
        case 0xEA:
        case 0xEB:
        case 0xF6:
        case 0xFC:
            return true;
        default:
            return false;
    }
}

static void traceTriggerAnomaly(u32 id, u32 seq)
{
    if (sAnomalyActive)
        return;
    sAnomalyActive = true;
    sDumpAnomId = id;
    sDumpAnomSeq = seq;
    sDumpPos = 0;

    u32 n = sHistoryCount < TRACE_HISTORY ? sHistoryCount : TRACE_HISTORY;
    u32 start = sHistoryCount - n;
    for (u32 i = 0; i < n; i++)
        sDumpRecs[i] = sHistory[(start + i) & TRACE_HISTORY_MASK];
    sDumpCount = n;
}

static void traceDecode(u32 cmd0, u32 cmd1, u32 tUs)
{
    u32 id = cmd0 >> 24;
    u32 seq = sTotal;

    sTotal++;
    switch (id)
    {
        case 0xE3: sCountE3++; sSeenE3 = true; sE4SinceE3 = 0; break;
        case 0xE4: sCountE4++; sE4SinceE3++; break;
        case 0xE5: sCountE5++; break;
        case 0xB7: sCountB7++; break;
        case 0xB8: sCountB8++; break;
        case 0xFC: sCountFc++; break;
        case 0xF6: sCountF6++; break;
        case 0xE8:
        case 0xE9:
        case 0xEA:
        case 0xEB: sCountUsb++; break;
        default:   sCountOther++; break;
    }

    trace_record_t* r = &sHistory[sHistoryCount & TRACE_HISTORY_MASK];
    r->cmd0 = cmd0;
    r->cmd1 = cmd1;
    r->tUs = tUs;
    sHistoryCount++;

    if (sSeenE3)
    {
        // Once DLDI traffic has started, an unknown command byte means the
        // service decoder and the wire have diverged.
        if (!traceIsKnownDldiCmd(id))
            traceTriggerAnomaly(id, seq);
        else if (id == 0xE5 && sE4SinceE3 == 0)
            traceTriggerAnomaly(id, seq); // E5 without any E4 poll since the last one
    }
}

static void traceConsume(void)
{
    u32 produced = traceProduced();
    if (produced == sConsumed)
        return;

    u32 available = produced - sConsumed;
    if (available > TRACE_RING_WORDS)
    {
        u32 skip = available - TRACE_RING_WORDS;
        skip = (skip + 1u) & ~1u; // keep record alignment
        sConsumed += skip;
        sDroppedRecords += skip / 2u;
        available = produced - sConsumed;
    }

    // One timer read timestamps the bounded batch.  Per-command timestamps
    // are not worth repeated APB traffic on the observer core; command order
    // remains exact in the raw DMA ring.
    u32 batchUs = time_us_32();
    u32 records = 0;
    while (available >= 2u && records < TRACE_CONSUME_BUDGET_RECORDS)
    {
        u32 cmd0 = sTraceRing[sConsumed & TRACE_RING_MASK];
        u32 cmd1 = sTraceRing[(sConsumed + 1u) & TRACE_RING_MASK];
        sConsumed += 2u;
        available -= 2u;
        sLastBusUs = batchUs;
        traceDecode(cmd0, cmd1, batchUs);
        records++;
    }
}

// ---- bounded output -------------------------------------------------------
static bool traceBusQuiet(void)
{
    if ((gpio_get(PIN_CEB) & 1u) == 0)
        return false;
    return (u32)(time_us_32() - sLastBusUs) >= TRACE_QUIET_US;
}

static void traceEmitSummary(void)
{
    ntrCardTraceSnapshot snap;
    bool haveSnap = traceReadSnapshot(&snap);

    traceTxPrintf(
        "[trace] n=%lu E3=%lu E4=%lu E5=%lu O=%lu drop=%lu tdrop=%lu\n",
        (unsigned long)sTotal, (unsigned long)sCountE3, (unsigned long)sCountE4,
        (unsigned long)sCountE5, (unsigned long)sCountOther,
        (unsigned long)sDroppedRecords, (unsigned long)sTxDropped);
    traceTxPrintf(
        "[trace] B7=%lu B8=%lu U=%lu FC=%lu F6=%lu\n",
        (unsigned long)sCountB7, (unsigned long)sCountB8, (unsigned long)sCountUsb,
        (unsigned long)sCountFc, (unsigned long)sCountF6);
    if (haveSnap)
        traceTxPrintf("[trace] sd=%lu/%lu sec=%08lX done=%lu/%lu cache=%08lX/%08lX\n",
            (unsigned long)snap.sdState, (unsigned long)snap.sdSequentialState,
            (unsigned long)snap.sdSectorAddress,
            (unsigned long)snap.sdSectorsCompleted,
            (unsigned long)snap.sdSectorCount,
            (unsigned long)snap.cacheSector0, (unsigned long)snap.cacheSector1);
    sLastSummaryTotal = sTotal;
}

static void traceEmitDump(void)
{
    if (sDumpPos == 0)
    {
        if (!traceTxSpace(120))
            return;
        traceTxPrintf(
            "[trace] ANOM id=%02lX (%s) at #%lu; last %lu records\n",
            (unsigned long)sDumpAnomId, traceCmdName(sDumpAnomId),
            (unsigned long)sDumpAnomSeq, (unsigned long)sDumpCount);
    }

    if (sDumpPos >= sDumpCount)
    {
        sAnomalyActive = false;
        sDumpPos = 0;
        return;
    }

    if (!traceTxSpace(96))
        return;

    trace_record_t* r = &sDumpRecs[sDumpPos];
    traceTxPrintf("[trace]  #%lu c0=%08lX c1=%08lX %s\n",
        (unsigned long)(sDumpAnomSeq - (sDumpCount - sDumpPos - 1u)),
        (unsigned long)r->cmd0, (unsigned long)r->cmd1,
        traceCmdName(r->cmd0 >> 24));
    sDumpPos++;
}

static void traceMaybeEmit(void)
{
    if (!traceBusQuiet())
        return;
    if (sAnomalyActive)
        traceEmitDump();
    else if (sTotal != sLastSummaryTotal)
        traceEmitSummary();
}

// ---- public ---------------------------------------------------------------
void ntrCardTraceGetCacheInfo(uint32_t out[4])
{
    out[0] = sSdSectorBuffersSectors[0];
    out[1] = sSdSectorBuffersSectors[1];
    out[2] = sBufferIndex;
    out[3] = (sReadBusy ? 1u : 0u) | (sWriteBusy ? 2u : 0u);
}

void ntrCardTraceInit(void)
{
    // The listener shares PIO0 with the cartridge service. SM2 must be free;
    // do not touch SM0/SM1 or their FIFOs.
    if (pio_sm_is_claimed(pio0, sTraceSm))
        return;
    if (!pio_can_add_program(pio0, &ntr_card_trace_program))
        return;
    pio_sm_claim(pio0, sTraceSm);

    pio_sm_clear_fifos(pio0, sTraceSm);
    sTraceOffset = pio_add_program(pio0, &ntr_card_trace_program);

    pio_sm_config c = ntr_card_trace_program_get_default_config(sTraceOffset);
    sm_config_set_in_pins(&c, PIN_D0);
    // Same shift/autopush configuration and clkdiv as the service program so
    // the two SMs sample identical bytes on the same WREB edge.
    sm_config_set_in_shift(&c, false, true, 32);
    sm_config_set_clkdiv(&c, 1);
    // The listener only ever reads; leave pin directions/function untouched.
    dspicoPioSmInit(pio0, sTraceSm, sTraceOffset, &c);

    // Dedicated DMA channel: never the service DMA0 or SDIO DMA2/3, no
    // completion IRQ, no chaining, write address wraps in hardware.
    sTraceDma = dma_claim_unused_channel(true);
    dma_channel_config dmacfg = dma_channel_get_default_config((uint)sTraceDma);
    channel_config_set_transfer_data_size(&dmacfg, DMA_SIZE_32);
    channel_config_set_read_increment(&dmacfg, false);
    channel_config_set_write_increment(&dmacfg, true);
    channel_config_set_ring(&dmacfg, true, TRACE_RING_BITS);
    channel_config_set_dreq(&dmacfg, pio_get_dreq(pio0, sTraceSm, false));
    dma_channel_configure((uint)sTraceDma, &dmacfg,
        sTraceRing, &pio0->rxf[sTraceSm], TRACE_DMA_COUNT, true);

    dspicoPioSmSetEnabled(pio0, sTraceSm, true);
    sTraceReady = true;
    // Core1 waits while the listener is unavailable during boot.  Wake it
    // only after the listener and its DMA sink are fully configured.
    __sev();
}

void ntrCardTraceCore1Poll(void)
{
    if (!sTraceReady)
    {
        __wfe();
        return;
    }
    traceConsume();
    // Do not begin or continue UART output while an observed transaction is
    // active.  The trace writer is on core1, but bounded output still uses
    // shared peripheral/interconnect resources and must yield to the bus.
    if (!traceBusQuiet())
        return;
    traceTxPump();
    traceMaybeEmit();
    traceTxPump();
}

#endif // NTRC_TRACE_ENABLED
