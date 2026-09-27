#include "common.h"
#include "hardware/vreg.h"
#include "hardware/clocks.h"
#include "hardware/timer.h"
#include "pico/time.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#ifdef CACHE_WATCH_LOG
#include "cacheDiagLog.h"
#ifdef CACHE_SUMMARY_LOG
#include "cachePageMap.h"
#endif
#endif
#include "hardware/gpio.h"
#include "hardware/dma.h"
#include "hardware/structs/scb.h"
#include "pico/binary_info.h"
#include "ntrCard.pio.h"
#include "romData.h"
#include "pico/multicore.h"
#include "blowfish.h"
#include "scrambler.h"
#include "sd/fatfs/ff.h"
#include "ntrCardRom.h"
#include "ntrCardSpiUart.h"
#include "r4.h"
#include "sd/SdCard.h"
#include "scramblerRing.h"
#include "pico/bootrom.h"
#include "hardware/xosc.h"
#include "powerSaving.h"
#ifdef ENABLE_UART_LOG
#include "uartLog.h"
#endif

// Deferred F6 write start, owned by ntrCardRomGameSd.cpp (design section 8.1).
extern "C" void ntrc_gameSdWriteService(void);
// Cartridge transaction generation (design section 6.3).
extern "C" void ntrc_cartBumpTxGen(void);
extern "C" volatile u32 gCartSdRecoveryPending;
extern "C" volatile u32 gCartSdFifoRecovery;
#if CACHE_STAGE >= 1
#include "cacheResourcePlan.h"
#include "cacheInitLog.h"
#endif
#if CACHE_STAGE >= 3
#include "cacheSd.h"
#include "psram.h"
#endif
#ifdef CACHE_WATCH_LOG
#include "hardware/uart.h"
#include "hardware/exception.h"
#include "hardware/structs/systick.h"
#endif

#ifndef CACHE_L2_MODE
#define CACHE_L2_MODE 5
#endif

static u32 sProgramOffset;
FATFS sFatFs;
SdCard gSdCard;
static bool sIsSdCardMounted;
#if CACHE_STAGE >= 3
static bool sPsramPinsInitialized;
#endif

#ifdef CACHE_WATCH_LOG
// Core0 writes only these markers; core1 owns the runtime UART in this build.
// Samples are diagnostic, not atomic storage-completion observations.
static volatile bool sWatchReady, sWatchL2;
#ifdef CACHE_WATCH_ACTIVE_MARKERS
static volatile u32 sWatchPhase, sWatchLoops;
static volatile u32 sWatchRecoveries, sWatchRecoverCmd, sWatchRecoverPc;
static volatile u32 sWatchRecoverRemaining, sWatchRecoverTx;
#endif
extern "C" void rp2350_sdio_debug_words(u32 out[4]);
extern volatile u32 gCartSdE4LenArmed;

// Core1-local interrupt: no timer alarm, shared SDK timer lock, or Core0
// callback. The periodic interrupt also bounds a tick/check/WFE race to 1 ms.
static volatile bool sWatchTick;
static volatile u32 sWatchTicks;
static void __scratch_x("cpu1") cacheWatchTick(void)
{
    sWatchTick = true;
    sWatchTicks++;
}

static void cacheWatchClockInit(void)
{
    // SDK start_all_ticks() supplies the external SysTick reference at 1 MHz.
    // Ordinary sleep preserves this wake source; don't use deep/WIC sleep.
    scb_hw->scr &= ~ARM_CPU_PREFIXED(SCR_SLEEPDEEP_BITS);
    systick_hw->csr = 0;
    exception_set_exclusive_handler(SYSTICK_EXCEPTION, cacheWatchTick);
    exception_set_priority(SYSTICK_EXCEPTION, PICO_LOWEST_EXCEPTION_PRIORITY);
    systick_hw->rvr = CACHE_WATCH_POLL_US - 1u;
    systick_hw->cvr = 0;
    systick_hw->csr = M33_SYST_CSR_TICKINT_BITS | M33_SYST_CSR_ENABLE_BITS;
}

#ifdef CACHE_SUMMARY_LOG
static void cacheSummaryPoll(void)
{
    static cacheWatchGate gate;
    static cacheTextStream stream;
    static cacheSdCounters counters;
    static u32 lastUs, lastUsed, lastHit, lastOther, lastCapacity;
    static bool reported;
    if (!sWatchReady) return;
    u32 now = time_us_32();
    if (!cacheWatchGateWarm(&gate, now)) return;
    const volatile cs_state* cs = cacheSdState();
    const volatile cacheSdTxSnapshot* tx = cacheSdLastTx();
    const u32 activity[7] = {gNtrRomEmu.cmd0, gNtrRomEmu.cmd1,
        (u32)gNtrRomEmu.wordIdx, cs->c_e4_busy, cs->c_e4_ready_queued,
        tx->sends, cs->write_epoch};
    bool available = gpio_get(PIN_CEB) && gpio_get(PIN_CS2) &&
        (!gComputeScrambler || SCR_RING_WRAP(gScramblerRingWPtr + 1) ==
                               gNtrRomEmu.scrRingRPtr);
    if (!cacheWatchGateIdle(&gate, now, available, activity)) return;
    if (stream.format)
    {
        for (unsigned n = 0; n < 16; n++)
        {
            if (!gpio_get(PIN_CEB) || !gpio_get(PIN_CS2) ||
                (gComputeScrambler && SCR_RING_WRAP(gScramblerRingWPtr + 1) !=
                                      gNtrRomEmu.scrRingRPtr) ||
                !uart_is_writable(uart1)) return;
            int ch = cacheTextNext(&stream);
            if (ch < 0) break;
            uart_putc_raw(uart1, (char)ch);
        }
        return;
    }
    if (reported && (u32)(now - lastUs) < 5000000u) return;
    cacheSdGetCounters(&counters);
    u32 mask = sWatchL2 ? psramChipReadyMask() & 0xFu : 0u;
    u32 capacity = (u32)__builtin_popcount(mask) *
        CACHE_PAGE_ACTIVE_SETS * CACHE_PAGE_DATA_BYTES;
    if (reported && capacity == lastCapacity &&
        counters.page_valid_sectors == lastUsed &&
        counters.demand_hit_ok == lastHit &&
        counters.demand_non_psram_ok == lastOther) return;
    lastUs = now;
    lastCapacity = capacity;
    lastUsed = counters.page_valid_sectors;
    lastHit = counters.demand_hit_ok;
    lastOther = counters.demand_non_psram_ok;
    reported = true;
    cacheSummaryStartLine(&stream, capacity, &counters);
}
#else
static void cacheWatchPoll(void)
{
    static u32 lastUs, sample, blockedUs;
    static bool blockedStream;
    static cacheTextStream stream;
    static unsigned part = 5;
    static cacheSdCounters counters; // avoid adding a snapshot to core1's stack
    static cacheWatchGate gate;
    // Keep peripheral reads and formatting out of the scrambler's busy path.
    if (!sWatchReady)
        return;
    u32 now = time_us_32();
    // No metadata scan or runtime UART during initial cartridge negotiation.
    if (!cacheWatchGateWarm(&gate, now)) return;
    const volatile cs_state* cs = cacheSdState();
    const volatile cacheSdTxSnapshot* tx = cacheSdLastTx();
    const u32 activity[7] = {gNtrRomEmu.cmd0, gNtrRomEmu.cmd1,
        (u32)gNtrRomEmu.wordIdx, cs->c_e4_busy, cs->c_e4_ready_queued,
        tx->sends, cs->write_epoch};
    bool available = gpio_get(PIN_CEB) && gpio_get(PIN_CS2) &&
        (!gComputeScrambler || SCR_RING_WRAP(gScramblerRingWPtr + 1) ==
                               gNtrRomEmu.scrRingRPtr);
    bool idle = cacheWatchGateIdle(&gate, now, available, activity);
    bool stalled = cacheWatchGateStalled(&gate, now);
    if (!idle && stalled)
    {
        // Post-stall summary only: no E4/E5 progress for five seconds, and
        // this function is only called when the producer has no work.
        // Do not require CS high: that is exactly the gate being diagnosed.
        if (!blockedStream)
        {
            if ((u32)(now - blockedUs) < CACHE_WATCH_STALLED_US) return;
            blockedUs = now;
            blockedStream = true;
            part = 5; // abandon any paused full group, start a delimited line
            unsigned n = 0;
            stream.values[n++] = now;
            stream.values[n++] = sWatchTicks;
            stream.values[n++] = gpio_get(PIN_CEB);
            stream.values[n++] = gpio_get(PIN_CS2);
            stream.values[n++] = gComputeScrambler;
            stream.values[n++] = (u32)gScramblerRingWPtr;
            stream.values[n++] = (u32)gNtrRomEmu.scrRingRPtr;
            stream.values[n++] = pio_sm_get_pc(pio0, 0) - sProgramOffset;
            stream.values[n++] = gNtrRomEmu.cmd0;
            stream.values[n++] = gNtrRomEmu.wordIdx;
            stream.values[n++] = gCacheSdReadReady;
            stream.values[n++] = cs->c_e4_busy;
            stream.values[n++] = cs->c_e4_ready_queued;
            stream.values[n++] = cs->c_e5_ok;
            cacheTextStart(&stream,
                "\n[watch-blocked] v=4 us=%u ticks=%u ceb=%u cs2=%u scramble=%u w=%08X r=%08X rel=%u cmd=%08X wi=%u ready=%u e4=%u/%u e5=%u\n", n);
        }
        for (unsigned n = 0; n < 16; n++)
        {
            if ((gComputeScrambler && SCR_RING_WRAP(gScramblerRingWPtr + 1) !=
                 gNtrRomEmu.scrRingRPtr) || !uart_is_writable(uart1)) return;
            int ch = cacheTextNext(&stream);
            if (ch < 0) { blockedStream = false; break; }
            uart_putc_raw(uart1, (char)ch);
        }
        return;
    }
    if (blockedStream)
    {
        // New host activity cancels this post-stall record, not the producer.
        stream.format = nullptr;
        blockedStream = false;
        part = 5;
    }
    if (!idle) return;
    // Formatting and draining are both incremental. Return to the producer
    // whenever the scrambler needs words or either cartridge bus becomes active.
    if (!gpio_get(PIN_CEB) || !gpio_get(PIN_CS2) ||
        (gComputeScrambler && SCR_RING_WRAP(gScramblerRingWPtr + 1) !=
                              gNtrRomEmu.scrRingRPtr)) return;
    if (stream.format)
    {
        for (unsigned n = 0; n < 16; n++)
        {
            if (!gpio_get(PIN_CEB) || !gpio_get(PIN_CS2) ||
                (gComputeScrambler && SCR_RING_WRAP(gScramblerRingWPtr + 1) !=
                                      gNtrRomEmu.scrRingRPtr) ||
                !uart_is_writable(uart1))
                return;
            int ch = cacheTextNext(&stream);
            if (ch < 0) break;
            uart_putc_raw(uart1, (char)ch);
        }
        return;
    }
#if defined(CACHE_SHADOW_STRIPES_4CHIP) || defined(CACHE_FULL_PAGE_4CHIP)
    constexpr unsigned kCacheWatchDetailLines = 6;
#elif defined(CACHE_PROBE_MISMATCH_DIAG)
    constexpr unsigned kCacheWatchDetailLines = 5;
#else
    constexpr unsigned kCacheWatchDetailLines = 4;
#endif
    if (part < kCacheWatchDetailLines)
    {
        cacheDiagStartLine(&stream, part++, sample, &counters);
        return;
    }
    if (now - lastUs < 1000000u)
        return;
    lastUs = now;
    sample++;
    cacheSdGetCounters(&counters);
    u32 flags = (cs->intent_valid ? 1u : 0u) |
                (cs->intent_consumed ? 2u : 0u) |
                (cs->completion_valid ? 4u : 0u) |
                (cs->binding_valid ? 8u : 0u) |
                (cs->write_pending ? 16u : 0u) |
                (cs->intent_faulted ? 32u : 0u);
    u32 sd[4];
    rp2350_sdio_debug_words(sd);
    unsigned field = 0;
    stream.values[field++] = (uint32_t)(sample);
    stream.values[field++] = (uint32_t)(now);
    stream.values[field++] = (uint32_t)(sWatchTicks);
#ifdef CACHE_WATCH_ACTIVE_MARKERS
    stream.values[field++] = (uint32_t)(sWatchLoops);
    stream.values[field++] = (uint32_t)(sWatchPhase);
#endif
    stream.values[field++] = (uint32_t)(sWatchL2);
    stream.values[field++] = (uint32_t)(gSdCard.DebugState());
    stream.values[field++] = (uint32_t)(gSdCard.DebugSectorAddress());
    stream.values[field++] = (uint32_t)(gSdCard.GetSectorsCompleted());
    stream.values[field++] = (uint32_t)(gSdCard.GetSectorCount());
    stream.values[field++] = (uint32_t)(gpio_get(PIN_CEB));
    stream.values[field++] = (uint32_t)(gpio_get(PIN_CS2));
    stream.values[field++] = (uint32_t)(pio_sm_get_pc(pio0, 0));
    stream.values[field++] = (uint32_t)(pio_sm_get_pc(pio0, 0) - sProgramOffset);
    stream.values[field++] = (uint32_t)(cs->intent.sector);
    stream.values[field++] = (uint32_t)(flags);
    stream.values[field++] = (uint32_t)(gCacheSdReadReady);
    stream.values[field++] = (uint32_t)(cs->c_protocol_fault);
    stream.values[field++] = (uint32_t)(cs->c_sd_retry);
    stream.values[field++] = (uint32_t)(cs->c_sd_fault);
    stream.values[field++] = (uint32_t)(cacheSdErrors());
    stream.values[field++] = (uint32_t)(sd[0]);
    stream.values[field++] = (uint32_t)(sd[1]);
    stream.values[field++] = (uint32_t)(sd[2]);
    stream.values[field++] = (uint32_t)(sd[3]);
    stream.values[field++] = (uint32_t)(dma_channel_is_busy(2));
    stream.values[field++] = (uint32_t)(dma_channel_is_busy(3));
    stream.values[field++] = (uint32_t)(gNtrRomEmu.cmd0);
    stream.values[field++] = (uint32_t)(gNtrRomEmu.wordIdx);
    stream.values[field++] = (uint32_t)(gCartSdE4LenArmed);
    stream.values[field++] = (uint32_t)(cacheSdOfferId());
    stream.values[field++] = (uint32_t)(cacheSdAckedOfferId());
    stream.values[field++] = (uint32_t)(cs->c_e4_busy);
    stream.values[field++] = (uint32_t)(cs->c_e4_ready_queued);
    stream.values[field++] = (uint32_t)(cs->c_e4_ack_failed);
    stream.values[field++] = (uint32_t)(cs->c_e5_ok);
    stream.values[field++] = (uint32_t)(cs->c_e5_no_ack);
    stream.values[field++] = (uint32_t)(cs->first_fault.kind);
    stream.values[field++] = (uint32_t)(cs->first_fault.seq);
    stream.values[field++] = (uint32_t)(cs->first_fault.sampled_ready);
    stream.values[field++] = (uint32_t)(cacheSdFills());
    stream.values[field++] = (uint32_t)(cacheSdHits());
    stream.values[field++] = (uint32_t)(gCartSdFifoRecovery);
    stream.values[field++] = (uint32_t)(gCartSdRecoveryPending);
    stream.values[field++] = (uint32_t)(tx->sends);
    stream.values[field++] = (uint32_t)(tx->sector);
    stream.values[field++] = (uint32_t)(tx->offer);
    stream.values[field++] = (uint32_t)(tx->head);
    stream.values[field++] = (uint32_t)(tx->tail);
#ifdef CACHE_WATCH_ACTIVE_MARKERS
    stream.values[field++] = (uint32_t)(sWatchRecoveries);
    stream.values[field++] = (uint32_t)(sWatchRecoverCmd);
    stream.values[field++] = (uint32_t)(sWatchRecoverPc);
    stream.values[field++] = (uint32_t)(sWatchRecoverRemaining);
    stream.values[field++] = (uint32_t)(sWatchRecoverTx);
#endif
    cacheTextStart(&stream,
        "\n[watch] v=4 q=%lu us=%lu ticks=%lu "
#ifdef CACHE_WATCH_ACTIVE_MARKERS
        "marks=1 loop=%lu phase=%lu "
#else
        "marks=0 "
#endif
        "l2=%u sd=%d sec=%08lX done=%lu/%lu ceb=%u cs2=%u pc=%lu rel=%lu\n"
        "[watch] req=%08lX flags=%02lX ready=%lu proto=%lu retry=%lu fault=%lu err=%lu io=%lu/%lu/%lu/%lu dma=%u%u cmd=%08lX wi=%lu e4=%lu\n"
        "[watch] offer=%lu ack=%lu e4=%lu/%lu/%lu e5=%lu/%lu first=%u seq=%lu sampled=%lu fill=%lu hitTry=%lu fifo=%lu pending=%lu\n"
        "[tx] n=%lu sec=%08lX offer=%lu head=%08lX tail=%08lX"
#ifdef CACHE_WATCH_ACTIVE_MARKERS
        " kicks=%lu lastCmd=%08lX rel=%lu remain=%lu txfifo=%lu"
#endif
        "\n", field);
    part = 0;
}
#endif

static void __scratch_x("cpu1") cacheWatchIdle(void)
{
    if (sWatchTick)
    {
        sWatchTick = false;
#ifdef CACHE_SUMMARY_LOG
        cacheSummaryPoll();
#else
        cacheWatchPoll();
#endif
    }
    // Also wakes immediately on the existing scrambler-consumer SEV.
    __wfe();
}
#endif
#ifdef CACHE_WATCH_ACTIVE_MARKERS
#define WATCH_PHASE(p) (sWatchPhase = (p))
#else
#define WATCH_PHASE(p) ((void)0)
#endif

#ifdef DETECT_CONSOLE_TYPE
static void setRomToDsiRom(void)
{
    gNtrRomEmu.romData = gDsiRom;
    gNtrRomEmu.romSize = ((u32)gDsiRomSize + 511) & ~511;
}
#endif

static void resetNtrCard(void)
{
    pwr_disableAfterBootPowerSaving();
    ntrc_setNormalMode();
    gNtrRomEmu.securePhase1 = false;
    gNtrRomEmu.cmdScramble = false;
    gNtrRomEmu.dataScramble = false;
    gComputeScrambler = false;
    gNtrRomEmu.scrRingRPtr = gScramblerRing;
    gScramblerRingWPtr = gScramblerRing;
    gNtrRomEmu.wordIdx = 0;
    gNtrRomEmu.twlMode = false;
    gNtrRomEmu.readDataDestination = nullptr;
    gNtrRomEmu.readDataCompleteHandler = nullptr;
    gNtrRomEmu.readDataLimit = 0;
    // The fifo clear below also drops a pending E4 pre-armed length word.
    // Opening a new transaction generation invalidates it unambiguously.
    ntrc_cartBumpTxGen();
#ifdef ENABLE_R4_MODE
    ntrc_resetR4();
#endif
    dma_channel_abort(0);
    pio_sm_set_enabled(pio0, 0, false);
    pio_sm_set_pindirs_with_mask(pio0, 0, 0, PIN_INPUT_MASK);
    pio_sm_clear_fifos(pio0, 0);
    pio_sm_restart(pio0, 0);
    pio_sm_clkdiv_restart(pio0, 0);
    irq_clear(PIO0_IRQ_0);
    irq_set_enabled(PIO0_IRQ_0, true);
    pio_sm_exec(pio0, 0, pio_encode_jmp(sProgramOffset));
    pio_sm_set_enabled(pio0, 0, true);
#if CACHE_STAGE >= 3
    // New cartridge session (design section 9.1 / R7): drop any read binding,
    // completion or intent from the previous session so an old result can
    // never bind to it. Already-accepted write tasks are tracked separately
    // and are not released here.
    cacheSdResetCart();
#endif
#ifdef DETECT_CONSOLE_TYPE  
    setRomToDsiRom();
    gNtrRomEmu.cardId = 0xC00000C2;
#endif
#ifdef DSPICO_ENABLE_WRFUXXED
    ntrc_resetSpiUart();
#endif
}

#ifdef ENABLE_PREVENT_DSI_AUTOBOOT
static u64 sResetStart;
#endif

static void __time_critical_func(gpioIrq)(uint gpio, u32 events)
{
#ifdef ENABLE_PREVENT_DSI_AUTOBOOT
    u64 time = time_us_64();
#endif
    if (gpio == PIN_CEB)
    {
        if (events & GPIO_IRQ_EDGE_RISE)
        {
        // Transfer ended (CEB high). A healthy SM is ALWAYS back at pc 0..2
        // (set pindirs / wait CEB) by now. Anything from pc 3 (set x,7) up to
        // 23 means the console abandoned the transaction (or our response was
        // late) and the SM would carry stale state into the NEXT command:
        //  - pc 4..7: mid-cmd_loop -> next command's bytes get captured with a
        //    wrong byte count
        //  - pc 8..11: stalled on the length autopull -> deaf until a word is
        //    pushed, then it eats the next command's strobes (r66 boot-3 final
        //    state: PC=8 with the dispatcher dead)
        //  - pc 12..23: mid payload -> missed WREB edges, waits forever
        // Re-sync ALL of it: kick the SM to start, drop any half-received
        // command left in the RX FIFO and reset the dispatcher's word index -
        // r66 proved a wordIdx left mid-command turns one lost transaction
        // into a garbage-decode cascade (D=1 -> U=1). The CEB level check
        // guards against a late-sampled rise firing while the next command is
        // already underway; a rise with the line low is a phantom (line
        // glitch or a batched fall we already handled).
        u32 relPc = pio_sm_get_pc(pio0, 0) - sProgramOffset;
        if ((gpio_get(PIN_CEB) & 1u) == 0)
        {
            return;
        }
        if (gCartSdRecoveryPending || (relPc >= 3 && relPc <= 23))
        {
            // The dispatch IRQ (now top priority) preempts this handler, so a
            // command arriving mid-kick would get its response pushed into a
            // SM/FIFO state that is about to be wiped. Mask it for the kick;
            // the level IRQ re-pends afterwards and the FSTAT guard drops
            // the (cleared) stale entry.
            irq_set_enabled(PIO0_IRQ_0, false);
            // Recheck after masking dispatch; stop capture before resetting
            // both FIFOs. A new transaction during recovery may be rejected.
            if (!gpio_get(PIN_CEB))
            {
                irq_set_enabled(PIO0_IRQ_0, true);
                return;
            }
#ifdef CACHE_WATCH_ACTIVE_MARKERS
            sWatchRecoverCmd = gNtrRomEmu.cmd0;
            sWatchRecoverPc = pio_sm_get_pc(pio0, 0) - sProgramOffset;
            sWatchRecoverRemaining = dma_hw->ch[0].transfer_count;
            sWatchRecoverTx = pio_sm_get_tx_fifo_level(pio0, 0);
            sWatchRecoveries++;
#endif
            pio_sm_set_enabled(pio0, 0, false);
            // Kill a still-running response DMA FIRST: after clear_fifos an
            // orphaned E5/B7 feed would keep writing into the empty TX FIFO
            // (r67: TXF=2 with the SM parked - the next command would read a
            // stale sector word as its length).
            dma_channel_abort(0);
            // clear_fifos only resets the FIFO pointers; a command aborted
            // mid cmd_loop leaves partial bytes in the ISR, shifting the next
            // command's word0 off the 32-bit boundary. pio_sm_restart clears
            // the ISR contents and both shift counters (PC untouched - the
            // jmp below handles it).
            pio_sm_clear_fifos(pio0, 0);
            pio_sm_restart(pio0, 0);
            pio_sm_exec(pio0, 0, pio_encode_jmp(sProgramOffset));
            gNtrRomEmu.wordIdx = 0;
            // clear_fifos also drops a pending E4 pre-armed length word -
            // keep the software flag in sync or the next poll would skip its
            // own length push and stall the SM at the autopull. A new
            // transaction generation invalidates it (design section 6.3).
            ntrc_cartBumpTxGen();
            pio_sm_set_enabled(pio0, 0, true);
            irq_clear(PIO0_IRQ_0);
            irq_set_enabled(PIO0_IRQ_0, true);
        }
        return;
        }
    }
    if (gpio == PIN_RST)
    {
        if (events & GPIO_IRQ_EDGE_FALL)
        {
            pio_sm_set_enabled(pio0, 0, false);
            pio_sm_set_pindirs_with_mask(pio0, 0, 0, PIN_INPUT_MASK);
        }
        if (events & GPIO_IRQ_EDGE_RISE)
        {
            resetNtrCard();
        #ifdef ENABLE_PREVENT_DSI_AUTOBOOT
            u32 resetTime = time - sResetStart;
            if (resetTime > 700000)
                pio_sm_set_enabled(pio0, 0, false);
            sResetStart = time;
        #endif
        }   
    }        
}

void __scratch_x("cpu1") core1_entry(void)
{
    irq_set_mask_enabled(~0u, false);
#ifdef CACHE_WATCH_LOG
    cacheWatchClockInit();
#else
    scb_hw->scr |= ARM_CPU_PREFIXED(SCR_SLEEPDEEP_BITS);
#endif
    while (!gComputeScrambler)
    {
        gScramblerRingWPtr = gScramblerRing;
#if NTRC_TRACE_ENABLED
        // Cartridge tracing is a side channel served by core1 while the
        // scrambler is idle (unscrambled game / DLDI). It never touches the
        // service SM, DMA0 or the SDIO state machine.
        ntrCardTraceCore1Poll();
#elif defined(CACHE_WATCH_LOG)
        cacheWatchIdle();
#else
        __wfe();
#endif
    }
    while (1)
    {
        u32* wPtr = gScramblerRingWPtr;
        u32* next = SCR_RING_WRAP(wPtr + 1);
        if (next == gNtrRomEmu.scrRingRPtr)
        {
#ifdef CACHE_WATCH_LOG
            cacheWatchIdle();
#else
            __wfe();
#endif
            continue;
        }

        *wPtr = scr_getNext32(&gScramblerState);
        gScramblerRingWPtr = next;
    }
}

static void initSd(void)
{
    memset(&sFatFs, 0, sizeof(sFatFs));

    //try mounting 3 times
    bool ok = false;
    int lastResult = -1;
    for (int i = 0; i < 3; i++)
    {
        FRESULT mountResult = f_mount(&sFatFs, "0:", 1);
        lastResult = (int)mountResult;
#if defined(ENABLE_UART_LOG) && !defined(CACHE_SUMMARY_LOG)
        uartLogPrintfBlocking("[sd] f_mount try %d -> %d\n", i, (int)mountResult);
#endif
        if (mountResult == FR_OK)
        {
            ok = true;
            sIsSdCardMounted = true;
            break;
        }
        else if (mountResult == FR_NO_FILESYSTEM)
        {
            break;
        }
    }
    if (!ok)
    {
        // FATFS FRESULT: 0=OK 1=DISK_ERR 2=INT_ERR 3=NOT_READY 4=NO_FILE
        // 5=NO_PATH 6=INVALID_NAME 7=DENIED 8=EXIST 9=INVALID_OBJECT
        // 10=WRITE_PROTECTED 11=INVALID_DRIVE 12=NOT_ENABLED 13=NO_FILESYSTEM
        // 14=MKFS_ABORTED 15=TIMEOUT. A repeat of 1/2/3/15 across tries points
        // at the SDIO/driver layer; 13 alone means the FAT got read but has no
        // filesystem.
#ifdef ENABLE_UART_LOG
        uartLogPrintfBlocking("[sd] f_mount all FAIL, last=%d\n", lastResult);
#endif
        sIsSdCardMounted = false;
    }
}

static void tryRebootToBootsel(void)
{
    if (!sIsSdCardMounted)
    {
        // RP2354A has no usable USB (pads are UART): halt and let the
        // failure be visible on the NDSL instead of a dead BOOTSEL reboot.
        while (1) __wfi();
    }
}

static inline void earlyGpioInit(void)
{
    // PSRAM was configured and self-tested before this function. Preserve its
    // mux, direction and chip-select levels while resetting the other pins.
    // Note that we rely on hardware reset having enabled pull-downs.
    uint32_t gpioInitMask = 0xFFFFFFFFu;
#if CACHE_STAGE >= 3
    if (sPsramPinsInitialized)
        gpioInitMask &= ~PSRAM_PIN_MASK;
#endif
    gpio_init_mask(gpioInitMask);

    // Set NTRCARD IRQ pin low.
    // This needs to happen immediately.
    gpio_put(PIN_IRQ, false);
    gpio_set_dir(PIN_IRQ, GPIO_OUT);
    gpio_disable_pulls(PIN_IRQ);

    // Set SDIO and NTRCARD pin drive strengths.
    // 4 mA is the default after reset. 2 mA is enough.
    gpio_set_drive_strength(SDIO_CLK, GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_drive_strength(SDIO_CMD, GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_drive_strength(SDIO_D0, GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_drive_strength(SDIO_D1, GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_drive_strength(SDIO_D2, GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_drive_strength(SDIO_D3, GPIO_DRIVE_STRENGTH_2MA);

    gpio_set_drive_strength(PIN_D0, GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_drive_strength(PIN_D1, GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_drive_strength(PIN_D2, GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_drive_strength(PIN_D3, GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_drive_strength(PIN_D4, GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_drive_strength(PIN_D5, GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_drive_strength(PIN_D6, GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_drive_strength(PIN_D7, GPIO_DRIVE_STRENGTH_2MA);

    // Setup SDIO and NTRCARD pin pulls.
    // SDIO CMD/DAT required when there are no external pull-ups.
    gpio_disable_pulls(SDIO_CLK);
#if 0
    // We have no external pull-ups.
    gpio_pull_up(SDIO_CMD);
    gpio_pull_up(SDIO_D0);
    gpio_pull_up(SDIO_D1);
    gpio_pull_up(SDIO_D2);
    gpio_pull_up(SDIO_D3);
#else
    // We have external pull-ups.
    gpio_disable_pulls(SDIO_CMD);
    gpio_disable_pulls(SDIO_D0);
    gpio_disable_pulls(SDIO_D1);
    gpio_disable_pulls(SDIO_D2);
    gpio_disable_pulls(SDIO_D3);
#endif

    gpio_disable_pulls(PIN_RST);
    gpio_disable_pulls(PIN_CEB);
    gpio_disable_pulls(PIN_WREB);
    gpio_disable_pulls(PIN_D0);
    gpio_disable_pulls(PIN_D1);
    gpio_disable_pulls(PIN_D2);
    gpio_disable_pulls(PIN_D3);
    gpio_disable_pulls(PIN_D4);
    gpio_disable_pulls(PIN_D5);
    gpio_disable_pulls(PIN_D6);
    gpio_disable_pulls(PIN_D7);
    gpio_disable_pulls(PIN_CS2);

    // Set SDIO and NTRCARD DAT pin slew rate.
    // Default slew rate after reset is slow. Good enough for 25 MHz.
#if 0
    gpio_set_slew_rate(SDIO_CLK, GPIO_SLEW_RATE_FAST);
    gpio_set_slew_rate(SDIO_CMD, GPIO_SLEW_RATE_FAST);
    gpio_set_slew_rate(SDIO_D0, GPIO_SLEW_RATE_FAST);
    gpio_set_slew_rate(SDIO_D1, GPIO_SLEW_RATE_FAST);
    gpio_set_slew_rate(SDIO_D2, GPIO_SLEW_RATE_FAST);
    gpio_set_slew_rate(SDIO_D3, GPIO_SLEW_RATE_FAST);

    gpio_set_slew_rate(PIN_D0, GPIO_SLEW_RATE_FAST);
    gpio_set_slew_rate(PIN_D1, GPIO_SLEW_RATE_FAST);
    gpio_set_slew_rate(PIN_D2, GPIO_SLEW_RATE_FAST);
    gpio_set_slew_rate(PIN_D3, GPIO_SLEW_RATE_FAST);
    gpio_set_slew_rate(PIN_D4, GPIO_SLEW_RATE_FAST);
    gpio_set_slew_rate(PIN_D5, GPIO_SLEW_RATE_FAST);
    gpio_set_slew_rate(PIN_D6, GPIO_SLEW_RATE_FAST);
    gpio_set_slew_rate(PIN_D7, GPIO_SLEW_RATE_FAST);
#endif

    // SDIO CLK pin needs to be low before init.
    gpio_put(SDIO_CLK, false);
    gpio_set_dir(SDIO_CLK, true);

    // Disable all unused GPIO inputs. Saves a little power.
#if 1
    uint32_t usedPins = NTRC_PIN_MASK | SDIO_PIN_MASK | DEV_UART_PIN_MASK;
#if CACHE_STAGE >= 3
    if (sPsramPinsInitialized)
        usedPins |= PSRAM_PIN_MASK;
#endif
    for(uint32_t i = 0; i < NUM_BANK0_GPIOS; i++)
    {
        if(!(usedPins & 1u))
        {
            gpio_set_input_enabled(i, false);
        }
        usedPins >>= 1;
    }
#endif
}


int __time_critical_func(main)()
{
    bi_decl(bi_program_description("Ntr card emulator"));
    bi_decl(bi_pin_mask_with_name(0xFF000, "Ntr card D0-D7"));
    bi_decl(bi_1pin_with_name(PIN_IRQ, "Ntr card irq"));
    bi_decl(bi_1pin_with_name(PIN_CEB, "Ntr card ceb (rom enable)"));
    bi_decl(bi_1pin_with_name(PIN_WREB, "Ntr card wreb (clock)"));
    bi_decl(bi_1pin_with_name(PIN_RST, "Ntr card reset"));
    bi_decl(bi_1pin_with_name(PIN_CS2, "Ntr card cs2 (spi enable)"));

    // u64 bootTime = time_us_64();

    // RP2354A: 200 MHz needs the 1.15 V VREG rail; set_sys_clock_khz selects
    // the RP2350 PLL path. The cartridge IRQ timing was designed at 200 MHz.
    vreg_set_voltage(VREG_VOLTAGE_1_15);
    sleep_us(100);
    set_sys_clock_khz(200000, true);
#ifdef ENABLE_UART_LOG
    uartLogInit();
#endif
#if CACHE_STAGE >= 1
    // Register resource ownership and open the bounded init-log ring. Both are
    // pure bookkeeping: no peripheral is claimed, no PSRAM is touched and the
    // cartridge path is unchanged (design stage S1).
    cacheResourcePlanInit();
    cacheInitLogInit();
    {
        const cacheResourcePlan* rp = cacheResourcePlanGet();
        cacheInitLogEmit(CACHE_LOG_EV_BOOT, (u8)CACHE_STAGE, 0, 0, 0,
                         clock_get_hz(clk_sys) / 1000u, 0, 0, 0, 0, 0);
        cacheInitLogEmit(CACHE_LOG_EV_RESOURCE, (u8)CACHE_STAGE, 0, 0, 0,
                         rp->pio0Words, rp->pio1Words, rp->pio2Words,
                         rp->dmaMask, rp->gpioMask, rp->initLogBytes);
    }
#endif
#if CACHE_STAGE >= 3
    // Initialise the cache state machine before the cartridge service starts:
    // E3/E4/E5 may fire as soon as resetNtrCard() enables PIO0 SM0. The PSRAM
    // device initialization also finishes before cartridge service starts.
    cacheSdInit();
#endif

    // Finish device reset/self-test before PIO0 or cartridge IRQs start.
#if CACHE_STAGE >= 3
    // L2 bring-up is layered (design section 8.4): resource -> device ->
    // self-test -> runtime enable. The diagnostic mode (section 10) decides
    // how far to go; a forced-off build overrides any mode. Any failure keeps
    // the normal SD service and never fakes a hit.
    {
        u8 l2Mode = (u8)CACHE_L2_MODE;
#ifdef CACHE_L2_DISABLED
        l2Mode = CACHE_L2_MODE_M0;
#endif
        cacheSdSetMode(l2Mode);
#if defined(ENABLE_UART_LOG) && !defined(CACHE_SUMMARY_LOG)
        uartLogPrintf("[cache-boot] start mode=M%u\n", (unsigned)l2Mode);
        uartLogFlush();
#endif
        bool enabled = false;
        u32 devMask = 0;
        if (l2Mode == CACHE_L2_MODE_M0)
        {
            cacheSdSetEnabled(false);
        }
        else
        {
            psramGpioInit();
            sPsramPinsInitialized = true;
            bool res = psramEngineInit();
            if (res)
                psramSetClockDiv(PSRAM_PIO_CLKDIV);
            bool dev = res && psramInitDevice();
            devMask = psramChipReadyMask();
            bool selftest = false;
            // M1/M2 never touch the data path, so DEVICE_READY is enough for
            // them; M3+ starts PSRAM writes/reads and therefore requires the
            // self-test to have verified an isolated block.
            if (dev && (l2Mode >= CACHE_L2_MODE_M3
#ifdef CACHE_PSRAM_BOOT_DIAG
                        || l2Mode == CACHE_L2_MODE_M2
#endif
                       ))
                selftest = psramSelfTest();
#ifdef CACHE_PSRAM_BOOT_DIAG
            psramCacheWindowDiag lowDiag = {};
            if (selftest)
                lowDiag = psramCacheWindowSelfTest();
#ifdef ENABLE_UART_LOG
            uartLogPrintf("[psram-low] flags=%02lX pioOff=%lu pio=%02X/%02X sioOff=%lu sio=%02X/%02X\n",
                          (unsigned long)lowDiag.flags,
                          (unsigned long)lowDiag.pioOff,
                          (unsigned)lowDiag.expectedPio, (unsigned)lowDiag.actualPio,
                          (unsigned long)lowDiag.sioOff,
                          (unsigned)lowDiag.expectedSio, (unsigned)lowDiag.actualSio);
            uartLogFlush();
#endif
#endif
            enabled = (l2Mode <= CACHE_L2_MODE_M2) ? dev : selftest;
            if (enabled)
                psramSetRuntimeEnabled(true);
            cacheSdSetEnabled(enabled);
        }
#ifdef CACHE_WATCH_LOG
        sWatchL2 = enabled;
#endif
#if defined(ENABLE_UART_LOG) && !defined(CACHE_SUMMARY_LOG)
        uartLogPrintf("[cache-boot] mode=M%u enabled=%u state=%u mask=%02lX\n",
                      (unsigned)l2Mode, (unsigned)enabled,
                      (unsigned)psramGetRuntimeState(), (unsigned long)devMask);
        uartLogFlush();
#endif
        cacheInitLogEmit(CACHE_LOG_EV_L2_STATE, l2Mode, 0, enabled ? 0 : 1,
                         (u32)psramGetRuntimeState(), enabled, devMask,
                         0, 0, 0, 0);
    }
#endif

    dma_channel_claim(0);

    memset(&gNtrRomEmu, 0, sizeof(gNtrRomEmu));


    // We support DSi mode if the firmware is dual mode, or if a single rom has the DSi flag set
#ifndef DETECT_CONSOLE_TYPE
    if (gDefaultRom[0x12] & 2)
    {
#endif
        gNtrRomEmu.cardId = CARD_ID_TWL;
#ifndef DETECT_CONSOLE_TYPE
    }
    else
    {
        gNtrRomEmu.cardId = CARD_ID_NTR;
    }
#endif

    multicore_launch_core1(core1_entry);

    earlyGpioInit();

#ifdef DETECT_CONSOLE_TYPE
    setRomToDsiRom();
    gNtrRomEmu.isDSMode = true;
#else
    gNtrRomEmu.romData = gDefaultRom;
    gNtrRomEmu.romSize = (u32)gDefaultRomSize;
    gNtrRomEmu.romSize = (gNtrRomEmu.romSize + 511) & ~511;
#endif

    sProgramOffset = pio_add_program(pio0, &ntr_card_program);
#ifdef DSPICO_ENABLE_WRFUXXED
    u32 spiUartProgOffs = pio_add_program(pio0, &ntr_card_spi_program);
#endif
    pio_sm_config c = ntr_card_program_get_default_config(sProgramOffset);
    sm_config_set_out_pins(&c, PIN_D0, 8);
    sm_config_set_in_pins(&c, PIN_D0);
    sm_config_set_sideset_pins(&c, PIN_D5);
    sm_config_set_set_pins(&c, PIN_D0, 5);
    sm_config_set_out_shift(&c, true, true, 32);
    sm_config_set_in_shift(&c, false, true, 32);
    sm_config_set_clkdiv(&c, 1);
    pio_sm_set_pindirs_with_mask(pio0, 0, 0, PIN_INPUT_MASK);
    pio_sm_set_pins_with_mask(pio0, 0, 0, PIN_INPUT_MASK);
    pio0->input_sync_bypass = 0xFF000;
    pio_gpio_init(pio0, PIN_D0);
    pio_gpio_init(pio0, PIN_D1);
    pio_gpio_init(pio0, PIN_D2);
    pio_gpio_init(pio0, PIN_D3);
    pio_gpio_init(pio0, PIN_D4);
    pio_gpio_init(pio0, PIN_D5);
    pio_gpio_init(pio0, PIN_D6);
    pio_gpio_init(pio0, PIN_D7);

    pio_sm_init(pio0, 0, sProgramOffset, &c);
    pio_set_irq0_source_enabled(pio0, pis_sm0_rx_fifo_not_empty, true);
    irq_set_exclusive_handler(PIO0_IRQ_0, ntrc_pioIrq);
#ifdef DSPICO_ENABLE_WRFUXXED
    ntrc_initSpiUart(spiUartProgOffs);
#endif

    gpio_set_irq_callback(gpioIrq);
    gpio_set_irq_enabled(PIN_RST, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(PIN_CEB, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    irq_set_enabled(IO_IRQ_BANK0, true);

    irq_init_priorities();
    // The cart command dispatch is THE hard real-time path: the E4 status
    // response must be armed before the console starts clocking the response
    // (~600-800 ns after the first command word). At equal priority a running
    // gpioIrq (CEB edge counting/sampling, with slow APB reads) cannot be
    // preempted by the dispatch - r67/r68 logs show E4 cmd0 dispatched with
    // the SM already at pc=8 (too late) whenever the fall handler ran long,
    // and the DLDI driver bails on the garbage status byte. Dispatch goes to
    // the top; the CEB edge work (diagnostics + failure-path kick) drops to
    // the bottom.
    irq_set_priority(PIO0_IRQ_0, 0x00);
    irq_set_priority(IO_IRQ_BANK0, 0x80);
    irq_set_priority(DMA_IRQ_1, 0x80);
    irq_set_priority(TIMER0_IRQ_0, 0x80);

    // printf("Starting\n");
    // printf("sProgramOffset %d\n", sProgramOffset);
    // printf("Boot time %d\n", (u32)bootTime);
    resetNtrCard();
    sIsSdCardMounted = false;
    initSd();

#if defined(ENABLE_UART_LOG) && !defined(CACHE_SUMMARY_LOG)
    // Print the mount result BEFORE tryRebootToBootsel can halt the main loop,
    // so a f_mount failure is distinguishable from a DLDI E3/E4/E5 failure.
    uartLogPrintf("[sd] f_mount=%s\n", sIsSdCardMounted ? "OK" : "FAIL");
    uartLogFlush();
#endif

    tryRebootToBootsel();

    // Cold-boot SD pre-warm: absorb the card's power-on housekeeping latency
    // here (no host deadline) and cache sector 0, so the loader's first
    // E3/E4 answers ready immediately instead of storming. See
    // ntrc_gameSdPrewarm in ntrCardRomGameSd.cpp.
    ntrc_gameSdPrewarm();


    pwr_initPowerSaving();
#if NTRC_TRACE_ENABLED
    // Non-intrusive diagnostics: a PIO0 listener observes the physical bus and
    // an IRQ-less DMA ring stores the raw command words for core1. Started
    // only after SD bring-up; nothing here touches PIO0 SM0, DMA0, the SDIO
    // state machine or any interrupt configuration.
    ntrCardTraceInit();
#endif

#if CACHE_STAGE >= 1
    {
        // No PSRAM or cache path exists yet at S1, so this only reports that
        // the resource registry found no conflict. It is not a stage PASS.
        const cacheResourcePlan* rp = cacheResourcePlanGet();
        cacheInitLogEmit(CACHE_LOG_EV_INIT_SUMMARY, (u8)CACHE_STAGE, 0,
                         rp->conflict ? 2 : 0, 0, 0, 0, 0, 0, 0, 0);
    }
#endif

#ifdef CACHE_WATCH_LOG
    __dmb();
    sWatchReady = true; // all boot UART output has finished
#endif
    while (1)
    {
#ifdef CACHE_WATCH_ACTIVE_MARKERS
        sWatchLoops++;
#endif
        WATCH_PHASE(1);
#if CACHE_STAGE >= 1
        // Bounded diagnostic pump: at most one record and 16 UART bytes per
        // call, only while the cartridge bus is idle.
        cacheInitLogPoll();
        cacheInitLogStatsStep();
#endif
        WATCH_PHASE(2);
        gSdCard.Update();
        gSdCard.Update();
#if CACHE_STAGE >= 3
        // Bounded cache work per iteration: at most 4 x 32 B PSRAM fragments
        // so a 512 B hit resolves quickly without blocking the main loop. The
        // deferred F6 write start runs first so a pending write owns the SD.
        WATCH_PHASE(3);
        ntrc_gameSdWriteService();
        WATCH_PHASE(4);
        // Bounded cache work per iteration. A step that published a demand
        // (DEMAND_PUBLISHED) explicitly started no background transport, so the
        // loop yields back to the host/IRQ path immediately (design 7.2).
        for (int cacheSteps = 0; cacheSteps < 4; cacheSteps++)
        {
            cacheSdStepResult sr = cacheSdStep();
            if (sr == CACHE_SD_STEP_IDLE || sr == CACHE_SD_STEP_DEMAND_PUBLISHED)
                break;
        }
#if defined(CACHE_STATS_LOG)
        {
            static u64 sLastCacheStats;
            u64 now = time_us_64();
            if (now - sLastCacheStats >= 5000000ull)
            {
                sLastCacheStats = now;
                cacheInitLogEmit(CACHE_LOG_EV_SD_STATS, 0, 0, 0, 0,
                                 cacheSdHits(), cacheSdMisses(), 0,
                                 cacheSdFills(), cacheSdErrors(),
                                 cacheSdSectors());
                // Categorized attribution (design section 9.1): separate SD
                // ownership, cache data and backfill so no single error total
                // is used to judge success.
                const cs_state* cs = cacheSdState();
                cacheSdCounters cnt;
                cacheSdGetCounters(&cnt);
                cacheInitLogEmit(CACHE_LOG_EV_SD_COUNTS, 0, 0, 0, 0,
                                 cnt.sd_token_obsolete, cnt.sd_token_live_conflict,
                                 cnt.sd_transfer_error, cnt.l2_hit_attempt,
                                 cnt.l2_hit_verified, cnt.l2_crc_fail);
                cacheInitLogEmit(CACHE_LOG_EV_FILL_COUNTS, 0, 0, 0, 0,
                                 cnt.psram_read_fail, cnt.psram_fill_fail,
                                 cnt.fill_admit, cnt.fill_drop, cnt.fill_defer,
                                 cnt.fill_commit);
                cacheInitLogEmit(CACHE_LOG_EV_HANDOVER, 0, 0, 0,
                                 cs->c_e5_slot_mismatch,
                                 cs->c_e4_busy, cs->c_e4_ready_queued,
                                 cs->c_e4_ack_failed, cs->c_e5_ok,
                                 cs->c_e5_no_ack, cs->c_e5_identity_mismatch);
                {
                    const cs_fault_record* ff = cacheSdFirstFault();
                    if (ff && ff->valid)
                        cacheInitLogEmit(CACHE_LOG_EV_FAULT_FIRST, ff->kind, 0, 0, 0,
                                         ff->detail, ff->offer_id, ff->acked_offer_id,
                                         ff->sampled_ready, ff->seq, 0);
                }
                cacheInitLogEmit(CACHE_LOG_EV_FIFO, 0, 0, 0, 0,
                                 gCartSdFifoRecovery, gCartSdRecoveryPending, 0, 0, 0, 0);
                cacheInitLogEmit(CACHE_LOG_EV_CACHE_HEALTH, 0, 0, 0, 0,
                                 cacheSdDegraded() ? 1u : 0u,
                                 (u32)cs->resp_state,
                                 cacheSdFillDropReason(),
                                 cnt.fill_quarantine,
                                 (u32)cs->last_invalidate_reason, 0);
                u32 flags = (cs->intent_valid ? 1u : 0u) |
                            (cs->intent_consumed ? 2u : 0u) |
                            (cs->completion_valid ? 4u : 0u) |
                            (cs->binding_valid ? 8u : 0u) |
                            (cs->write_pending ? 16u : 0u) |
                            (cs->intent_faulted ? 32u : 0u);
                cacheInitLogEmit(CACHE_LOG_EV_SD_STATE, 0, 0, 0, 0,
                                 cs->intent.sector, flags,
                                 (u32)gSdCard.DebugState(), cs->c_protocol_fault,
                                 cs->c_sd_retry, cs->c_sd_fault);
            }
        }
#endif
#endif
#if NTRC_TRACE_ENABLED
        // Publish a consistent SD/cache snapshot for core1 to correlate with
        // the decoded command stream. Core0 main loop only, never an IRQ.
        {
            static u32 lastSector = 0xFFFFFFFFu;
            static u32 requestGeneration;
            ntrCardTraceSnapshot snap;
            u32 cache[4];

            ntrCardTraceGetCacheInfo(cache);
            u32 sector = gSdCard.DebugSectorAddress();
            if (sector != lastSector)
            {
                lastSector = sector;
                requestGeneration++;
            }

            snap.sdState = (u32)gSdCard.DebugState();
            snap.sdSequentialState = (u32)gSdCard.DebugSequentialState();
            snap.sdSectorAddress = sector;
            snap.sdSectorsCompleted = gSdCard.GetSectorsCompleted();
            snap.sdSectorCount = gSdCard.GetSectorCount();
            snap.cacheSector0 = cache[0];
            snap.cacheSector1 = cache[1];
            snap.cacheIndex = cache[2];
            snap.requestGeneration = requestGeneration;
            snap.errorCode = cache[3];
            ntrCardTracePublishSnapshot(&snap);
        }
#endif
    #ifdef ENABLE_R4_MODE
        WATCH_PHASE(5);
        ntrc_gameR4Update();
    #endif
        WATCH_PHASE(6);
#if CACHE_STAGE >= 3
        // cacheSdStep() can leave a software-only SD ReadBegin or more PSRAM
        // fragments to process. Neither guarantees a new IRQ, and an E3 can
        // also arrive just before this point. Until an atomic idle/wakeup
        // protocol is implemented, keep servicing stage 3 even with L2 off.
        // This also lets the bounded log pump drain when the host goes idle.
        tight_loop_contents();
#else
        __wfi();
#endif
    }
}
