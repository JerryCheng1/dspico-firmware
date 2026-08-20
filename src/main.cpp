#include "common.h"
#include "hardware/vreg.h"
#include "hardware/clocks.h"
#include "hardware/timer.h"
#include "pico/time.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
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

static u32 sProgramOffset;
FATFS sFatFs;
SdCard gSdCard;
static bool sIsSdCardMounted;

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
    { extern volatile u32 gCartSdE4LenArmed; gCartSdE4LenArmed = 0; }
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

#ifdef ENABLE_UART_LOG
static volatile u32 sCartCebAborts;
static volatile u32 sCartCebFalls;
static volatile u32 sCartCebFallStuck;
static volatile u32 sCebPhantomRise;
static volatile u32 sCebPhantomFall;
static volatile u32 sCartLastAbortUs;
static volatile u32 sCartLastAbortPc;

#ifdef ENABLE_UART_LOG
// --- Crash triage -------------------------------------------------------
// pico-sdk's default fault handlers are `bkpt #0` (isr_hardfault et al.),
// which without a debugger escalates straight to silent LOCKUP - exactly the
// r65 boot-1 signature: log dies mid-character, [stuck] (a lower-priority
// IRQ) can never run, the cart goes deaf and the NDSL freezes on the logo.
// Override the weak symbols and dump the faulting context over the UART.
static void dbgDeferFlushAll(void);
extern "C" void __attribute__((noreturn)) dbgFaultDump(u32 excReturn, u32* frame)
{
    (void)excReturn;
    u32 ipsr;
    __asm volatile("mrs %0, ipsr" : "=r"(ipsr));
    // frame[0..7] = R0,R1,R2,R3,R12,LR,PC,xPSR pushed by the exception entry.
    uartLogPrintfBlocking(
        "\r\n[FAULT] vec=%lu PC=%08lX LR=%08lX XPSR=%08lX\r\n",
        (unsigned long)(ipsr & 0x1FFu), (unsigned long)frame[6],
        (unsigned long)frame[5], (unsigned long)frame[7]);
    uartLogPrintfBlocking(
        "[FAULT] R0=%08lX R1=%08lX R2=%08lX R3=%08lX R12=%08lX\r\n",
        (unsigned long)frame[0], (unsigned long)frame[1],
        (unsigned long)frame[2], (unsigned long)frame[3],
        (unsigned long)frame[4]);
    uartLogPrintfBlocking(
        "[FAULT] CFSR=%08lX HFSR=%08lX SHCSR=%08lX ICSR=%08lX\r\n",
        (unsigned long)scb_hw->cfsr, (unsigned long)scb_hw->hfsr,
        (unsigned long)scb_hw->shcsr, (unsigned long)scb_hw->icsr);
    dbgDeferFlushAll();
    while (true)
        tight_loop_contents();
}

// Each vector entry must itself be naked: a normal wrapper's prologue would
// push registers and move SP away from the exception frame before the dump
// could read it.
#define DBG_FAULT_HANDLER(name)                                        \
    extern "C" void __attribute__((naked)) name(void)                 \
    {                                                                  \
        __asm volatile("mov r0, lr\n"                                  \
                       "mov r1, sp\n"                                  \
                       "b dbgFaultDump\n");                            \
    }

DBG_FAULT_HANDLER(isr_hardfault)
DBG_FAULT_HANDLER(isr_busfault)
DBG_FAULT_HANDLER(isr_usagefault)
DBG_FAULT_HANDLER(isr_memmanage)

static void dbgEnableFaults(void)
{
    // Route MemManage/BusFault/UsageFault to their vectors (and our dump)
    // instead of escalating all of them into HardFault's FORCED path.
    hw_set_bits(&scb_hw->shcsr,
                M33_SHCSR_MEMFAULTENA_BITS | M33_SHCSR_BUSFAULTENA_BITS |
                    M33_SHCSR_USGFAULTENA_BITS);
}
#endif

#ifdef ENABLE_UART_LOG
// --- Deferred diagnostic log --------------------------------------------
// See common.h. The ring is written only from the core0 main loop (LOG call
// sites are all inside SdCard::Update()/this file) and drained only while the
// cartridge bus has been quiet, so nothing in the E3->E4->E5 timing path ever
// waits on the 115200-baud UART again.
#define DBG_DEFER_BITS 13u
#define DBG_DEFER_SIZE (1u << DBG_DEFER_BITS)
#define DBG_DEFER_MASK (DBG_DEFER_SIZE - 1u)
static char sDeferBuf[DBG_DEFER_SIZE];
static u32 sDeferW;
static u32 sDeferR;
static volatile u32 sDeferDropped;
static volatile u32 sCartLastBusUs;

// Dispatcher ground truth (written by ntrc_pioIrq in ntrCardIrq.S): every RX
// word read from the cart SM FIFO plus the SM's absolute PC at that instant.
extern "C" {
u32 gRxTrace[64];
u8 gRxTracePc[64];
u32 gRxTraceIdx;
}

void dbgDeferLog(const char* fmt, ...)
{
    char buf[160];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n <= 0)
        return;
    if ((u32)n > DBG_DEFER_SIZE - 1)
        n = DBG_DEFER_SIZE - 1;
    if (DBG_DEFER_SIZE - (sDeferW - sDeferR) < (u32)n + 1)
    {
        sDeferDropped++;
        return;
    }
    for (int i = 0; i < n; i++)
        sDeferBuf[(sDeferW + (u32)i) & DBG_DEFER_MASK] = buf[i];
    __asm volatile("" ::: "memory");
    sDeferW += (u32)n;
}

// Drains at most ONE line per call, and only when no command has started for
// >1 ms and CEB is idle-high: a line that begins in a quiet window can still
// block ~2.5 ms, so never start one while the loader might be mid-burst.
static void dbgDeferDrain(void)
{
    if (sDeferW == sDeferR)
        return;
    if ((u32)(time_us_32() - sCartLastBusUs) < 1000u)
        return;
    if ((gpio_get(PIN_CEB) & 1u) == 0)
        return;
    char line[192];
    u32 len = 0;
    while (sDeferR + len < sDeferW && len < sizeof(line) - 1)
    {
        char c = sDeferBuf[(sDeferR + len) & DBG_DEFER_MASK];
        line[len++] = c;
        if (c == '\n')
            break;
    }
    line[len] = 0;
    sDeferR += len;
    uartLogPrintfBlocking("%s", line);
}

// Emergency path (fault dump / main-loop-stuck probe): flush everything now,
// blocking. Only called when the session is already lost.
static void dbgDeferFlushAll(void)
{
    while (sDeferW != sDeferR)
    {
        char line[192];
        u32 len = 0;
        while (sDeferR + len < sDeferW && len < sizeof(line) - 1)
        {
            char c = sDeferBuf[(sDeferR + len) & DBG_DEFER_MASK];
            line[len++] = c;
            if (c == '\n')
                break;
        }
        line[len] = 0;
        sDeferR += len;
        uartLogPrintfBlocking("%s", line);
    }
}
#endif

// Periodic wake for the main loop on TIMER0 alarm 1. NOT the SDK alarm pool:
// SdCard installs its own exclusive handler on TIMER0_IRQ_0 (alarm 0, the
// sequential-read timeout), which silently kills the default alarm pool's
// repeating timers - that is why the r59/r60 heartbeats still stopped once
// the bus went quiet. Alarm 1 / TIMER0_IRQ_1 is independent of both.
static volatile u32 sWakeTicks;
static volatile u32 sHbAtWake;
static void __time_critical_func(dbgWakeIrq)(void)
{
    hw_clear_bits(&timer0_hw->intr, TIMER_INTR_ALARM_1_BITS);
    timer0_hw->alarm[1] = timer0_hw->timerawl + 100000u; // re-arm +100 ms
    sWakeTicks++;
    // Main-loop-stuck detector. r63 proved the main loop can die inside
    // SdCard::Update() after E3 (heartbeats stop, the queued "read#1 begin"
    // only surfaced at power-off). The heartbeat section refreshes sHbAtWake;
    // 5 wakes (500 ms) without a refresh means the main loop is wedged, and
    // this IRQ is then the only thing still able to report. Priority 0x00 -
    // ABOVE the cart IRQ (0x40) - so it preempts even a spinning PIO0/IO_BANK
    // handler; ICSR then names the exception that is hogging the core.
    if (sWakeTicks - sHbAtWake >= 5 && (sWakeTicks - sHbAtWake) % 5 == 0)
    {
        uartLogPrintfBlocking(
            "[stuck] t=%lums st=%d seq=%d sec=%08lX done=%lu PC=%u TXF=%u RXF=%u ICSR=%08lX\n",
            (unsigned long)millis(),
            gSdCard.DebugState(), gSdCard.DebugSequentialState(),
            (unsigned long)gSdCard.DebugSectorAddress(),
            (unsigned long)gSdCard.GetSectorsCompleted(),
            (unsigned)(pio_sm_get_pc(pio0, 0) - (u8)sProgramOffset),
            (unsigned)pio_sm_get_tx_fifo_level(pio0, 0),
            (unsigned)pio_sm_get_rx_fifo_level(pio0, 0),
            (unsigned long)scb_hw->icsr);
        // The main loop is wedged, so nothing else will ever drain the
        // deferred log - flush it here to recover the r63/r64 property that
        // the last printed line is the last step that actually ran.
        dbgDeferFlushAll();
    }
}

static void dbgWakeTimerInit(void)
{
    hardware_alarm_claim(1);
    irq_set_exclusive_handler(TIMER0_IRQ_1, dbgWakeIrq);
    irq_set_priority(TIMER0_IRQ_1, 0x00); // above the cart IRQ (0x40): preempts a spinning handler
    hw_set_bits(&timer0_hw->inte, TIMER_INTE_ALARM_1_BITS);
    irq_set_enabled(TIMER0_IRQ_1, true);
    timer0_hw->alarm[1] = timer0_hw->timerawl + 100000u;
}
#endif

static void __time_critical_func(gpioIrq)(uint gpio, u32 events)
{
#ifdef ENABLE_PREVENT_DSI_AUTOBOOT
    u64 time = time_us_64();
#endif
    if (gpio == PIN_CEB)
    {
#ifdef ENABLE_UART_LOG
        if (events & GPIO_IRQ_EDGE_FALL)
        {
            // Command-start counting + bus-quiet timestamp for the deferred
            // log drain. No PC sampling here any more: this handler now runs
            // at the LOWEST priority, so the sample lands after the dispatch
            // and reads the armed write path (12..23) on perfectly healthy
            // commands - r69's CFS=67 was that artifact, not real desyncs.
            sCartCebFalls++;
            sCartLastBusUs = time_us_32();
            if (gpio_get(PIN_CEB) & 1u)
                sCebPhantomFall++; // fall event but line high = line glitch
            return;
        }
#endif
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
#ifdef ENABLE_UART_LOG
            sCebPhantomRise++;
#endif
            return;
        }
        if (relPc >= 3 && relPc <= 23)
        {
            // The dispatch IRQ (now top priority) preempts this handler, so a
            // command arriving mid-kick would get its response pushed into a
            // SM/FIFO state that is about to be wiped. Mask it for the kick;
            // the level IRQ re-pends afterwards and the FSTAT guard drops
            // the (cleared) stale entry.
            irq_set_enabled(PIO0_IRQ_0, false);
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
            // own length push and stall the SM at the autopull.
            { extern volatile u32 gCartSdE4LenArmed; gCartSdE4LenArmed = 0; }
            // Ring-visible kick record: a KICK right after E4_READY means the
            // ready=1 response left the SM unparked at CEB rise despite the
            // pre-arm (and the kick can erase a following E5 word).
            // arg = (last dispatched cmd0 top byte)<<8 | relPc: names WHICH
            // transaction ended mid-write-loop (E4 poll vs E5 payload).
            SD_EVT(12, ((gNtrRomEmu.cmd0 >> 24) << 8) | (relPc & 0xFFu));
            irq_clear(PIO0_IRQ_0);
            irq_set_enabled(PIO0_IRQ_0, true);
#ifdef ENABLE_UART_LOG
            sCartCebAborts++;
            sCartLastAbortUs = time_us_32();
            sCartLastAbortPc = relPc;
#endif
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
    scb_hw->scr |= ARM_CPU_PREFIXED(SCR_SLEEPDEEP_BITS);
    while (!gComputeScrambler)
    {
        gScramblerRingWPtr = gScramblerRing;
        __wfe();
    }
    while (1)
    {
        u32* wPtr = gScramblerRingWPtr;
        u32* next = SCR_RING_WRAP(wPtr + 1);
        if (next == gNtrRomEmu.scrRingRPtr)
        {
            __wfe();
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
#ifdef ENABLE_UART_LOG
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
    // Set all GPIOs to inputs.
    // Note that we rely on hardware reset having enabled pull-downs.
    gpio_init_mask(0xFFFFFFFFu);

#ifdef ENABLE_UART_LOG
    // GPIO0 = logic-analyzer trigger output (raised when the first ready=1 E4
    // answer is queued, dropped when E5 fires). On this no-PSRAM baseline the
    // pin only reaches the unused PSRAM CLK pad, so driving it is harmless.
    gpio_init(0);
    gpio_put(0, false);
    gpio_set_dir(0, GPIO_OUT);
#endif

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

#ifdef ENABLE_UART_LOG
// Boundary trace: every main-loop iteration records the cart SM PC (relative
// to the ntr_card program start), the CEB line level and the E4/E5 counters.
// The ring freezes 48 iterations after the first ready=1 answer and dumps, so
// the log shows the exact E4->E5 boundary: whether the loader ever started an
// E5 transaction (CEB fall) and where the SM was at that moment.
#define CART_TRACE_BITS 8u
#define CART_TRACE_SIZE (1u << CART_TRACE_BITS)
static u32 sCartTrace[CART_TRACE_SIZE];
static u32 sCartTraceIdx;
static bool sCartTraceDumped;

static void cartTraceDump(void)
{
    // Counter snapshot FIRST, printed directly (not via the deferred ring) so
    // TRACE_QUIET_LOG builds keep it: this is the hb line's content.
    {
        extern volatile u32 gCartSdE3Requests, gCartSdE4Polls;
        extern volatile u32 gCartSdE4Ready, gCartSdE5Reads, gCartSdE5Unready;
        extern volatile u32 gCartSdDummyCmd0, gCartSdUnknownCmd1;
        extern volatile u32 gCartSdLastE4Us, gSdAllDoneUs;
        uartLogPrintfBlocking("[cnt] E3=%lu E4=%lu R=%lu E5=%lu EU=%lu D=%lu U=%lu A=%lu lastE4=%lu allDone=%lu abort@%lu abortPc=%lu\n",
            (unsigned long)gCartSdE3Requests, (unsigned long)gCartSdE4Polls,
            (unsigned long)gCartSdE4Ready, (unsigned long)gCartSdE5Reads,
            (unsigned long)gCartSdE5Unready,
            (unsigned long)gCartSdDummyCmd0, (unsigned long)gCartSdUnknownCmd1,
            (unsigned long)sCartCebAborts,
            (unsigned long)gCartSdLastE4Us, (unsigned long)gSdAllDoneUs,
            (unsigned long)sCartLastAbortUs, (unsigned long)sCartLastAbortPc);
    }
    u32 n = sCartTraceIdx < CART_TRACE_SIZE ? sCartTraceIdx : CART_TRACE_SIZE;
    u32 start = sCartTraceIdx - n;
    uartLogPrintfBlocking("[trace] E4->E5 boundary dump (%lu entries, oldest first):\n",
        (unsigned long)n);
    for (u32 i = 0; i < n; i++)
    {
        u32 e = sCartTrace[(start + i) & (CART_TRACE_SIZE - 1)];
        uartLogPrintfBlocking("[trace] pc=%u ceb=%u wreb=%u E4=%lu E5=%lu\n",
            (unsigned)(e & 0x1F), (unsigned)((e >> 5) & 1), (unsigned)((e >> 6) & 1),
            (unsigned long)((e >> 7) & 0xFFF),
            (unsigned long)((e >> 19) & 0xFFF));
    }
    // Dispatcher ground truth: what words actually came off the wire and
    // where the SM was when each was read. rel PC: 0-3 idle/arm, 4-7 cmd_loop
    // (healthy dispatch), 8+ means the SM already left cmd_loop - the
    // response push will be late.
    {
        u32 idx = gRxTraceIdx;
        u32 m = idx < 64 ? idx : 64;
        u32 rstart = idx - m;
        uartLogPrintfBlocking("[rx] %lu words, idx=%lu, oldest first:\n",
            (unsigned long)m, (unsigned long)idx);
        for (u32 i = 0; i < m; i++)
        {
            u32 j = (rstart + i) & 63u;
            uartLogPrintfBlocking("[rx] w=%08lX pc=%d\n",
                (unsigned long)gRxTrace[j],
                (int)((int)gRxTracePc[j] - (int)(u8)sProgramOffset));
        }
    }
    // E4 pre-arm ground truth: TX FIFO level (0-4) and the pre-arm flag (A)
    // sampled at each E4 cmd0 dispatch. Healthy mid-storm is "0A": the SM's
    // end-of-response autopull already sucked the pre-armed length into the
    // OSR, so the FIFO reads empty; storm-entry polls read "0-".
    {
        extern volatile u32 gE4FlIdx;
        extern u8 gE4FlRing[16];
        u32 idx = gE4FlIdx;
        u32 m = idx < 16 ? idx : 16;
        u32 rstart = idx - m;
        uartLogPrintfBlocking("[e4fl] %lu polls, idx=%lu, oldest first (level,flag):\n",
            (unsigned long)m, (unsigned long)idx);
        for (u32 i = 0; i < m; i++)
        {
            u8 e = gE4FlRing[(rstart + i) & 15u];
            uartLogPrintfBlocking("[e4fl] %u%c\n",
                (unsigned)(e & 0xFu), (e & 0x10u) ? 'A' : '-');
        }
    }
    // SD pipeline ground truth (codes in common.h): where the read actually
    // stopped - BEGIN with no CMD18, CMD18 with no ALLDONE, ALLDONE with no
    // E4_READY, etc. One line per event, code + raw arg.
    {
        extern volatile u32 gSdEvtIdx;
        extern u32 gSdEvtRing[64];
        static const char* const sNames[] = {
            "?", "RD_BEGIN", "RD_RXCONT", "CMD18_OK", "CMD18_FAIL",
            "BLOCK_OK", "CRC_FAIL", "TIMEOUT", "ALLDONE", "E3_REQ",
            "E4_READY", "E4_READY_CACHED", "KICK", "E5_FETCH"
        };
        u32 idx = gSdEvtIdx;
        u32 m = idx < 64 ? idx : 64;
        u32 rstart = idx - m;
        uartLogPrintfBlocking("[sdevt] %lu events, idx=%lu, oldest first:\n",
            (unsigned long)m, (unsigned long)idx);
        for (u32 i = 0; i < m; i++)
        {
            u32 e = gSdEvtRing[(rstart + i) & 63u];
            u32 code = e >> 28;
            const char* name =
                code < sizeof(sNames) / sizeof(sNames[0]) ? sNames[code] : "?";
            if (code == 12 || code == 13)
                // KICK: (cmd0 top byte)<<8|relPc; E5_FETCH: raw word (top
                // nibble masked by the code field - legit is x5000000).
                uartLogPrintfBlocking("[sdevt] %s 0x%07lX\n", name,
                    (unsigned long)(e & 0x0FFFFFFFu));
            else
                uartLogPrintfBlocking("[sdevt] %s %lu\n", name,
                    (unsigned long)(e & 0x0FFFFFFFu));
        }
    }
}
#endif

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
    dbgEnableFaults();
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

#ifdef ENABLE_UART_LOG
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

#ifdef ENABLE_UART_LOG
    dbgWakeTimerInit();
    // Loader-time read tracing: [sdio] read#N lines show CMD18 / block / retry
    // progress of the first 8 cartridge reads. Without this the SD side of the
    // E3->E4->E5 flow is a blind spot - r62 showed E4=861 R=0 (the loader
    // polling a read that never completes) with zero visibility into why.
    gSdCard.EnableRuntimeDiagnostics();
#endif


    while (1)
    {
        gSdCard.Update();
        gSdCard.Update();
#ifdef ENABLE_UART_LOG
        {
            extern volatile u32 gCartSdE4Polls, gCartSdE5Reads, gCartSdE4Ready;
            if (!sCartTraceDumped)
            {
                sCartTrace[sCartTraceIdx++ & (CART_TRACE_SIZE - 1)] =
                    ((u32)(pio_sm_get_pc(pio0, 0) - (u8)sProgramOffset) & 0x1Fu) |
                    ((u32)(gpio_get(PIN_CEB) & 1) << 5) |
                    ((u32)(gpio_get(PIN_WREB) & 1) << 6) |
                    ((gCartSdE4Polls & 0xFFFu) << 7) |
                    ((gCartSdE5Reads & 0xFFFu) << 19);
            }
        }
        {
            // Periodic heartbeat, unconditional: proves the main loop is alive
            // and shows every counter even if nothing changes. Previously the
            // print only fired on E3/E5 transitions, so a halted main loop
            // (f_mount FAIL) looked identical to "no DLDI activity".
            // All of these go through the deferred ring: a blocking print
            // here while the loader is mid-poll delays the E3->ready=1 answer
            // past the DLDI read deadline (see common.h).
            static u32 lastHbMs;
            static u32 readySeenMs;
            extern volatile u32 gCartSdE3Requests, gCartSdE4Polls;
            extern volatile u32 gCartSdE4Ready, gCartSdE5Reads;
            extern volatile u32 gCartSdDummyCmd0, gCartSdUnknownCmd1;
            extern volatile u32 gCartSdUnknownWord, gCartSdUnknownCmd0;
            extern volatile u32 gCartSdLastE4Us, gSdAllDoneUs;
            u32 now = millis();
            sHbAtWake = sWakeTicks; // feed the main-loop-stuck detector
            // Edge-triggered: print the moment the loader starts the SD
            // sequence, so a mount attempt shorter than the 500 ms heartbeat
            // interval can no longer pass unrecorded.
            static u32 lastE3Seen;
            if (gCartSdE3Requests != lastE3Seen)
            {
                lastE3Seen = gCartSdE3Requests;
                LOG("[sd] E3 #%lu t=%lums E4=%lu R=%lu E5=%lu A=%lu\n",
                    (unsigned long)gCartSdE3Requests, (unsigned long)now,
                    (unsigned long)gCartSdE4Polls, (unsigned long)gCartSdE4Ready,
                    (unsigned long)gCartSdE5Reads, (unsigned long)sCartCebAborts);
            }
            // Raw words of the last "unknown game command": shows WHAT the
            // desynced dispatcher actually read (real command vs. garbage).
            static u32 lastUnknownSeen;
            if (gCartSdUnknownCmd1 != lastUnknownSeen)
            {
                lastUnknownSeen = gCartSdUnknownCmd1;
                LOG("[cart] unknown cmd1 #%lu w=%08lX c0=%08lX\n",
                    (unsigned long)gCartSdUnknownCmd1,
                    (unsigned long)gCartSdUnknownWord,
                    (unsigned long)gCartSdUnknownCmd0);
            }
            // E5 payload evidence: the sector served, its first/last bytes,
            // and for sector 0 the MBR signature. If our buffer is valid but
            // the console still rejects the mount, the corruption is on the
            // wire (E5 stream), not in the SD read.
            extern volatile u32 gE5TraceCount;
            extern u32 gE5TraceSector[8], gE5TraceHead[8][2], gE5TraceTail[8][2];
            static u32 lastE5TraceSeen;
            if (gE5TraceCount != lastE5TraceSeen)
            {
                lastE5TraceSeen = gE5TraceCount;
                u32 t = (gE5TraceCount - 1) & 7u;
                LOG("[e5] #%lu sec=%lu head=%08lX %08lX tail=%08lX %08lX%s\n",
                    (unsigned long)gE5TraceCount, (unsigned long)gE5TraceSector[t],
                    (unsigned long)gE5TraceHead[t][0], (unsigned long)gE5TraceHead[t][1],
                    (unsigned long)gE5TraceTail[t][0], (unsigned long)gE5TraceTail[t][1],
                    (gE5TraceSector[t] == 0 &&
                     ((gE5TraceTail[t][1] >> 16) & 0xFFFFu) == 0xAA55u) ? " MBR-SIG-OK" : "");
            }
            if ((s32)(now - lastHbMs) >= 500) {
                lastHbMs = now;
                // SM PC tells us where the PIO state machine is: if the loader
                // sent E5 but the SM never framed it, PC freezes at the same
                // instruction across heartbeats (e.g. stuck in the E4 response
                // write_loop, or waiting on CEB) instead of cycling cmd_loop.
                u8 smPc = pio_sm_get_pc(pio0, 0);
                // Relative PC from ntr_card program start. 0=start/CEB-idle,
                // 4-7=cmd_loop (reading cmd bytes), 8-11=length/dir autopull,
                // 12-16=read_loop, 17-23=write path (17-18 setup, 19-23
                // write_loop). A frozen PC across heartbeats = SM stuck there.
                u8 relPc = smPc - (u8)sProgramOffset;
                // SD state machine snapshot: st 0=Uninit 1=Idle 2=ReadBegin
                // 3=ReadBusy 4=ReadWriteCancel 5=WriteBegin 6=WriteBusy;
                // seq 0=None 1=Read 2=Write. With E4 polling and R=0 this
                // pins down WHERE the read stalls instead of only that it
                // never completes.
                LOG("[sd] hb E3=%lu E4=%lu R=%lu E5=%lu D=%lu U=%lu A=%lu PC=%u TXF=%u RXF=%u st=%d seq=%d sec=%08lX done=%lu CF=%lu phR=%lu phF=%lu lastE4=%lu allDone=%lu abort@%lu pc=%u drop=%lu\n",
                    (unsigned long)gCartSdE3Requests, (unsigned long)gCartSdE4Polls,
                    (unsigned long)gCartSdE4Ready, (unsigned long)gCartSdE5Reads,
                    (unsigned long)gCartSdDummyCmd0, (unsigned long)gCartSdUnknownCmd1,
                    (unsigned long)sCartCebAborts, (unsigned)relPc,
                    (unsigned)pio_sm_get_tx_fifo_level(pio0, 0),
                    (unsigned)pio_sm_get_rx_fifo_level(pio0, 0),
                    gSdCard.DebugState(), gSdCard.DebugSequentialState(),
                    (unsigned long)gSdCard.DebugSectorAddress(),
                    (unsigned long)gSdCard.GetSectorsCompleted(),
                    (unsigned long)sCartCebFalls, (unsigned long)sCebPhantomRise,
                    (unsigned long)sCebPhantomFall,
                    (unsigned long)gCartSdLastE4Us, (unsigned long)gSdAllDoneUs,
                    (unsigned long)sCartLastAbortUs, (unsigned)sCartLastAbortPc,
                    (unsigned long)sDeferDropped);

                // Boundary dump trigger: the loader started the SD sequence
                // (E3>0) but ready=1 was never served within 400 ms. Fires
                // regardless of the E5 count - r61 proved the loader can send
                // E5 (here E5=1) after a timeout WITHOUT ever seeing ready,
                // and the old `E5==0` requirement suppressed the dump exactly
                // then. Time-based because the main loop sleeps in __wfi once
                // the bus goes silent (a 100 ms wake timer keeps this alive).
                if (!sCartTraceDumped && gCartSdE3Requests > 0 && gCartSdE4Ready == 0)
                {
                    if (readySeenMs == 0)
                        readySeenMs = now;
                    else if ((u32)(now - readySeenMs) > 400)
                    {
                        sCartTraceDumped = true;
                        cartTraceDump();
                    }
                }

                // Early-abort signature (r65 boot 2): the loader gave up
                // BEFORE ever issuing E3 - only a handful of cart commands
                // (CEB falls) ever happened and then the bus went silent and
                // the NDSL reports a mount failure. Dump the trace so the SM
                // state around those few commands is visible.
                if (!sCartTraceDumped && gCartSdE3Requests == 0)
                {
                    static u32 lastCfSeen;
                    static u32 cfIdleMs;
                    if (sCartCebFalls != lastCfSeen)
                    {
                        lastCfSeen = sCartCebFalls;
                        cfIdleMs = now;
                    }
                    else if (sCartCebFalls > 0 && sCartCebFalls < 64 &&
                             cfIdleMs != 0 && (u32)(now - cfIdleMs) > 400)
                    {
                        sCartTraceDumped = true;
                        uartLogPrintfBlocking("[trace] early-abort dump CF=%lu\n",
                            (unsigned long)sCartCebFalls);
                        cartTraceDump();
                    }
                }

                // Quiet dump: the console went silent after real DLDI traffic
                // (r69: full E3 -> 175 polls -> ready=1 -> E5 -> silence, then
                // "Failed to mount"). Neither the E3==0 nor the R==0 trigger
                // covers that path. Re-arms whenever traffic resumes so a
                // later failure can still dump.
                {
                    static u32 lastCfQ;
                    static u32 cfQuietMs;
                    static bool quietDumped;
                    if (sCartCebFalls != lastCfQ)
                    {
                        lastCfQ = sCartCebFalls;
                        cfQuietMs = now;
                        quietDumped = false;
                    }
                    else if (!quietDumped && sCartCebFalls > 64 &&
                             (u32)(now - cfQuietMs) > 400)
                    {
                        quietDumped = true;
                        uartLogPrintfBlocking("[trace] quiet dump CF=%lu\n",
                            (unsigned long)sCartCebFalls);
                        cartTraceDump();
                    }
                }
            }
            (void)now;
        }
#endif
    #ifdef ENABLE_R4_MODE
        ntrc_gameR4Update();
    #endif
#ifdef ENABLE_UART_LOG
        // Drain deferred diagnostics while the cart bus is quiet; sleep only
        // when there is nothing left to print (a non-empty backlog keeps the
        // loop spinning so the next read's completion is still detected
        // within microseconds).
        dbgDeferDrain();
        if (sDeferW == sDeferR)
            __wfi();
#else
        __wfi();
#endif
    }
}
