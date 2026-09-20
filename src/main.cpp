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
    scb_hw->scr |= ARM_CPU_PREFIXED(SCR_SLEEPDEEP_BITS);
    while (!gComputeScrambler)
    {
        gScramblerRingWPtr = gScramblerRing;
#if NTRC_TRACE_ENABLED
        // Cartridge tracing is a side channel served by core1 while the
        // scrambler is idle (unscrambled game / DLDI). It never touches the
        // service SM, DMA0 or the SDIO state machine.
        ntrCardTraceCore1Poll();
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
#if NTRC_TRACE_ENABLED
    // Non-intrusive diagnostics: a PIO0 listener observes the physical bus and
    // an IRQ-less DMA ring stores the raw command words for core1. Started
    // only after SD bring-up; nothing here touches PIO0 SM0, DMA0, the SDIO
    // state machine or any interrupt configuration.
    ntrCardTraceInit();
#endif

    while (1)
    {
        gSdCard.Update();
        gSdCard.Update();
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
        ntrc_gameR4Update();
    #endif
        __wfi();
    }
}
