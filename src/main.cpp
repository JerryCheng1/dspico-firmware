#include "common.h"
#include <string.h>
#include <stdio.h>
#include "hardware/gpio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
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
#include "hardware/xosc.h"
#include "powerSaving.h"
#include "pioUtil.h"
#ifdef ENABLE_PSRAM_CACHE
#include "psram.h"
#include "romCache.h"
#endif

static u32 sProgramOffset;
FATFS sFatFs;
SdCard gSdCard;
static bool sIsSdCardMounted;
volatile u32 gNtrBusCmdCount;

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
#ifdef ENABLE_R4_MODE
    ntrc_resetR4();
#endif
    dma_channel_abort(0);
    // Use an atomic CTRL alias so reset handling cannot restore a stale enable
    // state for another PIO0 machine (for example the optional SPI adapter).
    dspicoPioSmSetEnabled(pio0, 0, false);
    pio_sm_set_pindirs_with_mask(pio0, 0, 0, PIN_INPUT_MASK);
    pio_sm_clear_fifos(pio0, 0);
    pio_sm_restart(pio0, 0);
    pio_sm_clkdiv_restart(pio0, 0);
    irq_clear(PIO0_IRQ_0);
    irq_set_enabled(PIO0_IRQ_0, true);
    pio_sm_exec(pio0, 0, pio_encode_jmp(sProgramOffset));
    dspicoPioSmSetEnabled(pio0, 0, true);
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
    if (gpio == PIN_RST)
    {
        if (events & GPIO_IRQ_EDGE_FALL)
        {
            dspicoPioSmSetEnabled(pio0, 0, false);
            pio_sm_set_pindirs_with_mask(pio0, 0, 0, PIN_INPUT_MASK);
        }
        if (events & GPIO_IRQ_EDGE_RISE)
        {
            resetNtrCard();
        #ifdef ENABLE_PREVENT_DSI_AUTOBOOT
            u32 resetTime = time - sResetStart;
            if (resetTime > 700000)
            {
                dspicoPioSmSetEnabled(pio0, 0, false);
            }
            sResetStart = time;
        #endif
        }
    }        
}

void __scratch_x("cpu1") core1_entry(void)
{
    irq_set_mask_enabled(~0u, false);
    scb_hw->scr |= M0PLUS_SCR_SLEEPDEEP_BITS;
#ifdef ENABLE_PSRAM_CACHE
    // Before game mode needs the scrambler, run the PSRAM probe + full-chip
    // test here on core1. The same core also services every runtime PSRAM
    // request (PIO or bit-bang), while core0 remains available for the
    // cartridge IRQ. Once gComputeScrambler goes true, keep its ring full and
    // use only the otherwise-idle slots for individual PSRAM bus bursts.
    while (!gComputeScrambler)
    {
        gScramblerRingWPtr = gScramblerRing;
        if (!psram_core1_service() && !romCacheCore1Poll())
        {
            // probe/test done (or terminal): park until game mode.
            __wfe();
        }
    }
    while (1)
    {
        u32* wPtr = gScramblerRingWPtr;
        u32* next = SCR_RING_WRAP(wPtr + 1);
        if (next == gNtrRomEmu.scrRingRPtr)
        {
            // The ring is full. Spend one idle slot on one short PSRAM burst,
            // then re-check/refill the scrambler before the following burst.
            // This prevents a 512-byte cache hit from starving the encrypted
            // cartridge data stream for roughly 166 us.
            if (psram_core1_service())
                continue;

            romCacheCore1Poll();
            __wfe();
            continue;
        }

        *wPtr = scr_getNext32(&gScramblerState);
        gScramblerRingWPtr = next;
    }
#else
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
#endif
}

static void initSd(void)
{
    memset(&sFatFs, 0, sizeof(sFatFs));

    //try mounting 16 times
    bool ok = false;
    for (int i = 0; i < 16; i++)
    {
        FRESULT mountResult = f_mount(&sFatFs, "0:", 1);
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
        sIsSdCardMounted = false;
    }
}

static void tryRebootToBootsel(void)
{
    if (!sIsSdCardMounted)
    {
        // No USB hardware on this board, so BOOTSEL (USB mass-storage) flashing
        // is not possible. Halt so the failure is observable on the log UART
        // instead of silently rebooting into a useless BOOTSEL mode.
        LOG("[BOOT] no SD card mounted, halting (no USB to BOOTSEL-flash)\n");
        while (1)
        {
        #ifdef ENABLE_UART_LOG
            if (!uartLogDrain())
                __wfe();
        #else
            __wfi();
        #endif
        }
    }
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

    // set_sys_clock_khz(/*125000*/200000, true);
    // 200 MHz = 1200 MHz / 6 / 1
    set_sys_clock_pll(1200000000, 6, 1);

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

#if defined(ENABLE_PSRAM_CACHE) && PSRAM_FORCE_BITBANG
    // Finish every startup bit-bang burst before the cartridge PIO is enabled.
    // Previously the probe began after initSd(), while NDSL could already be
    // issuing its first mount commands on PIO0. A failed trace with no E3/SDIO
    // read proved that even the probe-only activity could overlap and disrupt
    // that handshake. The selected GPIO22-26/29 backend is independent of
    // SDIO, so it can be qualified here while the cartridge interface is still
    // inactive. Runtime cache traffic remains gated for the later warmup.
    romCacheInit();
    romCacheSdInit();
    while (!romCacheQualificationFinished())
        __wfe();
#endif

    gpio_init_mask(PIN_INPUT_MASK);
    gpio_set_dir_in_masked(PIN_INPUT_MASK);
    gpio_init(PIN_IRQ);
    gpio_put(PIN_IRQ, 0);
    gpio_set_dir(PIN_IRQ, GPIO_OUT);
    gpio_disable_pulls(PIN_D0);
    gpio_disable_pulls(PIN_D1);
    gpio_disable_pulls(PIN_D2);
    gpio_disable_pulls(PIN_D3);
    gpio_disable_pulls(PIN_D4);
    gpio_disable_pulls(PIN_D5);
    gpio_disable_pulls(PIN_D6);
    gpio_disable_pulls(PIN_D7);
    gpio_set_slew_rate(PIN_D0, GPIO_SLEW_RATE_FAST);
    gpio_set_slew_rate(PIN_D1, GPIO_SLEW_RATE_FAST);
    gpio_set_slew_rate(PIN_D2, GPIO_SLEW_RATE_FAST);
    gpio_set_slew_rate(PIN_D3, GPIO_SLEW_RATE_FAST);
    gpio_set_slew_rate(PIN_D4, GPIO_SLEW_RATE_FAST);
    gpio_set_slew_rate(PIN_D5, GPIO_SLEW_RATE_FAST);
    gpio_set_slew_rate(PIN_D6, GPIO_SLEW_RATE_FAST);
    gpio_set_slew_rate(PIN_D7, GPIO_SLEW_RATE_FAST);
    gpio_pull_up(PIN_CEB);
    gpio_pull_up(PIN_WREB);
    gpio_pull_down(PIN_RST);
    gpio_pull_up(PIN_CS2);
    gpio_set_drive_strength(PIN_D0, GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_drive_strength(PIN_D1, GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_drive_strength(PIN_D2, GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_drive_strength(PIN_D3, GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_drive_strength(PIN_D4, GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_drive_strength(PIN_D5, GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_drive_strength(PIN_D6, GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_drive_strength(PIN_D7, GPIO_DRIVE_STRENGTH_2MA);

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

    dspicoPioSmInit(pio0, 0, sProgramOffset, &c);
    pio_set_irq0_source_enabled(pio0, pis_sm0_rx_fifo_not_empty, true);
    irq_set_exclusive_handler(PIO0_IRQ_0, ntrc_pioIrq);
#ifdef DSPICO_ENABLE_WRFUXXED
    ntrc_initSpiUart(spiUartProgOffs);
#endif

    gpio_set_irq_callback(gpioIrq);
    gpio_set_irq_enabled(PIN_RST, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    irq_set_enabled(IO_IRQ_BANK0, true);

    irq_init_priorities();
    irq_set_priority(PIO0_IRQ_0, 0x40);
    irq_set_priority(IO_IRQ_BANK0, 0x40);
    irq_set_priority(DMA_IRQ_1, 0x80);
    irq_set_priority(TIMER_IRQ_0, 0x80);

    // printf("Starting\n");
    // printf("sProgramOffset %d\n", sProgramOffset);
    // printf("Boot time %d\n", (u32)bootTime);

#ifdef ENABLE_UART_LOG
    // Debug UART on GPIO0/1, which the init above does not touch.
    uartLogInit();
#endif

    // Boot banner: the very first thing on the wire. If this does not show
    // up, the problem is UART wiring/baud, not the firmware.
    LOG("\n[BOOT] DSpico firmware up, sysclk=%lu MHz, UART async log OK\n",
        (unsigned long)(clock_get_hz(clk_sys) / 1000000));
    LOG("[BOOT] PSRAM SD cache: %s\n",
    #ifdef ENABLE_PSRAM_CACHE
        "compiled in"
    #else
        "not compiled"
    #endif
    );
#ifdef ENABLE_PSRAM_CACHE
    LOG("[BOOT] PSRAM backend: %s; E3/E5 cache: %s\n",
    #if PSRAM_FORCE_BITBANG
        "bit-bang/core1 burst-interleaved",
    #else
        "PIO1 SM2/core1 burst-interleaved",
    #endif
    #if PSRAM_CACHE_ENABLE_ON_PROBE
        "enabled after probe"
    #else
        "disabled (probe-only diagnostic)"
    #endif
    );
    #if PSRAM_FORCE_BITBANG
    LOG("[BOOT] PSRAM startup qualification: completed before cartridge PIO0\n");
    #endif
#endif

    resetNtrCard();
    sIsSdCardMounted = false;
    initSd();
    LOG("[BOOT] SD card: %s\n", sIsSdCardMounted ? "mounted" : "not mounted");
    gSdCard.EnableRuntimeDiagnostics();

    tryRebootToBootsel();

#if defined(ENABLE_PSRAM_CACHE) && !PSRAM_FORCE_BITBANG
    // SDIO initialization clears and reloads PIO1 instruction memory, so the
    // PIO1 SM2 PSRAM program must be installed only after the physical SD card
    // has completed its final high-speed initialization. Core1 qualifies the
    // PSRAM asynchronously; until it succeeds, all cache lookups safely miss.
    // The cartridge remains on the independent PIO0 block throughout.
    romCacheInit();
    romCacheSdInit();
#endif

#ifdef ENABLE_UART_LOG
    // Log build: skip pwr_initPowerSaving() so clk_peri (UART baud clock) and
    // the timer stay on - the deep-sleep config freezes both, hiding all logs
    // after this point and breaking the deferred probe's bus-idle timing. The
    // daily nolog build below calls it for power saving.
    LOG("[BOOT] pwr_initPowerSaving() skipped (log build)\n");
#else
    pwr_initPowerSaving();
#endif

    while (1)
    {
        gSdCard.Update();
        gSdCard.Update();
    #ifdef ENABLE_R4_MODE
        ntrc_gameR4Update();
    #endif
    #ifdef ENABLE_PSRAM_CACHE
        ntrc_sdCacheFetchDrain();
        romCacheUpdate();
    #endif
    #ifdef ENABLE_UART_LOG
        // LOG only queues text. Keep updating SD while the hardware FIFO is
        // filled in non-blocking pieces; sleep only after the queue empties.
        // A message queued by core1 wakes this WFE through SEV.
        if (!uartLogDrain())
            __wfe();
    #else
        __wfi();
    #endif
    }
}
