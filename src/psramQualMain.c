// Standalone APS6404L qualification firmware (design stage S2, section 11.4).
//
// This target is built instead of / alongside the cartridge firmware and must
// never run at the same time as it on the bus. It drives only the PSRAM pins
// (GPIO0, 22..29); the NDS cartridge pins GPIO9..21 are left high-Z and the SD
// interface is not initialised. Output goes to the existing UART1 boot log
// (USB_DP TX, 115200 8N1).

#include "common.h"
#include <string.h>
#include <stdio.h>
#include "hardware/vreg.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
#ifdef PSRAM_QUAL_WARM_CYCLES
#include "hardware/watchdog.h"
#endif
#include "uartLog.h"
#include "psram.h"

#define LOG(...) uartLogPrintfBlocking(__VA_ARGS__)

#ifndef PSRAM_QUAL_FULL
// Full 8 MiB per-chip walk is available but left off for the first bring-up;
// the bounded regional + isolation tests below validate the electrical path.
#define PSRAM_QUAL_FULL 0
#endif

#ifndef UART_LOG_BUILD_TAG
#define UART_LOG_BUILD_TAG "PSRAM_QUAL"
#endif

#define QUAL_TEST_BYTES 64u

#ifdef PSRAM_QUAL_WARM_CYCLES
#define QUAL_WARM_MAGIC 0x51574D32u
static u32 warmCycleAtBoot(void)
{
    // SDK watchdog_reboot() uses scratch[4..7]; 0..2 belong to this isolated
    // qualifier. A non-watchdog start always begins a new sequence.
    if (!watchdog_caused_reboot() || watchdog_hw->scratch[0] != QUAL_WARM_MAGIC)
        return 0;
    u32 cycle = watchdog_hw->scratch[1];
    if (watchdog_hw->scratch[2] != ~cycle || cycle > PSRAM_QUAL_WARM_CYCLES)
        return 0;
    return cycle;
}

static void warmCycleFinish(u32 cycle, bool passed)
{
    if (!passed)
    {
        LOG("[psram-reset] cycle=%lu/%u result=FAIL halt=1\n",
            (unsigned long)cycle, (unsigned)PSRAM_QUAL_WARM_CYCLES);
        return;
    }
    if (cycle == PSRAM_QUAL_WARM_CYCLES)
    {
        LOG("[psram-reset] result=WARM_%u_OK completed=%u\n",
            (unsigned)PSRAM_QUAL_WARM_CYCLES, (unsigned)PSRAM_QUAL_WARM_CYCLES);
        return;
    }
    u32 next = cycle + 1u;
    watchdog_hw->scratch[0] = QUAL_WARM_MAGIC;
    watchdog_hw->scratch[1] = next;
    watchdog_hw->scratch[2] = ~next;
    LOG("[psram-reset] cycle=%lu/%u result=OK next=watchdog\n",
        (unsigned long)cycle, (unsigned)PSRAM_QUAL_WARM_CYCLES);
    psramDeselectAll();
    busy_wait_ms(20); // let the UART shift register drain before reset
    watchdog_start_tick(XOSC_MHZ);
    watchdog_reboot(0, 0, 100);
    while (1) tight_loop_contents();
}
#endif

static void ndsPinsToHighZ(void)
{
    // Keep every cartridge control/data line an input with pulls disabled.
    const uint pins[] = { 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21 };
    for (unsigned i = 0; i < sizeof(pins) / sizeof(pins[0]); i++)
    {
        gpio_init(pins[i]);
        gpio_set_dir(pins[i], GPIO_IN);
        gpio_disable_pulls(pins[i]);
    }
}

static void fillPattern(u8* buf, u32 len, u32 seed)
{
    u32 x = seed * 2654435761u + 0x9E3779B9u;
    for (u32 i = 0; i < len; i++)
    {
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        buf[i] = (u8)x;
    }
}

// Write \p len bytes with a seed derived from chip/address using the path
// selected by \p usePio, read them back with the path selected by \p readPio,
// and compare.
static bool verifyRegion(u32 chip, u32 addr, u32 len, bool usePio, bool readPio,
                         u8* wbuf, u8* rbuf)
{
    fillPattern(wbuf, len, chip * 131u + addr);
    memset(rbuf, 0, len);

    bool sSavedUsePio = gPsramUsePio;

    gPsramUsePio = usePio;
    bool okw = psramWrite(chip, addr, wbuf, len);
    gPsramUsePio = readPio;
    bool okr = psramRead(chip, addr, rbuf, len);
    gPsramUsePio = sSavedUsePio;

    if (!okw || !okr || memcmp(wbuf, rbuf, len) != 0)
    {
        LOG("[psram] chip=%lu ref=U%c addr=0x%06lX MISMATCH wr=%s rd=%s\n",
            (unsigned long)chip, gPsramChips[chip].refdes, (unsigned long)addr,
            usePio ? "pio" : "bb", readPio ? "pio" : "bb");
        LOG("[psram]   wrote ");
        for (u32 i = 0; i < 8; i++) LOG("%02X", wbuf[i]);
        LOG("  read ");
        for (u32 i = 0; i < 8; i++) LOG("%02X", rbuf[i]);
        LOG("\n");
        return false;
    }
    return true;
}

static bool testChipRegions(u32 chip, u8* wbuf, u8* rbuf)
{
    static const u32 addrs[] = {
        0x000000, 0x000400, 0x001000, 0x3FFC00, 0x7FFF00,
    };
    bool allOk = true;

    for (unsigned i = 0; i < sizeof(addrs) / sizeof(addrs[0]); i++)
    {
        bool bbOk = verifyRegion(chip, addrs[i], QUAL_TEST_BYTES, false, false, wbuf, rbuf);
        bool pioOk = true;
        if (gPsramUsePio)
        {
            // Cross the data paths so a write-side and a read-side PIO bug
            // cannot cancel out.
            bool pioW = verifyRegion(chip, addrs[i], QUAL_TEST_BYTES, true, false, wbuf, rbuf);
            bool pioR = verifyRegion(chip, addrs[i], QUAL_TEST_BYTES, false, true, wbuf, rbuf);
            pioOk = pioW && pioR;
        }

        LOG("[psram] chip=%lu ref=U%c addr=0x%06lX bb=%s pio=%s\n",
            (unsigned long)chip, gPsramChips[chip].refdes, (unsigned long)addrs[i],
            bbOk ? "OK" : "FAIL",
            gPsramUsePio ? (pioOk ? "OK" : "FAIL") : "n/a");
        allOk = allOk && bbOk && pioOk;
    }
    return allOk;
}

static const u32 kProbeAddrs[] = {
    0x000000, 0x000400, 0x001000, 0x3FFC00, 0x7FFF00,
};

// Run the five-address pattern test through the PIO engine only, without the
// per-address logging, so a frequency sweep stays readable.
static bool pioRegionsPass(u32 chip, u8* wbuf, u8* rbuf)
{
    bool saved = gPsramUsePio;
    gPsramUsePio = true;
    bool ok = true;
    for (unsigned i = 0; i < sizeof(kProbeAddrs) / sizeof(kProbeAddrs[0]); i++)
        if (!verifyRegion(chip, kProbeAddrs[i], QUAL_TEST_BYTES, true, true, wbuf, rbuf))
            ok = false;
    gPsramUsePio = saved;
    return ok;
}

// Design section 11.4: frequency must be brought up in steps and the stable
// point plus margin recorded. div is listed fastest-last so the final OK is the
// highest passing SCLK.
static bool sweepPioSpeeds(u32 chip, u8* wbuf, u8* rbuf)
{
    static const float divs[] = { 40.0f, 20.0f, 10.0f, 5.0f, 3.0f, 2.0f, 1.0f };
    float fastestDiv = 0.0f;

    for (unsigned i = 0; i < sizeof(divs) / sizeof(divs[0]); i++)
    {
        psramSetClockDiv(divs[i]);
        bool ok = pioRegionsPass(chip, wbuf, rbuf);
        u32 cmdKhz = (u32)(clock_get_hz(clk_sys) / (2u * (u32)divs[i]) / 1000u);
        u32 rdKhz = (u32)(clock_get_hz(clk_sys) / (3u * (u32)divs[i]) / 1000u);
        LOG("[psram] chip=%lu pio div=%.0f cmd_sclk_khz=%lu rd_sclk_khz=%lu %s\n",
            (unsigned long)chip, (double)divs[i], (unsigned long)cmdKhz,
            (unsigned long)rdKhz, ok ? "OK" : "FAIL");
        if (ok)
            fastestDiv = divs[i];
    }

    psramSetClockDiv(PSRAM_PIO_CLKDIV);
    if (fastestDiv > 0.0f)
        LOG("[psram] chip=%lu pio fastest_pass_div=%.0f rd_sclk_khz=%lu\n",
            (unsigned long)chip, (double)fastestDiv,
            (unsigned long)(clock_get_hz(clk_sys) / (3u * (u32)fastestDiv) / 1000u));
    else
        LOG("[psram] chip=%lu pio fastest_pass_div=none\n", (unsigned long)chip);
    return fastestDiv > 0.0f;
}

// Every chip sees the same address, so if a CE# or direction switch leaked the
// read-back patterns would collide.
static bool testChipIsolation(u8* wbuf, u8* rbuf)
{
    const u32 probeAddr = 0x000800;
    u8 patterns[PSRAM_CHIP_COUNT][QUAL_TEST_BYTES];
    bool allOk = true;

    for (u32 chip = 0; chip < PSRAM_CHIP_COUNT; chip++)
    {
        if (!gPsramChips[chip].present)
            continue;
        fillPattern(patterns[chip], QUAL_TEST_BYTES, 0x1000u + chip);
        if (!psramWrite(chip, probeAddr, patterns[chip], QUAL_TEST_BYTES))
            allOk = false;
    }
    for (u32 chip = 0; chip < PSRAM_CHIP_COUNT; chip++)
    {
        if (!gPsramChips[chip].present)
            continue;
        memset(rbuf, 0, QUAL_TEST_BYTES);
        if (!psramRead(chip, probeAddr, rbuf, QUAL_TEST_BYTES))
            allOk = false;
        else if (memcmp(rbuf, patterns[chip], QUAL_TEST_BYTES) != 0)
        {
            LOG("[psram] isolation FAIL chip=%lu own data mismatch\n",
                (unsigned long)chip);
            allOk = false;
        }
        for (u32 other = 0; other < PSRAM_CHIP_COUNT; other++)
        {
            if (other == chip || !gPsramChips[other].present)
                continue;
            if (memcmp(rbuf, patterns[other], QUAL_TEST_BYTES) == 0)
            {
                LOG("[psram] isolation FAIL chip=%lu matches chip=%lu\n",
                    (unsigned long)chip, (unsigned long)other);
                allOk = false;
            }
        }
    }
    LOG("[psram] chip isolation %s\n", allOk ? "OK" : "FAIL");
    return allOk;
}

static bool testChipTailAndBounds(u32 chip, u8* wbuf, u8* rbuf)
{
    const u32 base = PSRAM_SIZE_BYTES - 8u;
    const u8 initial[8] = { 0xA5, 0x10, 0xC3, 0x7E, 0x29, 0xD4, 0x56, 0xB8 };
    const u8 patch[3] = { 0x91, 0xE2, 0x3F };
    const u8 firstByte = 0x4Du;
    u8 expected[8];
    memcpy(expected, initial, sizeof(expected));

    bool ok = psramWrite(chip, 0u, &firstByte, 1u);
    ok = psramWrite(chip, base, initial, sizeof(initial)) && ok;
    ok = psramWrite(chip, base + 3u, patch, sizeof(patch)) && ok;
    memcpy(expected + 3u, patch, sizeof(patch));
    wbuf[0] = 0x6Bu;
    ok = psramWrite(chip, PSRAM_SIZE_BYTES - 1u, wbuf, 1u) && ok;
    expected[7] = wbuf[0];

    memset(rbuf, 0, sizeof(expected));
    ok = psramRead(chip, base, rbuf, sizeof(expected)) && ok;
    ok = memcmp(rbuf, expected, sizeof(expected)) == 0 && ok;
    memset(rbuf, 0, sizeof(patch));
    ok = psramRead(chip, base + 3u, rbuf, sizeof(patch)) && ok;
    ok = memcmp(rbuf, patch, sizeof(patch)) == 0 && ok;
    rbuf[0] = 0u;
    ok = psramRead(chip, 0u, rbuf, 1u) && ok;
    ok = rbuf[0] == firstByte && ok;

    // Start one byte before a 32-byte boundary, then end with a short tail.
    // The bytes on either side must survive the mixed PIO/SIO transaction.
    u8 expectedMiddle[40];
    u8 patchMiddle[35];
    for (u32 i = 0; i < sizeof(expectedMiddle); i++)
        expectedMiddle[i] = (u8)(0x40u + i);
    for (u32 i = 0; i < sizeof(patchMiddle); i++)
        patchMiddle[i] = (u8)(0xD3u ^ i);
    ok = psramWrite(chip, 30u, expectedMiddle, sizeof(expectedMiddle)) && ok;
    ok = psramWrite(chip, 31u, patchMiddle, sizeof(patchMiddle)) && ok;
    memcpy(expectedMiddle + 1u, patchMiddle, sizeof(patchMiddle));
    memset(rbuf, 0, sizeof(expectedMiddle));
    ok = psramRead(chip, 30u, rbuf, sizeof(expectedMiddle)) && ok;
    ok = memcmp(rbuf, expectedMiddle, sizeof(expectedMiddle)) == 0 && ok;

    // Invalid operations must be rejected before any CE goes low, including
    // the UINT32 wraparound case.
    bool rejectedRead = !psramRead(chip, PSRAM_SIZE_BYTES, rbuf, 1u);
    bool rejectedWrite = !psramWrite(chip, PSRAM_SIZE_BYTES - 1u, wbuf, 2u);
    bool rejectedWrap = !psramRead(chip, UINT32_MAX, rbuf, 2u);
    bool rejected = rejectedRead && rejectedWrite && rejectedWrap;
    LOG("[psram] chip=%lu tail_frag_exact=%s bounds_reject=%s\n",
        (unsigned long)chip, ok ? "OK" : "FAIL", rejected ? "OK" : "FAIL");
    return ok && rejected;
}

#ifdef PSRAM_QUAL_FAULT_TEST
static bool testFaultCase(u32 chip, u8 fault, const char* name,
                          u8* wbuf, u8* rbuf)
{
    const u32 addr = 0x600000u + (u32)fault * QUAL_TEST_BYTES;
    fillPattern(wbuf, QUAL_TEST_BYTES, chip * 17u + fault);
    bool primed = psramWrite(chip, addr, wbuf, QUAL_TEST_BYTES);
    if (!primed)
    {
        LOG("[psram-fault] chip=%lu case=%s prime=FAIL\n",
            (unsigned long)chip, name);
        return false;
    }

    psramQualInjectFaultOnce(fault);
    u32 started = time_us_32();
    bool rejected = fault == PSRAM_QUAL_FAULT_RX_STALL
        ? !psramRead(chip, addr, rbuf, PSRAM_PIO_FRAG_BYTES)
        : !psramWrite(chip, addr, wbuf, PSRAM_PIO_FRAG_BYTES);
    u32 elapsed = time_us_32() - started;
    bool idle = psramQualBusIdle();

    // The aborted write may have changed its destination. A fresh complete
    // transaction must restore data, proving that no stale FIFO/DMA state
    // leaks into the next request.
    bool recovered = rejected && idle &&
                     psramWrite(chip, addr, wbuf, QUAL_TEST_BYTES) &&
                     psramRead(chip, addr, rbuf, QUAL_TEST_BYTES) &&
                     memcmp(wbuf, rbuf, QUAL_TEST_BYTES) == 0;
    bool ok = rejected && idle && recovered;
    LOG("[psram-fault] chip=%lu case=%s rejected=%u idle=%u recovered=%u elapsed_us=%lu result=%s\n",
        (unsigned long)chip, name, rejected ? 1u : 0u, idle ? 1u : 0u,
        recovered ? 1u : 0u, (unsigned long)elapsed, ok ? "OK" : "FAIL");
    return ok;
}

static bool testChipFaultRecovery(u32 chip, u8* wbuf, u8* rbuf)
{
    bool rx = testFaultCase(chip, PSRAM_QUAL_FAULT_RX_STALL,
                            "rx_stall", wbuf, rbuf);
    bool tx = rx && testFaultCase(chip, PSRAM_QUAL_FAULT_TX_STALL,
                                  "tx_stall", wbuf, rbuf);
    bool abort = tx && testFaultCase(chip, PSRAM_QUAL_FAULT_TX_AFTER_DMA,
                                     "tx_after_dma", wbuf, rbuf);
    bool cross = false;
    if (abort)
    {
        // Switch CE immediately after a recovered fault. The other chip may
        // not have been probed yet in this standalone sequence, so reset it
        // explicitly before comparing its own data.
        u32 other = (chip + 1u) & (PSRAM_CHIP_COUNT - 1u);
        psramResetChip(other);
        fillPattern(wbuf, QUAL_TEST_BYTES, chip * 41u + other);
        cross = psramWrite(other, 0x600400u, wbuf, QUAL_TEST_BYTES) &&
                psramRead(other, 0x600400u, rbuf, QUAL_TEST_BYTES) &&
                memcmp(wbuf, rbuf, QUAL_TEST_BYTES) == 0;
    }
    bool ok = rx && tx && abort && cross;
    LOG("[psram-fault] chip=%lu next_chip=%lu cross_recovery=%s recovery=%s\n",
        (unsigned long)chip, (unsigned long)((chip + 1u) & (PSRAM_CHIP_COUNT - 1u)),
        cross ? "OK" : "FAIL", ok ? "OK" : "FAIL");
    return ok;
}
#endif

#if PSRAM_QUAL_FULL
typedef enum {
    QUAL_ZERO,
    QUAL_ONES,
    QUAL_AA,
    QUAL_55,
    QUAL_ADDRESS,
    QUAL_RANDOM,
    QUAL_PATTERN_COUNT,
} qualPattern;

static const char* const kPatternNames[QUAL_PATTERN_COUNT] = {
    "00", "FF", "AA", "55", "address", "random",
};

static void fullPattern(u8* buf, u32 chip, u32 addr, qualPattern pattern)
{
    switch (pattern)
    {
    case QUAL_ZERO: memset(buf, 0x00, QUAL_TEST_BYTES); break;
    case QUAL_ONES: memset(buf, 0xFF, QUAL_TEST_BYTES); break;
    case QUAL_AA: memset(buf, 0xAA, QUAL_TEST_BYTES); break;
    case QUAL_55: memset(buf, 0x55, QUAL_TEST_BYTES); break;
    case QUAL_ADDRESS:
        for (u32 i = 0; i < QUAL_TEST_BYTES; i += 4)
        {
            u32 word = (addr + i) ^ (chip << 24);
            for (u32 b = 0; b < 4; b++) buf[i + b] = (u8)(word >> (8u * b));
        }
        break;
    case QUAL_RANDOM:
        fillPattern(buf, QUAL_TEST_BYTES, addr ^ (chip << 24) ^ 0xB5A1634Du);
        break;
    default: break;
    }
}

static bool testChipFullCapacity(u32 chip, u8* wbuf, u8* rbuf)
{
    // A write-then-immediate-read loop misses distant address aliases: the
    // aliased location is overwritten just before it is checked. Write the
    // ENTIRE chip first, then read it in a separate pass for each pattern.
    // These are qualification tests, never part of cartridge startup.
    u32 totalErrors = 0;
    for (u32 p = 0; p < QUAL_PATTERN_COUNT; p++)
    {
        u32 errors = 0;
        LOG("[psram] chip=%lu full pattern=%s phase=write begin\n",
            (unsigned long)chip, kPatternNames[p]);
        for (u32 addr = 0; addr < PSRAM_SIZE_BYTES; addr += QUAL_TEST_BYTES)
        {
            fullPattern(wbuf, chip, addr, (qualPattern)p);
            if (!psramWrite(chip, addr, wbuf, QUAL_TEST_BYTES))
            {
                LOG("[psram] chip=%lu full pattern=%s write-fail @0x%06lX\n",
                    (unsigned long)chip, kPatternNames[p], (unsigned long)addr);
                return false; // transport state is not safe to reuse
            }
            if ((addr & 0xFFFFFu) == 0)
                LOG("[psram] chip=%lu full pattern=%s write 0x%06lX\n",
                    (unsigned long)chip, kPatternNames[p], (unsigned long)addr);
        }
        LOG("[psram] chip=%lu full pattern=%s phase=read begin\n",
            (unsigned long)chip, kPatternNames[p]);
        for (u32 addr = 0; addr < PSRAM_SIZE_BYTES; addr += QUAL_TEST_BYTES)
        {
            fullPattern(wbuf, chip, addr, (qualPattern)p);
            if (!psramRead(chip, addr, rbuf, QUAL_TEST_BYTES))
            {
                LOG("[psram] chip=%lu full pattern=%s read-fail @0x%06lX\n",
                    (unsigned long)chip, kPatternNames[p], (unsigned long)addr);
                return false;
            }
            if (memcmp(wbuf, rbuf, QUAL_TEST_BYTES) != 0)
            {
                if (errors < 8)
                {
                    u32 i = 0;
                    while (i < QUAL_TEST_BYTES && wbuf[i] == rbuf[i]) i++;
                    LOG("[psram] chip=%lu full pattern=%s mismatch @0x%06lX exp=%02X got=%02X\n",
                        (unsigned long)chip, kPatternNames[p], (unsigned long)(addr + i),
                        wbuf[i], rbuf[i]);
                }
                errors++;
            }
            if ((addr & 0xFFFFFu) == 0)
                LOG("[psram] chip=%lu full pattern=%s read 0x%06lX\n",
                    (unsigned long)chip, kPatternNames[p], (unsigned long)addr);
        }
        LOG("[psram] chip=%lu full pattern=%s errors=%lu\n",
            (unsigned long)chip, kPatternNames[p], (unsigned long)errors);
        totalErrors += errors;
    }
    LOG("[psram] chip=%lu full capacity bytes=%lu patterns=%u errors=%lu\n",
        (unsigned long)chip, (unsigned long)PSRAM_SIZE_BYTES,
        (unsigned)QUAL_PATTERN_COUNT, (unsigned long)totalErrors);
    return totalErrors == 0;
}
#endif

int main(void)
{
    vreg_set_voltage(VREG_VOLTAGE_1_15);
    busy_wait_us(100);
    set_sys_clock_khz(200000, true);

    uartLogInit();
    LOG("\n[boot] build=%s stage=S2 qual=1 sys_khz=%lu\n",
        UART_LOG_BUILD_TAG, (unsigned long)(clock_get_hz(clk_sys) / 1000u));
#ifdef PSRAM_QUAL_WARM_CYCLES
    u32 warmCycle = warmCycleAtBoot();
    LOG("[psram-reset] cycle=%lu/%u cause=%s\n", (unsigned long)warmCycle,
        (unsigned)PSRAM_QUAL_WARM_CYCLES,
        watchdog_caused_reboot() ? "watchdog" : "non_watchdog");
#endif

    ndsPinsToHighZ();
    psramGpioInit();

    bool engine = psramEngineInit();
    LOG("[psram] engine pio2_sm0=%s dma_tx=%d dma_rx=%d div=%.1f cmd_sclk_khz=%lu rd_sclk_khz=%lu\n",
        engine ? "on" : "off", engine ? 4 : -1, engine ? 5 : -1,
        (double)PSRAM_PIO_CLKDIV,
        (unsigned long)(clock_get_hz(clk_sys) / (2u * (u32)PSRAM_PIO_CLKDIV) / 1000u),
        (unsigned long)(clock_get_hz(clk_sys) / (3u * (u32)PSRAM_PIO_CLKDIV) / 1000u));
    if (!engine)
    {
        gPsramUsePio = false;
        LOG("[psram] PIO2/DMA unavailable; running bit-bang only\n");
    }

    static u8 wbuf[QUAL_TEST_BYTES] __attribute__((aligned(4)));
    static u8 rbuf[QUAL_TEST_BYTES] __attribute__((aligned(4)));
    u32 presentMask = 0;
    u32 qualifiedMask = 0;
#ifdef PSRAM_QUAL_FAULT_TEST
    u32 faultMask = 0;
#endif

    for (u32 chip = 0; chip < PSRAM_CHIP_COUNT; chip++)
    {
        psramResetChip(chip);
        psramReadId(chip, gPsramChips[chip].id);

        u32 mf = 0, kgd = 0;
        u32 size = psramDecodeIdSize(gPsramChips[chip].id, &mf, &kgd);
        gPsramChips[chip].idValid = (mf == 0x0D && kgd == 0x5D);
        gPsramChips[chip].idSizeBytes = size;

        LOG("[psram] chip=%lu ref=U%c ce_gpio=%u part=APS6404L nominal_bytes=%lu\n",
            (unsigned long)chip, gPsramChips[chip].refdes, gPsramChips[chip].cePin,
            (unsigned long)PSRAM_SIZE_BYTES);
        LOG("[psram] chip=%lu id=", (unsigned long)chip);
        for (int i = 0; i < 16; i++)
            LOG("%02X", gPsramChips[chip].id[i]);
        LOG(" mf=0x%02lX kgd=0x%02lX size=%lu\n",
            (unsigned long)mf, (unsigned long)kgd, (unsigned long)size);

        bool regionsOk = testChipRegions(chip, wbuf, rbuf);
        bool anySpeed = gPsramUsePio ? sweepPioSpeeds(chip, wbuf, rbuf) : false;
        bool runtimeSpeedOk = gPsramUsePio && pioRegionsPass(chip, wbuf, rbuf);
        bool tailBoundsOk = testChipTailAndBounds(chip, wbuf, rbuf);
#ifdef PSRAM_QUAL_FAULT_TEST
        bool faultOk = engine && runtimeSpeedOk &&
                       testChipFaultRecovery(chip, wbuf, rbuf);
        if (!faultOk)
        {
            LOG("[psram-fault] chip=%lu HALT after failed recovery\n",
                (unsigned long)chip);
            psramDeselectAll();
            while (1)
                tight_loop_contents();
        }
        if (faultOk)
            faultMask |= 1u << chip;
#endif
        gPsramChips[chip].quadWriteOk = regionsOk && runtimeSpeedOk;
        gPsramChips[chip].quadReadOk = regionsOk && runtimeSpeedOk;
        gPsramChips[chip].present = regionsOk || anySpeed || gPsramChips[chip].idValid;
        if (gPsramChips[chip].present)
            presentMask |= (1u << chip);

        bool regionRuntimeOk = engine && gPsramChips[chip].idValid &&
                               size == PSRAM_SIZE_BYTES && regionsOk && runtimeSpeedOk &&
                               tailBoundsOk;
#ifdef PSRAM_QUAL_FAULT_TEST
        regionRuntimeOk = regionRuntimeOk && faultOk;
#endif
#if PSRAM_QUAL_FULL
        bool fullOk = regionRuntimeOk && testChipFullCapacity(chip, wbuf, rbuf);
        if (fullOk)
            qualifiedMask |= (1u << chip);
#endif

        LOG("[psram] chip=%lu region_runtime=%s full=%s\n", (unsigned long)chip,
            regionRuntimeOk ? "OK" : "FAIL",
#if PSRAM_QUAL_FULL
            fullOk ? "OK" : "FAIL");
#else
            "NOT_RUN");
#endif
        psramDeselectAll();
    }

    bool isolated = presentMask && testChipIsolation(wbuf, rbuf);
#ifdef PSRAM_QUAL_FAULT_TEST
    LOG("[psram-fault] result=%s mask=%lX\n",
        faultMask == 0xFu ? "FOUR_CHIP_OK" : "FAIL", (unsigned long)faultMask);
#endif
    if (!isolated)
        qualifiedMask = 0; // one CE alias invalidates the entire shared-bus profile

    LOG("[cache] qual result=%s present_mask=%lX qualified_mask=%lX isolated=%u tested_bytes=%lu\n",
#if PSRAM_QUAL_FULL
        qualifiedMask == 0xFu ? "FULL_4CHIP" : (qualifiedMask ? "DEGRADED" : "FAIL"),
        (unsigned long)presentMask, (unsigned long)qualifiedMask, isolated ? 1u : 0u,
        (unsigned long)(PSRAM_SIZE_BYTES * __builtin_popcount(qualifiedMask)));
#else
        "REGION_ONLY", (unsigned long)presentMask, 0ul, isolated ? 1u : 0u, 0ul);
#endif

#ifdef PSRAM_QUAL_WARM_CYCLES
    warmCycleFinish(warmCycle, engine && presentMask == 0xFu &&
                    faultMask == 0xFu && isolated);
#endif

    while (1)
    {
        tight_loop_contents();
    }
}
