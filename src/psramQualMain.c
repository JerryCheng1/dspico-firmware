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
#include "uartLog.h"
#include "psram.h"

#define LOG(...) uartLogPrintfBlocking(__VA_ARGS__)

#ifndef PSRAM_QUAL_FULL
// Full 8 MiB per-chip walk is available but left off for the first bring-up;
// the bounded regional + isolation tests below validate the electrical path.
#define PSRAM_QUAL_FULL 0
#endif

#define QUAL_TEST_BYTES 64u

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

#if PSRAM_QUAL_FULL
static bool testChipFullCapacity(u32 chip, u8* wbuf, u8* rbuf)
{
    u32 errors = 0;
    for (u32 addr = 0; addr < PSRAM_SIZE_BYTES; addr += QUAL_TEST_BYTES)
    {
        if (!verifyRegion(chip, addr, QUAL_TEST_BYTES, gPsramUsePio, gPsramUsePio,
                          wbuf, rbuf))
        {
            if (++errors <= 8)
                LOG("[psram] chip=%lu full-test error @0x%06lX\n",
                    (unsigned long)chip, (unsigned long)addr);
        }
        if ((addr & 0xFFFFFu) == 0)
            LOG("[psram] chip=%lu full-test 0x%06lX\n",
                (unsigned long)chip, (unsigned long)addr);
    }
    LOG("[psram] chip=%lu full capacity errors=%lu\n",
        (unsigned long)chip, (unsigned long)errors);
    return errors == 0;
}
#endif

int main(void)
{
    vreg_set_voltage(VREG_VOLTAGE_1_15);
    busy_wait_us(100);
    set_sys_clock_khz(200000, true);

    uartLogInit();
    LOG("\n[boot] build=%s stage=S2 qual=1 sys_khz=%lu\n",
        "PSRAM_QUAL", (unsigned long)(clock_get_hz(clk_sys) / 1000u));

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

        bool ok = testChipRegions(chip, wbuf, rbuf);
        bool anySpeed = gPsramUsePio ? sweepPioSpeeds(chip, wbuf, rbuf) : false;
        gPsramChips[chip].quadWriteOk = ok || anySpeed;
        gPsramChips[chip].quadReadOk = ok || anySpeed;
        gPsramChips[chip].present = ok || anySpeed || gPsramChips[chip].idValid;
        if (gPsramChips[chip].present)
            presentMask |= (1u << chip);

#if PSRAM_QUAL_FULL
        if (gPsramChips[chip].present)
            ok = testChipFullCapacity(chip, wbuf, rbuf) && ok;
#endif

        LOG("[psram] chip=%lu result=%s\n", (unsigned long)chip,
            ok ? "OK" : (gPsramChips[chip].present ? "FAIL" : "ABSENT"));
        psramDeselectAll();
    }

    if (presentMask)
        testChipIsolation(wbuf, rbuf);

    LOG("[cache] init local_result=%s usable_mask=%lX\n",
        presentMask == 0xFu ? "FULL_4CHIP" : (presentMask ? "DEGRADED" : "CACHE_OFF"),
        (unsigned long)presentMask);

    while (1)
    {
        tight_loop_contents();
    }
}
