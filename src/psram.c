#include "common.h"
#include <string.h>
#include "hardware/gpio.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/structs/sio.h"
#include "psram.h"
#include "psram.pio.h"

// APS6404L command set (design section 2.6).
#define PSRAM_CMD_RESET_ENABLE   0x66
#define PSRAM_CMD_RESET          0x99
#define PSRAM_CMD_READ_ID        0x9F
#define PSRAM_CMD_READ           0x03
#define PSRAM_CMD_WRITE          0x02
#define PSRAM_CMD_FAST_READ_QUAD 0xEB   // 6 dummy clocks
#define PSRAM_CMD_QUAD_WRITE     0x38   // no dummy clocks

#define PSRAM_RX_DUMMY_NIBBLES   6

// Upper bound on the number of polling iterations a single PSRAM fragment may
// spend waiting for DMA/PIO. The fragment is at most PSRAM_PIO_FRAG_BYTES and
// the engine runs at PSRAM_PIO_CLKDIV, so a healthy transfer settles in a few
// hundred iterations; 200k (~1 ms at 200 MHz) is far above any legitimate
// completion yet keeps the runtime cache step bounded instead of spinning
// forever on a wedged DMA/PIO (design section 9.2 / R8).
#define PSRAM_XFER_SPIN_LIMIT 200000u

psramChipStatus gPsramChips[PSRAM_CHIP_COUNT] = {
    { PSRAM_PIN_CE0, '2', false, false, { 0 }, 0, false, false },
    { PSRAM_PIN_CE1, '4', false, false, { 0 }, 0, false, false },
    { PSRAM_PIN_CE2, '5', false, false, { 0 }, 0, false, false },
    { PSRAM_PIN_CE3, '6', false, false, { 0 }, 0, false, false },
};
volatile bool gPsramUsePio = false;

static int sPioOffset = -1;
static int sDmaTx = -1;
static int sDmaRx = -1;
static bool sEngineReady = false;
static volatile u32 sRuntimeState = PSRAM_STATE_NONE;
#ifdef PSRAM_QUAL_FAULT_TEST
static volatile u8 sQualFaultOnce;
#endif

// Isolated self-test block, above the 512 KiB/chip the sector cache uses.
#define PSRAM_SELFTEST_ADDR (PSRAM_SIZE_BYTES - 128u)
#define PSRAM_SELFTEST_BYTES 32u

// ---------------------------------------------------------------------------
// GPIO / bus ownership
// ---------------------------------------------------------------------------

// Design section 8.3: once the run-time PIO mode is fixed, do NOT switch the
// SIO/PIO mux on every fragment. Track the current owner and only reprogram the
// pin functions on init/restore/qualification transitions.
static bool sMuxIsPio;

static inline void muxToSio(void)
{
    if (!sMuxIsPio)
        return;
    gpio_set_function(PSRAM_PIN_SCLK, GPIO_FUNC_SIO);
    for (uint p = PSRAM_PIN_IO0; p <= PSRAM_PIN_IO3; p++)
        gpio_set_function(p, GPIO_FUNC_SIO);
    sMuxIsPio = false;
}

static inline void muxToPio(void)
{
    if (sMuxIsPio)
        return;
    gpio_set_function(PSRAM_PIN_SCLK, GPIO_FUNC_PIO2);
    for (uint p = PSRAM_PIN_IO0; p <= PSRAM_PIN_IO3; p++)
        gpio_set_function(p, GPIO_FUNC_PIO2);
    sMuxIsPio = true;
}

void psramDeselectAll(void)
{
    // All CE# high: no device drives the shared data bus.
    sio_hw->gpio_set = PSRAM_CE_MASK;
}

void psramSelectChip(u32 chip)
{
    if (chip >= PSRAM_CHIP_COUNT)
        return;
    sio_hw->gpio_set = PSRAM_CE_MASK;
    sio_hw->gpio_clr = 1u << (PSRAM_PIN_CE0 + chip);
}

void psramGpioInit(void)
{
    for (uint p = 0; p < PSRAM_CHIP_COUNT; p++)
        gpio_init(PSRAM_PIN_CE0 + p);
    gpio_init(PSRAM_PIN_SCLK);
    for (uint p = PSRAM_PIN_IO0; p <= PSRAM_PIN_IO3; p++)
        gpio_init(p);

    // Bring CE# high and SCLK low before enabling any output driver so power-up
    // cannot present a spurious low chip select (design section 9.1).
    sio_hw->gpio_set = PSRAM_CE_MASK;
    sio_hw->gpio_clr = PSRAM_SCLK_MASK;
    sio_hw->gpio_oe_clr = PSRAM_IO_MASK;
    sio_hw->gpio_oe_set = PSRAM_CE_MASK | PSRAM_SCLK_MASK;

    for (uint p = 0; p < PSRAM_CHIP_COUNT; p++)
        gpio_set_dir(PSRAM_PIN_CE0 + p, GPIO_OUT);
    gpio_set_dir(PSRAM_PIN_SCLK, GPIO_OUT);
    gpio_put(PSRAM_PIN_SCLK, false);
    for (uint p = PSRAM_PIN_IO0; p <= PSRAM_PIN_IO3; p++)
    {
        gpio_set_dir(p, GPIO_IN);
        gpio_set_input_enabled(p, true);
        gpio_set_function(p, GPIO_FUNC_SIO);
        gpio_set_slew_rate(p, GPIO_SLEW_RATE_FAST);
        gpio_set_drive_strength(p, GPIO_DRIVE_STRENGTH_4MA);
    }
    gpio_set_slew_rate(PSRAM_PIN_SCLK, GPIO_SLEW_RATE_FAST);
    gpio_set_drive_strength(PSRAM_PIN_SCLK, GPIO_DRIVE_STRENGTH_4MA);
    sMuxIsPio = false; // psramGpioInit() reprogrammed every pin to SIO

    // The datasheet (P3) requires the device to see at least 150 us of stable
    // power before the first command. The board rails are up long before this
    // firmware runs; the explicit delay is a conservative guard for the
    // standalone qualification boot.
    busy_wait_us(200);
}

// ---------------------------------------------------------------------------
// SIO bit-bang backend (low-speed SPI init and fallback data path)
// ---------------------------------------------------------------------------

static inline void bbClkLow(void) { sio_hw->gpio_clr = PSRAM_SCLK_MASK; }
static inline void bbClkHigh(void) { sio_hw->gpio_set = PSRAM_SCLK_MASK; }

// One serial SPI byte out on SIO0, CPOL=0/CPHA=0. Leaves SCLK low.
static void bbSpiSendByte(u8 b)
{
    bbClkLow();
    sio_hw->gpio_oe_set = 1u << PSRAM_PIN_IO0;
    for (int i = 7; i >= 0; i--)
    {
        if (b & (1u << i))
            sio_hw->gpio_set = 1u << PSRAM_PIN_IO0;
        else
            sio_hw->gpio_clr = 1u << PSRAM_PIN_IO0;
        bbClkHigh();
        bbClkLow();
    }
}

// One serial SPI byte in on SIO1, sampled on the rising edge.
static u8 bbSpiRecvByte(void)
{
    u8 v = 0;
    for (int i = 7; i >= 0; i--)
    {
        bbClkHigh();
        __asm volatile("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop");
        v = (u8)((v << 1) | ((sio_hw->gpio_in >> PSRAM_PIN_IO1) & 1u));
        bbClkLow();
    }
    return v;
}

// One quad (4-bit) nibble out on SIO0..3. Leaves SCLK low.
static void bbSendNibble(u8 n)
{
    bbClkLow();
    sio_hw->gpio_clr = PSRAM_IO_MASK;
    sio_hw->gpio_oe_set = PSRAM_IO_MASK;
    sio_hw->gpio_set = ((u32)n << PSRAM_PIN_IO0) & PSRAM_IO_MASK;
    bbClkHigh();
    bbClkLow();
}

// One quad nibble in from SIO0..3, sampled on the rising edge.
static u8 bbRecvNibble(void)
{
    bbClkHigh();
    __asm volatile("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop");
    u8 n = (u8)((sio_hw->gpio_in >> PSRAM_PIN_IO0) & 0xFu);
    bbClkLow();
    return n;
}

static void bbReadData(u8* buf, u32 len)
{
    for (u32 i = 0; i < len; i++)
    {
        u8 hi = bbRecvNibble();
        u8 lo = bbRecvNibble();
        buf[i] = (u8)((hi << 4) | lo);
    }
}

void psramResetChip(u32 chip)
{
    muxToSio();
    sio_hw->gpio_oe_clr = PSRAM_IO_MASK;
    psramDeselectAll();
    bbClkLow();
    busy_wait_us(1);

    // 66h and 99h must each be a complete CE#-low command and nothing may be
    // inserted between them.
    psramSelectChip(chip);
    bbSpiSendByte(PSRAM_CMD_RESET_ENABLE);
    psramDeselectAll();

    busy_wait_us(1); // tCPH >= 18 ns with margin
    psramSelectChip(chip);
    bbSpiSendByte(PSRAM_CMD_RESET);
    psramDeselectAll();

    busy_wait_us(100); // reset recovery >= 50 ns with margin
}

void psramReadId(u32 chip, u8 id[16])
{
    muxToSio();
    // Command and address are serial on SIO0; the device answers on SIO1.
    sio_hw->gpio_oe_clr = PSRAM_IO_MASK;
    sio_hw->gpio_oe_set = 1u << PSRAM_PIN_IO0;

    psramSelectChip(chip);
    bbSpiSendByte(PSRAM_CMD_READ_ID);
    // Over-read: the part on this board replies 0x00 0x00 0x00 then the APM
    // signature 0x0D 0x5D, so a fixed 8-byte window hides the layout.
    for (int i = 0; i < 16; i++)
        id[i] = bbSpiRecvByte();
    psramDeselectAll();

    sio_hw->gpio_oe_clr = PSRAM_IO_MASK;
    bbClkLow();
}

u32 psramDecodeIdSize(const u8 id[16], u32* manufacturer, u32* kgd)
{
    // Search for the APM manufacturer (0x0D) + Known-Good-Die (0x5D) pair and
    // decode density from the following EID byte, matching the RP2350 SDK's
    // default mapping for this part family. The raw bytes are always logged so
    // any offset stays visible rather than being silently assumed.
    for (u32 off = 0; off + 2 < 16; off++)
    {
        if (id[off] != 0x0D || id[off + 1] != 0x5D)
            continue;
        *manufacturer = id[off];
        *kgd = id[off + 1];
        u8 eid = id[off + 2];
        u8 sizeId = (u8)(eid >> 5);
        if (eid == 0x26 || sizeId == 2 || sizeId == 3)
            return 8u * 1024u * 1024u;
        if (sizeId == 1)
            return 4u * 1024u * 1024u;
        if (sizeId == 0)
            return 2u * 1024u * 1024u;
        return 0;
    }
    *manufacturer = id[0];
    *kgd = id[1];
    return 0;
}

// ---------------------------------------------------------------------------
// PIO2 + DMA4/5 backend
// ---------------------------------------------------------------------------

bool psramEngineInit(void)
{
    if (sEngineReady)
        return true;
    if (pio_sm_is_claimed(pio2, 0))
        return false;
    if (!pio_can_add_program(pio2, &psram_quad_program))
        return false;
    if (dma_channel_is_claimed(4) || dma_channel_is_claimed(5))
        return false;

    sPioOffset = pio_add_program(pio2, &psram_quad_program);
    pio_sm_claim(pio2, 0);

    pio_sm_config c = psram_quad_program_get_default_config((uint)sPioOffset);
    sm_config_set_out_pins(&c, PSRAM_PIN_IO0, 4);
    sm_config_set_in_pins(&c, PSRAM_PIN_IO0);
    sm_config_set_set_pins(&c, PSRAM_PIN_IO0, 4);
    sm_config_set_sideset_pins(&c, PSRAM_PIN_SCLK);
    sm_config_set_out_shift(&c, false, false, 32); // MSB first, autopull off
    sm_config_set_in_shift(&c, false, true, 32);   // MSB first, autopush on
    sm_config_set_clkdiv(&c, PSRAM_PIO_CLKDIV);

    pio_gpio_init(pio2, PSRAM_PIN_SCLK);
    for (uint p = PSRAM_PIN_IO0; p <= PSRAM_PIN_IO3; p++)
        pio_gpio_init(pio2, p);
    sMuxIsPio = true; // pio_gpio_init() put SCLK/IO on PIO2

    pio_sm_init(pio2, 0, (uint)sPioOffset, &c);
    // Bypass the 2-cycle input synchronizer so the read-data sample point is
    // the instruction itself rather than two SM cycles earlier; the extra
    // "nop side 1" in rd_data then provides a full half-cycle of settling.
    pio_set_input_sync_bypass_with_mask(pio2, PSRAM_IO_MASK, PSRAM_IO_MASK);
    pio_sm_set_pins_with_mask(pio2, 0, PSRAM_IO_MASK, PSRAM_IO_MASK);
    pio_sm_set_pindirs_with_mask(pio2, 0, PSRAM_IO_MASK | PSRAM_SCLK_MASK,
                                 PSRAM_IO_MASK | PSRAM_SCLK_MASK);
    pio_sm_set_pins_with_mask(pio2, 0, 0, PSRAM_SCLK_MASK);
    pio_sm_set_enabled(pio2, 0, false);

    dma_channel_claim(4);
    dma_channel_claim(5);
    sDmaTx = 4;
    sDmaRx = 5;

    sEngineReady = true;
    gPsramUsePio = true;
    if (sRuntimeState < PSRAM_STATE_RESOURCE_READY)
        sRuntimeState = PSRAM_STATE_RESOURCE_READY;
    return true;
}

psramRuntimeState psramGetRuntimeState(void) { return (psramRuntimeState)sRuntimeState; }

u8 psramChipReadyMask(void)
{
    u8 mask = 0;
    for (u32 c = 0; c < PSRAM_CHIP_COUNT; c++)
        if (gPsramChips[c].present && gPsramChips[c].idSizeBytes != 0)
            mask |= (u8)(1u << c);
    return mask;
}

bool psramInitDevice(void)
{
    if (sRuntimeState < PSRAM_STATE_RESOURCE_READY)
        return false; // the engine must exist before the devices are probed

    u8 mask = 0;
    for (u32 c = 0; c < PSRAM_CHIP_COUNT; c++)
    {
        psramResetChip(c);
        psramReadId(c, gPsramChips[c].id);
        u32 man = 0, kgd = 0;
        u32 size = psramDecodeIdSize(gPsramChips[c].id, &man, &kgd);
        gPsramChips[c].idValid = (size != 0);
        gPsramChips[c].idSizeBytes = size;
        gPsramChips[c].present = (size != 0);
        if (size != 0)
            mask |= (u8)(1u << c);
    }

    if (mask == (u8)((1u << PSRAM_CHIP_COUNT) - 1u) &&
        sRuntimeState < PSRAM_STATE_DEVICE_READY)
        sRuntimeState = PSRAM_STATE_DEVICE_READY;
    return sRuntimeState >= PSRAM_STATE_DEVICE_READY;
}

bool psramSelfTest(void)
{
    if (sRuntimeState < PSRAM_STATE_DEVICE_READY)
        return false;

    u8 out[PSRAM_SELFTEST_BYTES];
    u8 in[PSRAM_SELFTEST_BYTES];
    for (u32 c = 0; c < PSRAM_CHIP_COUNT; c++)
    {
        if (!gPsramChips[c].present)
            return false;
        for (u32 i = 0; i < PSRAM_SELFTEST_BYTES; i++)
            out[i] = (u8)(0xA5u ^ (u8)(c * 0x11u) ^ (u8)i);
        if (!psramWrite(c, PSRAM_SELFTEST_ADDR, out, PSRAM_SELFTEST_BYTES))
            return false;
        memset(in, 0, sizeof(in));
        if (!psramRead(c, PSRAM_SELFTEST_ADDR, in, PSRAM_SELFTEST_BYTES))
            return false;
        if (memcmp(in, out, PSRAM_SELFTEST_BYTES) != 0)
            return false;
    }

    if (sRuntimeState < PSRAM_STATE_SELFTEST_OK)
        sRuntimeState = PSRAM_STATE_SELFTEST_OK;
    return true;
}

#ifdef PSRAM_BOOT_AUTOSWEEP
bool psramCrossSelfTest(void)
{
    if (sRuntimeState < PSRAM_STATE_DEVICE_READY)
        return false;

    u8 pattern[PSRAM_SELFTEST_BYTES];
    u8 readback[PSRAM_SELFTEST_BYTES];
    for (u32 c = 0; c < PSRAM_CHIP_COUNT; c++)
    {
        if (!gPsramChips[c].present)
            return false;

        // Verify the PIO write with an independent SIO read.
        for (u32 i = 0; i < PSRAM_SELFTEST_BYTES; i++)
            pattern[i] = (u8)(0x5Au ^ (u8)(c * 0x13u) ^ (u8)(i * 7u));
        if (!psramWrite(c, PSRAM_SELFTEST_ADDR, pattern, sizeof(pattern)) ||
            !psramBitBangRead(c, PSRAM_SELFTEST_ADDR, readback, sizeof(readback)) ||
            memcmp(pattern, readback, sizeof(pattern)) != 0)
            return false;

        // Verify the PIO read after an independent SIO write.
        for (u32 i = 0; i < PSRAM_SELFTEST_BYTES; i++)
            pattern[i] = (u8)(0xC3u ^ (u8)(c * 0x29u) ^ (u8)(i * 11u));
        if (!psramBitBangWrite(c, PSRAM_SELFTEST_ADDR, pattern, sizeof(pattern)) ||
            !psramRead(c, PSRAM_SELFTEST_ADDR, readback, sizeof(readback)) ||
            memcmp(pattern, readback, sizeof(pattern)) != 0)
            return false;
    }
    return true;
}
#endif

#ifdef CACHE_PSRAM_BOOT_DIAG
static psramCacheWindowDiag psramCacheWindowCompare(const u8 source[512], bool sioWrite)
{
    // Address zero is where a cached sector zero would live. M2 never serves
    // or fills PSRAM, so this boot-only test cannot change game data.
    static u8 pioData[512] __attribute__((aligned(4)));
    static u8 sioData[512] __attribute__((aligned(4)));
    psramCacheWindowDiag d = {0};
    if (sRuntimeState < PSRAM_STATE_SELFTEST_OK || !source)
        return d;
    memset(pioData, 0, sizeof(pioData));
    memset(sioData, 0, sizeof(sioData));

    bool ok = true;
    for (u32 off = 0; off < 512u; off += PSRAM_PIO_FRAG_BYTES)
        if (!(sioWrite ? psramBitBangWrite(0, off, source + off, PSRAM_PIO_FRAG_BYTES)
                       : psramWrite(0, off, source + off, PSRAM_PIO_FRAG_BYTES)))
        { ok = false; break; }
    if (!ok) return d;
    d.flags |= 1u;

    ok = true;
    for (u32 off = 0; off < 512u; off += PSRAM_PIO_FRAG_BYTES)
        if (!psramRead(0, off, pioData + off, PSRAM_PIO_FRAG_BYTES))
        { ok = false; break; }
    if (ok) d.flags |= 2u;

    ok = psramBitBangRead(0, 0, sioData, sizeof(sioData));
    if (ok) d.flags |= 4u;

    if (d.flags & 2u)
    {
        d.pioOff = 512u;
        for (u32 i = 0; i < 512u; i++)
            if (pioData[i] != source[i]) { d.pioOff = i; break; }
        if (d.pioOff == 512u) d.flags |= 8u;
        else { d.expectedPio = source[d.pioOff]; d.actualPio = pioData[d.pioOff]; }
    }
    if (d.flags & 4u)
    {
        d.sioOff = 512u;
        for (u32 i = 0; i < 512u; i++)
            if (sioData[i] != source[i]) { d.sioOff = i; break; }
        if (d.sioOff == 512u) d.flags |= 16u;
        else { d.expectedSio = source[d.sioOff]; d.actualSio = sioData[d.sioOff]; }
    }
    return d;
}

psramCacheWindowDiag psramCacheWindowSelfTest(void)
{
    static u8 source[512] __attribute__((aligned(4)));
    for (u32 i = 0; i < sizeof(source); i++)
        source[i] = (u8)(0xA5u ^ (u8)(i * 37u) ^ (u8)(i >> 3));
    return psramCacheWindowCompare(source, false);
}

#ifdef CACHE_PSRAM_SECTOR0_DIAG
psramCacheWindowDiag psramCacheWindowDataTest(const u8 source[512])
{
    return psramCacheWindowCompare(source, false);
}

psramCacheWindowDiag psramCacheWindowSioWriteTest(const u8 source[512])
{
    return psramCacheWindowCompare(source, true);
}
#endif
#endif

void psramSetRuntimeEnabled(bool enabled)
{
    if (enabled)
    {
        if (sRuntimeState >= PSRAM_STATE_SELFTEST_OK)
            sRuntimeState = PSRAM_STATE_RUNTIME_ENABLED;
    }
    else
    {
        if (sRuntimeState == PSRAM_STATE_RUNTIME_ENABLED)
            sRuntimeState = PSRAM_STATE_SELFTEST_OK;
    }
}

void psramSetClockDiv(float div)
{
    if (!sEngineReady)
        return;
    pio_sm_set_enabled(pio2, 0, false);
    pio_sm_set_clkdiv(pio2, 0, div);
}

float psramGetClockDiv(void)
{
    if (!sEngineReady)
        return PSRAM_PIO_CLKDIV;
    // Derive the integer divider actually loaded in SM0.
    u32 intDiv = (pio2->sm[0].clkdiv >> 16) & 0xFFFFu;
    return intDiv ? (float)intDiv : 1.0f;
}

// Stop the SM, drop stale FIFO/shift state and park the PC on \p entryOffset.
static void pioPrepare(uint entryOffset)
{
    pio_sm_set_enabled(pio2, 0, false);
    pio_sm_clear_fifos(pio2, 0);
    pio_sm_restart(pio2, 0);
    // SCLK low and all four data lines high before the serial phase starts:
    // OUT with a 1-bit width then only changes SIO0, so SIO1..3 must already
    // be driven high (design section 2.6 / prior PIO proof).
    pio_sm_set_pins_with_mask(pio2, 0, PSRAM_IO_MASK, PSRAM_IO_MASK | PSRAM_SCLK_MASK);
    pio_sm_set_pindirs_with_mask(pio2, 0, PSRAM_IO_MASK | PSRAM_SCLK_MASK,
                                 PSRAM_IO_MASK | PSRAM_SCLK_MASK);
    pio_sm_exec(pio2, 0, pio_encode_jmp((uint)sPioOffset + entryOffset));
}

// Fault exits must restore a bus state from which the next chip can safely
// start. Do not assume DMA completion means the PIO has finished its clocks.
static void pioAbortTransfer(uint dmaChannel)
{
    pio_sm_set_enabled(pio2, 0, false);
    psramDeselectAll();
    muxToSio();
    sio_hw->gpio_oe_clr = PSRAM_IO_MASK;
    sio_hw->gpio_clr = PSRAM_SCLK_MASK;
    sio_hw->gpio_oe_set = PSRAM_SCLK_MASK;
    // Make the external bus idle before waiting for DMA cancellation. Even
    // if that wait stalls, no chip remains selected or clocked.
    dma_channel_abort(dmaChannel);
    pio_sm_clear_fifos(pio2, 0);
}

#ifdef PSRAM_QUAL_FAULT_TEST
void psramQualInjectFaultOnce(u8 fault) { sQualFaultOnce = fault; }

bool psramQualBusIdle(void)
{
    return (sio_hw->gpio_out & PSRAM_CE_MASK) == PSRAM_CE_MASK &&
           (sio_hw->gpio_in & PSRAM_CE_MASK) == PSRAM_CE_MASK &&
           (sio_hw->gpio_oe & PSRAM_CE_MASK) == PSRAM_CE_MASK &&
           !(sio_hw->gpio_out & PSRAM_SCLK_MASK) &&
           !(sio_hw->gpio_oe & PSRAM_IO_MASK) &&
           !sMuxIsPio && !(pio2->ctrl & 1u) &&
           !dma_channel_is_busy((uint)sDmaTx) &&
           !dma_channel_is_busy((uint)sDmaRx);
}
#endif

static bool pioReadFragment(u32 chip, u32 addr, u8* buf, u32 len)
{
    // The DMA moves whole 32-bit words, so a caller buffer that is unaligned
    // or a length that is not a multiple of four goes through a bounce.
    static u32 rxBounce[PSRAM_PIO_FRAG_BYTES / 4];
    bool bounce = (((uintptr_t)buf & 3u) != 0u) || ((len & 3u) != 0u);
    void* dst = bounce ? (void*)rxBounce : (void*)buf;

    muxToPio();
    pioPrepare((uint)psram_quad_offset_psram_quad_read);

    pio_sm_put(pio2, 0, ((u32)PSRAM_CMD_FAST_READ_QUAD << 24) | (addr & 0x00FFFFFFu));

    dma_channel_config rx = dma_channel_get_default_config((uint)sDmaRx);
    channel_config_set_transfer_data_size(&rx, DMA_SIZE_32);
    channel_config_set_read_increment(&rx, false);
    channel_config_set_write_increment(&rx, true);
    // The PIO assembles the MSB-first serial bitstream into ISR bits 31:0, so
    // the first byte of the PSRAM stream lands in the word's most significant
    // byte. Byte-swap the 32-bit write so memory[0] is the first PSRAM byte.
    channel_config_set_bswap(&rx, true);
    channel_config_set_dreq(&rx, pio_get_dreq(pio2, 0, false));
    dma_channel_configure((uint)sDmaRx, &rx, dst, &pio2->rxf[0], (len + 3u) / 4u, false);

    psramSelectChip(chip);
    dma_channel_start((uint)sDmaRx);
#ifdef PSRAM_QUAL_FAULT_TEST
    bool forceStall = sQualFaultOnce == PSRAM_QUAL_FAULT_RX_STALL;
    if (forceStall)
        sQualFaultOnce = 0;
    if (!forceStall)
#endif
    pio_sm_set_enabled(pio2, 0, true);

    u32 spin = 0;
    while (dma_channel_is_busy((uint)sDmaRx))
    {
        if (++spin > PSRAM_XFER_SPIN_LIMIT)
        {
            // Bounded failure: never leave the engine enabled or CE# low.
            pioAbortTransfer((uint)sDmaRx);
            return false;
        }
        tight_loop_contents();
    }

    pio_sm_set_enabled(pio2, 0, false);
    dma_channel_abort((uint)sDmaRx);
    psramDeselectAll();
    pio_sm_clear_fifos(pio2, 0);
    if (bounce)
        memcpy(buf, rxBounce, len);
    return true;
}

static bool pioWriteFragment(u32 chip, u32 addr, const u8* buf, u32 len)
{
    // One command/address word plus one word per four data bytes. Fragments
    // are capped well below this, but keep the guard explicit.
    static u32 txWords[1 + PSRAM_PIO_FRAG_BYTES / 4];
    u32 nWords = 1 + (len + 3u) / 4u;
    if (nWords > sizeof(txWords) / sizeof(txWords[0]))
        return false;

    txWords[0] = ((u32)PSRAM_CMD_QUAD_WRITE << 24) | (addr & 0x00FFFFFFu);
    for (u32 off = 0; off < len; off += 4)
    {
        u32 w = 0;
        for (u32 k = 0; k < 4 && (off + k) < len; k++)
            w |= (u32)buf[off + k] << (24 - 8 * k);
        txWords[1 + off / 4] = w;
    }

    muxToPio();
    pioPrepare((uint)psram_quad_offset_psram_quad_write);

    dma_channel_config tx = dma_channel_get_default_config((uint)sDmaTx);
    channel_config_set_transfer_data_size(&tx, DMA_SIZE_32);
    channel_config_set_read_increment(&tx, true);
    channel_config_set_write_increment(&tx, false);
    channel_config_set_dreq(&tx, pio_get_dreq(pio2, 0, true));
    dma_channel_configure((uint)sDmaTx, &tx, &pio2->txf[0], txWords, nWords, false);

    psramSelectChip(chip);
    dma_channel_start((uint)sDmaTx);
#ifdef PSRAM_QUAL_FAULT_TEST
    u8 injected = sQualFaultOnce;
    if (injected == PSRAM_QUAL_FAULT_TX_STALL ||
        injected == PSRAM_QUAL_FAULT_TX_AFTER_DMA)
        sQualFaultOnce = 0;
    if (injected != PSRAM_QUAL_FAULT_TX_STALL)
#endif
    pio_sm_set_enabled(pio2, 0, true);

    u32 spin = 0;
    while (dma_channel_is_busy((uint)sDmaTx))
    {
        if (++spin > PSRAM_XFER_SPIN_LIMIT)
        {
            pioAbortTransfer((uint)sDmaTx);
            return false;
        }
        tight_loop_contents();
    }
#ifdef PSRAM_QUAL_FAULT_TEST
    if (injected == PSRAM_QUAL_FAULT_TX_AFTER_DMA)
    {
        pioAbortTransfer((uint)sDmaTx);
        return false;
    }
#endif
    // DMA finished when the last word reached the FIFO; the PIO may still be
    // shifting it. Wait until the SM blocks on the write-data PULL with an
    // empty TX FIFO, which is the exact end of the last quad clock.
    spin = 0;
    while (!(pio_sm_is_tx_fifo_empty(pio2, 0) &&
             pio_sm_get_pc(pio2, 0) ==
                 (uint8_t)((sPioOffset + psram_quad_offset_psram_quad_write_idle) & 0x1F)))
    {
        if (++spin > PSRAM_XFER_SPIN_LIMIT)
        {
            pioAbortTransfer((uint)sDmaTx);
            return false;
        }
        tight_loop_contents();
    }

    pio_sm_set_enabled(pio2, 0, false);
    dma_channel_abort((uint)sDmaTx);
    psramDeselectAll();
    pio_sm_clear_fifos(pio2, 0);
    return true;
}

// ---------------------------------------------------------------------------
// Bit-bang quad fragments
// ---------------------------------------------------------------------------

static void bbQuadReadFragment(u32 chip, u32 addr, u8* buf, u32 len)
{
    muxToSio();
    sio_hw->gpio_oe_clr = PSRAM_IO_MASK;
    bbClkLow();

    psramSelectChip(chip);
    sio_hw->gpio_oe_set = 1u << PSRAM_PIN_IO0;
    bbSpiSendByte(PSRAM_CMD_FAST_READ_QUAD);

    sio_hw->gpio_oe_set = PSRAM_IO_MASK;
    for (int s = 20; s >= 0; s -= 4)
        bbSendNibble((u8)((addr >> s) & 0xFu));

    // Release the bus before the six dummy clocks; the device starts driving
    // after them.
    bbClkLow();
    sio_hw->gpio_oe_clr = PSRAM_IO_MASK;
    for (int i = 0; i < PSRAM_RX_DUMMY_NIBBLES; i++)
    {
        bbClkHigh();
        bbClkLow();
    }

    bbReadData(buf, len);
    psramDeselectAll();
    sio_hw->gpio_oe_clr = PSRAM_IO_MASK;
    bbClkLow();
}

static void bbQuadWriteFragment(u32 chip, u32 addr, const u8* buf, u32 len)
{
    muxToSio();
    sio_hw->gpio_oe_clr = PSRAM_IO_MASK;
    bbClkLow();

    psramSelectChip(chip);
    sio_hw->gpio_oe_set = 1u << PSRAM_PIN_IO0;
    bbSpiSendByte(PSRAM_CMD_QUAD_WRITE);

    sio_hw->gpio_oe_set = PSRAM_IO_MASK;
    for (int s = 20; s >= 0; s -= 4)
        bbSendNibble((u8)((addr >> s) & 0xFu));

    for (u32 i = 0; i < len; i++)
    {
        bbSendNibble((u8)(buf[i] >> 4));
        bbSendNibble((u8)(buf[i] & 0xFu));
    }

    psramDeselectAll();
    sio_hw->gpio_oe_clr = PSRAM_IO_MASK;
    bbClkLow();
}

// ---------------------------------------------------------------------------
// Public data API with fragmenting
// ---------------------------------------------------------------------------

static bool psramAccess(u32 chip, u32 addr, void* buf, u32 len, bool write, bool forceBitBang)
{
    if (chip >= PSRAM_CHIP_COUNT)
        return false;
    if (addr > PSRAM_SIZE_BYTES || len > PSRAM_SIZE_BYTES - addr)
        return false;

    u8* p = (u8*)buf;
    while (len > 0)
    {
        bool usePio = !forceBitBang && gPsramUsePio && sEngineReady;
        u32 fragSize = usePio ? PSRAM_PIO_FRAG_BYTES : PSRAM_BB_FRAG_BYTES;
        u32 frag = fragSize - (addr & (fragSize - 1u));
        if (frag > len)
            frag = len;

        // The PIO engine transfers whole 32-bit words. An incomplete word
        // would leave a read DMA waiting for an autopush, or write padding
        // bytes beyond the requested range. Transfer complete words with PIO
        // and leave fewer than four bytes for a bounded SIO transaction.
        if (usePio && (frag & 3u) != 0u)
        {
            u32 whole = frag & ~3u;
            if (whole > 0)
                frag = whole;
            else
                usePio = false;
        }
        if (!usePio)
        {
            if (write)
                bbQuadWriteFragment(chip, addr, p, frag);
            else
                bbQuadReadFragment(chip, addr, p, frag);
        }
        else
        {
            if (write)
            {
                if (!pioWriteFragment(chip, addr, p, frag))
                    return false;
            }
            else
            {
                if (!pioReadFragment(chip, addr, p, frag))
                    return false;
            }
        }

        addr += frag;
        p += frag;
        len -= frag;
    }
    return true;
}

bool psramRead(u32 chip, u32 addr, void* buf, u32 len)
{
    return psramAccess(chip, addr, buf, len, false, false);
}

bool psramWrite(u32 chip, u32 addr, const void* buf, u32 len)
{
    return psramAccess(chip, addr, (void*)buf, len, true, false);
}

bool psramBitBangRead(u32 chip, u32 addr, void* buf, u32 len)
{
    return psramAccess(chip, addr, buf, len, false, true);
}

bool psramBitBangWrite(u32 chip, u32 addr, const void* buf, u32 len)
{
    return psramAccess(chip, addr, (void*)buf, len, true, true);
}
