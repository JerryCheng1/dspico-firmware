#include "common.h"
#include <stdio.h>
#include <string.h>
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/sync.h"
#include "hardware/structs/sio.h"
#include "psram.h"
#include "psram.pio.h"

// APS6404L command set (see APS6404L_3SQR datasheet).
// In SPI mode (QE=0) the command phase is always serial; address and data
// phases of the quad commands are 4-bit. No mode switch is required.
#define PSRAM_CMD_RESET_ENABLE  0x66
#define PSRAM_CMD_RESET         0x99
#define PSRAM_CMD_FAST_READ_QUAD 0xEB // 6 dummy cycles
#define PSRAM_CMD_QUAD_WRITE    0x38

// The datasheet limits CE# low time to tCEM (max 8 us); longer transfers pause
// the internal DRAM refresh and can corrupt data. All transfers are therefore
// split into bursts that complete well within tCEM, including margin for
// interruptions by the cartridge bus IRQ. Bursts are kept aligned to their
// size so they never cross a 1K PSRAM page boundary.
// At clkdiv 3 a 128-byte PIO burst alone takes ~8 us (the RX pump moves one
// nibble per 12 sysclk), leaving zero IRQ margin - keep PIO bursts at 32 B
// (~2-4 us) like the bit-bang path.
#define PSRAM_PIO_BURST_BYTES   32
#define PSRAM_BB_BURST_BYTES    32

#define PSRAM_RX_DUMMY_NIBBLES 6

#define PSRAM_PIO       pio0

// pio0 SM0 is the cartridge emulator (ntr_card), SM1 the WRFUXXED SPI-UART;
// the PSRAM pump takes SM2/SM3.
#define PSRAM_TX_SM     2
#define PSRAM_RX_SM     3

#define PSRAM_IO_MASK   (0xFu << PSRAM_PIN_IO0)
#define PSRAM_IO0_MASK  (1u << PSRAM_PIN_IO0)
#define PSRAM_CLK_MASK  (1u << PSRAM_PIN_CLK)
#define PSRAM_CE_MASK   (1u << PSRAM_PIN_CE)

// When true, the address/data phases are streamed by two pio0 state machines
// (psram_qspi_tx/psram_qspi_rx, clkdiv 3 -> SCLK <= sysclk / 6). Falls back to
// bit-banging when the pio0 resources are unavailable (e.g. WRFUXXED builds).
static bool sUsePio;
static uint sTxSm;
static uint sRxSm;

static inline void psramCeLow(void)
{
    sio_hw->gpio_clr = PSRAM_CE_MASK;
}

static inline void psramCeHigh(void)
{
    sio_hw->gpio_set = PSRAM_CE_MASK;
}

static inline void psramClkLow(void)
{
    sio_hw->gpio_clr = PSRAM_CLK_MASK;
}

static inline void psramClkHigh(void)
{
    sio_hw->gpio_set = PSRAM_CLK_MASK;
}

static void psramMuxToSio(void)
{
    gpio_set_function(PSRAM_PIN_IO0, GPIO_FUNC_SIO);
    gpio_set_function(PSRAM_PIN_IO1, GPIO_FUNC_SIO);
    gpio_set_function(PSRAM_PIN_IO2, GPIO_FUNC_SIO);
    gpio_set_function(PSRAM_PIN_IO3, GPIO_FUNC_SIO);
    gpio_set_function(PSRAM_PIN_CLK, GPIO_FUNC_SIO);
}

static void psramMuxToPio(void)
{
    gpio_set_function(PSRAM_PIN_IO0, GPIO_FUNC_PIO0);
    gpio_set_function(PSRAM_PIN_IO1, GPIO_FUNC_PIO0);
    gpio_set_function(PSRAM_PIN_IO2, GPIO_FUNC_PIO0);
    gpio_set_function(PSRAM_PIN_IO3, GPIO_FUNC_PIO0);
    gpio_set_function(PSRAM_PIN_CLK, GPIO_FUNC_PIO0);
}

// Sends one byte serially on IO0 (the command phase is always serial).
// Leaves the clock low and IO0 released. Requires the pins muxed to SIO.
static void psramSendByteSerial(u8 b)
{
    sio_hw->gpio_oe_set = PSRAM_IO0_MASK;
    for (int i = 0; i < 8; i++)
    {
        psramClkLow();
        if (b & 0x80)
            sio_hw->gpio_set = PSRAM_IO0_MASK;
        else
            sio_hw->gpio_clr = PSRAM_IO0_MASK;
        b <<= 1;
        psramClkHigh();
    }
    psramClkLow();
    sio_hw->gpio_oe_clr = PSRAM_IO0_MASK;
}

// ---------------------------------------------------------------------------
// PIO data pump
// ---------------------------------------------------------------------------

static bool psramPioInit(void)
{
    int txOffset = pio_add_program(PSRAM_PIO, &psram_qspi_tx_program);
    if (txOffset < 0)
        return false;
    int rxOffset = pio_add_program(PSRAM_PIO, &psram_qspi_rx_program);
    if (rxOffset < 0)
        return false;

    // The cartridge emulator hardcodes pio0 SM0 (and SM1 in WRFUXXED builds)
    // WITHOUT registering them with the SDK claim mechanism. Claiming "any
    // unused SM" would therefore hand the PSRAM pump SM0/SM1 and clobber the
    // cartridge protocol (observed on hardware: NDS boots the cart, then the
    // file API dies the moment the PSRAM is initialized). Claim SM2/SM3
    // explicitly instead.
    if (pio_sm_is_claimed(PSRAM_PIO, PSRAM_TX_SM) || pio_sm_is_claimed(PSRAM_PIO, PSRAM_RX_SM))
        return false;
    pio_sm_claim(PSRAM_PIO, PSRAM_TX_SM);
    pio_sm_claim(PSRAM_PIO, PSRAM_RX_SM);
    sTxSm = PSRAM_TX_SM;
    sRxSm = PSRAM_RX_SM;

    // TX pump: nibbles from the TX FIFO to IO0-IO3, SCLK on side-set.
    // clkdiv 3: one nibble takes 2 sm cycles, so SCLK = sysclk / 6 (~33 MHz).
    pio_sm_config c = psram_qspi_tx_program_get_default_config((uint)txOffset);
    sm_config_set_out_pins(&c, PSRAM_PIN_IO0, 4);
    sm_config_set_in_pins(&c, PSRAM_PIN_IO0);
    sm_config_set_set_pins(&c, PSRAM_PIN_IO0, 4);
    sm_config_set_sideset_pins(&c, PSRAM_PIN_CLK);
    sm_config_set_clkdiv(&c, 3.0f);
    sm_config_set_out_shift(&c, false, true, 32); // MSB (top nibble) first, autopull
    pio_sm_init(PSRAM_PIO, sTxSm, (uint)txOffset, &c);
    pio_sm_set_pindirs_with_mask(PSRAM_PIO, sTxSm, 0, PSRAM_IO_MASK);
    // side-set can only drive pins whose direction is output for this SM;
    // without this SCLK never toggles (pio_sm_set_pins only sets the level)
    pio_sm_set_pindirs_with_mask(PSRAM_PIO, sTxSm, PSRAM_CLK_MASK, PSRAM_CLK_MASK);
    pio_sm_set_pins_with_mask(PSRAM_PIO, sTxSm, 0, PSRAM_CLK_MASK);

    // RX pump: nibbles from IO0-IO3 to the RX FIFO. One nibble takes 4 sm
    // cycles, so with clkdiv 3 SCLK = sysclk / 12 (~17 MHz). No nibble
    // counting in the program; the CPU reads exactly len / 4 FIFO words.
    c = psram_qspi_rx_program_get_default_config((uint)rxOffset);
    sm_config_set_in_pins(&c, PSRAM_PIN_IO0);
    sm_config_set_sideset_pins(&c, PSRAM_PIN_CLK);
    sm_config_set_clkdiv(&c, 3.0f);
    sm_config_set_in_shift(&c, false, true, 32); // first nibble lands at the top, autopush
    pio_sm_init(PSRAM_PIO, sRxSm, (uint)rxOffset, &c);
    pio_sm_set_pindirs_with_mask(PSRAM_PIO, sRxSm, 0, PSRAM_IO_MASK);
    pio_sm_set_pindirs_with_mask(PSRAM_PIO, sRxSm, PSRAM_CLK_MASK, PSRAM_CLK_MASK);
    pio_sm_set_pins_with_mask(PSRAM_PIO, sRxSm, 0, PSRAM_CLK_MASK);

    return true;
}

// Starts the TX pump. The first FIFO word is the nibble count - 1, which the
// program pulls into x itself; the second word is the first stream word. The
// stream is MSB-nibble first; every 32-bit word holds 8 nibbles.
static inline void psramTxSmStart(u32 nibbleCount, u32 firstWord)
{
    pio_sm_restart(PSRAM_PIO, sTxSm);
    pio_sm_clear_fifos(PSRAM_PIO, sTxSm);
    pio_sm_put(PSRAM_PIO, sTxSm, nibbleCount - 1);
    pio_sm_put(PSRAM_PIO, sTxSm, firstWord);
    pio_sm_set_pindirs_with_mask(PSRAM_PIO, sTxSm, PSRAM_IO_MASK, PSRAM_IO_MASK);
    pio_sm_set_enabled(PSRAM_PIO, sTxSm, true);
}

// Waits for the TX pump to finish (TX FIFO drained, then the remaining
// in-flight OSR bits shift out) and releases the data bus.
static inline void psramTxSmWait(void)
{
    while (pio_sm_get_tx_fifo_level(PSRAM_PIO, sTxSm) != 0);
    // at most one OSR word (8 nibbles, 16 sm cycles) is still in flight
    for (int i = 0; i < 64; i++)
        __asm volatile ("nop");
    pio_sm_set_enabled(PSRAM_PIO, sTxSm, false);
    pio_sm_set_pindirs_with_mask(PSRAM_PIO, sTxSm, 0, PSRAM_IO_MASK);
}

static void __no_inline_not_in_flash_func(psramPioWriteBurst)(u32 addr, const u8* buf, u32 len)
{
    // The cartridge bus IRQ handlers (resetNtrCard on the reset pin,
    // ntrc_pioIrq on sm0) do their own read-modify-writes of pio0 registers.
    // Racing them here can lose an enable/restart on either side, which kills
    // the burst mid-stream. A burst takes only a few microseconds, so keep it
    // atomic.
    uint32_t irqState = save_and_disable_interrupts();
    psramMuxToSio();
    psramClkLow();
    psramCeLow();
    psramSendByteSerial(PSRAM_CMD_QUAD_WRITE);

    psramMuxToPio();
    // The 24-bit address takes the top 6 nibbles of the first stream word;
    // the first data byte fills its bottom 2 nibbles so the nibble stream is
    // gapless. Later words carry 4 data bytes each, big-endian.
    psramTxSmStart(6 + 2 * len, (addr << 8) | buf[0]);
    for (u32 off = 1; off < len; off += 4)
    {
        u32 w = 0;
        for (u32 k = 0; k < 4 && off + k < len; k++)
            w |= (u32)buf[off + k] << (24 - 8 * k);
        pio_sm_put_blocking(PSRAM_PIO, sTxSm, w);
    }
    psramTxSmWait();

    psramMuxToSio();
    psramCeHigh();
    restore_interrupts(irqState);
}

static void __no_inline_not_in_flash_func(psramPioReadBurst)(u32 addr, u8* buf, u32 len)
{
    // See psramPioWriteBurst for why this runs with interrupts disabled.
    uint32_t irqState = save_and_disable_interrupts();
    psramMuxToSio();
    psramClkLow();
    psramCeLow();
    psramSendByteSerial(PSRAM_CMD_FAST_READ_QUAD);

    // address + dummy clocks via the TX pump. The address word's 2 spare
    // bottom nibbles are zeros and serve as the first 2 dummy clocks; a zero
    // word provides the remaining 4.
    psramMuxToPio();
    psramTxSmStart(6 + PSRAM_RX_DUMMY_NIBBLES, addr << 8);
    pio_sm_put_blocking(PSRAM_PIO, sTxSm, 0);
    psramTxSmWait();

    // data via the RX pump; the FIFO words are big-endian, hence the bswap.
    // The program free-runs (no nibble count): read exactly len / 4 words,
    // then disable the SM.
    pio_sm_restart(PSRAM_PIO, sRxSm);
    pio_sm_clear_fifos(PSRAM_PIO, sRxSm);
    pio_sm_set_enabled(PSRAM_PIO, sRxSm, true);
    u32* w = (u32*)buf;
    for (u32 i = 0; i < len / 4; i++)
    {
        w[i] = __builtin_bswap32(pio_sm_get_blocking(PSRAM_PIO, sRxSm));
    }
    pio_sm_set_enabled(PSRAM_PIO, sRxSm, false);

    psramMuxToSio();
    psramCeHigh();
    restore_interrupts(irqState);
}

// ---------------------------------------------------------------------------
// Bit-bang fallback data path
// ---------------------------------------------------------------------------

static inline void psramBbSendNibble(u8 nibble)
{
    psramClkLow();
    // NOTE: do NOT use hw_write_masked() on SIO registers - it writes through
    // the 0x1000 XOR alias, which the SIO block does not implement (its bus has
    // no alias decoding), so the write is silently dropped and the pins never
    // change. Plain read-modify-write of GPIO_OUT is safe here.
    sio_hw->gpio_out = (sio_hw->gpio_out & ~PSRAM_IO_MASK) | ((u32)nibble << PSRAM_PIN_IO0);
    psramClkHigh();
}

static inline u8 psramBbRecvNibble(void)
{
    psramClkHigh();
    __asm volatile ("nop\n nop\n nop\n nop");
    u8 nibble = (sio_hw->gpio_in >> PSRAM_PIN_IO0) & 0xF;
    psramClkLow();
    return nibble;
}

static void __no_inline_not_in_flash_func(psramBbReadBurst)(u32 addr, u8* buf, u32 len)
{
    psramClkLow();
    psramCeLow();
    psramSendByteSerial(PSRAM_CMD_FAST_READ_QUAD);

    // 24-bit address, 6 quad clocks
    sio_hw->gpio_oe_set = PSRAM_IO_MASK;
    for (int s = 20; s >= 0; s -= 4)
        psramBbSendNibble((addr >> s) & 0xF);

    // 6 dummy clocks, IOs released
    // psramBbSendNibble leaves SCLK high; pull it low first so the loop below
    // produces a real rising edge on every dummy clock (otherwise the chip
    // sees only 5 dummy clocks and all read data shifts by one nibble).
    psramClkLow();
    sio_hw->gpio_oe_clr = PSRAM_IO_MASK;
    for (int i = 0; i < PSRAM_RX_DUMMY_NIBBLES; i++)
    {
        psramClkHigh();
        psramClkLow();
    }

    for (u32 i = 0; i < len; i++)
    {
        u8 hi = psramBbRecvNibble();
        u8 lo = psramBbRecvNibble();
        buf[i] = (hi << 4) | lo;
    }
    psramCeHigh();
}

static void __no_inline_not_in_flash_func(psramBbWriteBurst)(u32 addr, const u8* buf, u32 len)
{
    psramClkLow();
    psramCeLow();
    psramSendByteSerial(PSRAM_CMD_QUAD_WRITE);

    // 24-bit address, 6 quad clocks
    sio_hw->gpio_oe_set = PSRAM_IO_MASK;
    for (int s = 20; s >= 0; s -= 4)
        psramBbSendNibble((addr >> s) & 0xF);

    for (u32 i = 0; i < len; i++)
    {
        psramBbSendNibble(buf[i] >> 4);
        psramBbSendNibble(buf[i] & 0xF);
    }
    psramCeHigh();

    // release the data bus
    sio_hw->gpio_oe_clr = PSRAM_IO_MASK;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void psram_read(u32 addr, void* buf, u32 len)
{
    // The PIO path streams whole 32-bit words, so address and length must be
    // word aligned; anything else goes through the bit-bang path.
    bool usePio = sUsePio && ((addr | len) & 3) == 0;
    u8* dst = (u8*)buf;
    while (len > 0)
    {
        u32 burstBytes = usePio ? PSRAM_PIO_BURST_BYTES : PSRAM_BB_BURST_BYTES;
        u32 burst = burstBytes - (addr & (burstBytes - 1));
        if (burst > len)
            burst = len;

        if (usePio)
            psramPioReadBurst(addr, dst, burst);
        else
            psramBbReadBurst(addr, dst, burst);

        addr += burst;
        dst += burst;
        len -= burst;
    }
}

void psram_write(u32 addr, const void* buf, u32 len)
{
    bool usePio = sUsePio && ((addr | len) & 3) == 0;
    const u8* src = (const u8*)buf;
    while (len > 0)
    {
        u32 burstBytes = usePio ? PSRAM_PIO_BURST_BYTES : PSRAM_BB_BURST_BYTES;
        u32 burst = burstBytes - (addr & (burstBytes - 1));
        if (burst > len)
            burst = len;

        if (usePio)
            psramPioWriteBurst(addr, src, burst);
        else
            psramBbWriteBurst(addr, src, burst);

        addr += burst;
        src += burst;
        len -= burst;
    }
}

static void psramSendCmd(u8 cmd)
{
    psramClkLow();
    psramCeLow();
    psramSendByteSerial(cmd);
    psramCeHigh();
}

// Quick presence probe: write address-dependent patterns at the start,
// middle and end of the address space and read them back. Fails fast when
// no PSRAM is fitted, in which case the caller must not use the PSRAM.
// The exhaustive full-chip test runs in the background from
// romCacheUpdate() - it must not block boot: the cartridge protocol is
// served from the main loop, and the console gives up on the cart within
// a fraction of a second.
static bool psramProbe(void)
{
    for (u32 addr = 0; addr < PSRAM_SIZE_BYTES; addr += (PSRAM_SIZE_BYTES / 2) - PSRAM_PIO_BURST_BYTES)
    {
        u8 pattern[PSRAM_PIO_BURST_BYTES];
        u8 readBack[PSRAM_PIO_BURST_BYTES];
        for (u32 i = 0; i < sizeof(pattern); i++)
            pattern[i] = (u8)(addr + i * 0x9Du + 0x35u);

        // Retry a few times: the console boots in parallel and its reset
        // pulses can disturb a transfer even with the IRQ shielding.
        bool ok = false;
        for (int attempt = 0; attempt < 3 && !ok; attempt++)
        {
            psram_write(addr, pattern, sizeof(pattern));
            psram_read(addr, readBack, sizeof(readBack));
            ok = memcmp(pattern, readBack, sizeof(pattern)) == 0;
        }
        if (!ok)
        {
            LOG("PSRAM: probe FAILED @0x%08lX wrote[0:4]=", addr);
            for (u32 i = 0; i < 4; i++) LOG("%02X", pattern[i]);
            LOG(" read[0:4]=");
            for (u32 i = 0; i < 4; i++) LOG("%02X", readBack[i]);
            LOG("\n");
            if (sUsePio)
            {
                // Cross-check with the known-good bit-bang path, same as the
                // dspico-debug harness: pioW->bbR shows what the PIO write
                // actually stored, bbW->pioR isolates the PIO read pump.
                u8 xbuf[8];
                psramBbReadBurst(addr, xbuf, 8);
                LOG("PSRAM: pioW->bbR[0:8]=");
                for (u32 i = 0; i < 8; i++) LOG("%02X", xbuf[i]);
                LOG("\n");
                psramBbWriteBurst(addr, pattern, 8);
                memset(xbuf, 0, 8);
                psramPioReadBurst(addr, xbuf, 8);
                LOG("PSRAM: bbW->pioR[0:8]=");
                for (u32 i = 0; i < 8; i++) LOG("%02X", xbuf[i]);
                LOG("\n");
            }
            return false;
        }
    }
    return true;
}

bool psram_init(void)
{
    gpio_init_mask(PSRAM_PIN_MASK);

    // CE# and CLK are outputs, the data pins are inputs until a transfer starts.
    gpio_put(PSRAM_PIN_CE, true);
    gpio_put(PSRAM_PIN_CLK, false);
    gpio_set_dir_out_masked(PSRAM_CE_MASK | PSRAM_CLK_MASK);

    // Fast edges for the high PIO clock rate.
    for (uint pin = PSRAM_PIN_IO0; pin <= PSRAM_PIN_CLK; pin++)
    {
        gpio_set_slew_rate(pin, GPIO_SLEW_RATE_FAST);
        gpio_set_drive_strength(pin, GPIO_DRIVE_STRENGTH_8MA);
    }

    // The device needs 150 us after power-up before it accepts commands.
    sleep_us(200);

    // Software reset (RSTEN must be immediately followed by RST).
    psramSendCmd(PSRAM_CMD_RESET_ENABLE);
    psramSendCmd(PSRAM_CMD_RESET);
    sleep_us(50);

    sUsePio = psramPioInit();
    LOG("PSRAM: probing (%s path)...\n", sUsePio ? "PIO" : "bit-bang");
    if (psramProbe())
    {
        LOG("PSRAM: %s data path\n", sUsePio ? "PIO" : "bit-bang");
        return true;
    }

    // The PIO pump failing where bit-banging works points at a pump/timing
    // bug, not a missing chip - retry with bit-bang so the cache still works.
    if (sUsePio)
    {
        LOG("PSRAM: PIO probe failed, retrying with bit-bang...\n");
        pio_sm_set_enabled(PSRAM_PIO, sTxSm, false);
        pio_sm_set_enabled(PSRAM_PIO, sRxSm, false);
        sUsePio = false;
        psramMuxToSio();
        if (psramProbe())
        {
            LOG("PSRAM: bit-bang data path\n");
            return true;
        }
    }
    return false;
}
