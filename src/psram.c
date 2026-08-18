#include "common.h"
#include <stdio.h>
#include <string.h>
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/structs/sio.h"
#include "psram.h"
#include "psram.pio.h"

// APS6404L command set (see APS6404L_3SQR datasheet).
// In SPI mode (QE=0) the command phase is always serial; address and data
// phases of the quad commands are 4-bit. No mode switch is required.
#define PSRAM_CMD_RESET_ENABLE  0x66
#define PSRAM_CMD_RESET         0x99
#define PSRAM_CMD_ENTER_QUAD    0x35 // EQIO: set QE; harmless if already in quad mode
#define PSRAM_CMD_FAST_READ_QUAD 0xEB // 6 dummy cycles
#define PSRAM_CMD_QUAD_WRITE    0x38

// The datasheet limits CE# low time to tCEM (max 8 us); longer transfers pause
// the internal DRAM refresh and can corrupt data. All transfers are therefore
// split into bursts that complete well within tCEM, including margin for
// interruptions by the cartridge bus IRQ. Bursts are kept aligned to their
// size so they never cross a 1K PSRAM page boundary.
// The PIO pump is much faster than bit-banging, so it can afford bigger bursts.
#define PSRAM_PIO_BURST_BYTES   128
#define PSRAM_BB_BURST_BYTES    32

#define PSRAM_RX_DUMMY_NIBBLES 6

#define PSRAM_PIO       pio0

#define PSRAM_IO_MASK   (0xFu << PSRAM_PIN_IO0)
#define PSRAM_IO0_MASK  (1u << PSRAM_PIN_IO0)
#define PSRAM_CLK_MASK  (1u << PSRAM_PIN_CLK)
#define PSRAM_CE_MASK(chip)  (1u << (PSRAM_PIN_CE0 + (chip)))

// When true, the address/data phases are streamed by two pio0 state machines
// (psram_qspi_tx/psram_qspi_rx, clkdiv 3 -> SCLK <= sysclk / 6). Falls back to
// bit-banging when the pio0 resources are unavailable (e.g. WRFUXXED builds).
static bool sUsePio;
static uint sTxSm;
static uint sRxSm;
static uint sTxOffset, sRxOffset; // kept for diagnostics

static inline void psramCeLow(uint chip)
{
    sio_hw->gpio_clr = PSRAM_CE_MASK(chip);
}

static inline void psramCeHigh(uint chip)
{
    sio_hw->gpio_set = PSRAM_CE_MASK(chip);
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
    gpio_set_function(PSRAM_PIN_CLK, GPIO_FUNC_SIO);
    gpio_set_function(PSRAM_PIN_IO0, GPIO_FUNC_SIO);
    gpio_set_function(PSRAM_PIN_IO1, GPIO_FUNC_SIO);
    gpio_set_function(PSRAM_PIN_IO2, GPIO_FUNC_SIO);
    gpio_set_function(PSRAM_PIN_IO3, GPIO_FUNC_SIO);
}

static void psramMuxToPio(void)
{
    gpio_set_function(PSRAM_PIN_CLK, GPIO_FUNC_PIO0);
    gpio_set_function(PSRAM_PIN_IO0, GPIO_FUNC_PIO0);
    gpio_set_function(PSRAM_PIN_IO1, GPIO_FUNC_PIO0);
    gpio_set_function(PSRAM_PIN_IO2, GPIO_FUNC_PIO0);
    gpio_set_function(PSRAM_PIN_IO3, GPIO_FUNC_PIO0);
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
    int txSm = pio_claim_unused_sm(PSRAM_PIO, false);
    if (txSm < 0)
        return false;
    int rxSm = pio_claim_unused_sm(PSRAM_PIO, false);
    if (rxSm < 0)
        return false;

    sTxSm = (uint)txSm;
    sRxSm = (uint)rxSm;
    sTxOffset = (uint)txOffset;
    sRxOffset = (uint)rxOffset;

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
    // cycles, so with clkdiv 3 SCLK = sysclk / 12 (~17 MHz).
    c = psram_qspi_rx_program_get_default_config((uint)rxOffset);
    sm_config_set_in_pins(&c, PSRAM_PIN_IO0);
    sm_config_set_sideset_pins(&c, PSRAM_PIN_CLK);
    sm_config_set_clkdiv(&c, 3.0f);
    sm_config_set_in_shift(&c, false, true, 32); // first nibble lands at the top, autopush
    sm_config_set_out_shift(&c, false, false, 32); // no autopull: the count pull is explicit
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
    const uint chip = PSRAM_CHIP_OF(addr);
    addr = PSRAM_ADDR_IN_CHIP(addr);

    psramMuxToSio();
    psramClkLow();
    psramCeLow(chip);
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
    psramCeHigh(chip);
}

static void __no_inline_not_in_flash_func(psramPioReadBurst)(u32 addr, u8* buf, u32 len)
{
    const uint chip = PSRAM_CHIP_OF(addr);
    addr = PSRAM_ADDR_IN_CHIP(addr);

    psramMuxToSio();
    psramClkLow();
    psramCeLow(chip);
    psramSendByteSerial(PSRAM_CMD_FAST_READ_QUAD);

    // address + dummy clocks via the TX pump. The address word's 2 spare
    // bottom nibbles are zeros and serve as the first 2 dummy clocks; a zero
    // word provides the remaining 4.
    psramMuxToPio();
    psramTxSmStart(6 + PSRAM_RX_DUMMY_NIBBLES, addr << 8);
    pio_sm_put_blocking(PSRAM_PIO, sTxSm, 0);
    psramTxSmWait();

    // data via the RX pump; the FIFO words are big-endian, hence the bswap.
    // The first TX-FIFO word is the nibble count - 1, pulled into x by the
    // program itself.
    pio_sm_restart(PSRAM_PIO, sRxSm);
    pio_sm_clear_fifos(PSRAM_PIO, sRxSm);
    pio_sm_put(PSRAM_PIO, sRxSm, 2 * len - 1);
    pio_sm_set_enabled(PSRAM_PIO, sRxSm, true);
    u32* w = (u32*)buf;
    for (u32 i = 0; i < len / 4; i++)
    {
        w[i] = __builtin_bswap32(pio_sm_get_blocking(PSRAM_PIO, sRxSm));
    }
    pio_sm_set_enabled(PSRAM_PIO, sRxSm, false);

    psramMuxToSio();
    psramCeHigh(chip);
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
    const uint chip = PSRAM_CHIP_OF(addr);
    addr = PSRAM_ADDR_IN_CHIP(addr);

    psramClkLow();
    psramCeLow(chip);
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
    psramCeHigh(chip);
}

static void __no_inline_not_in_flash_func(psramBbWriteBurst)(u32 addr, const u8* buf, u32 len)
{
    const uint chip = PSRAM_CHIP_OF(addr);
    addr = PSRAM_ADDR_IN_CHIP(addr);

    psramClkLow();
    psramCeLow(chip);
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
    psramCeHigh(chip);

    // release the data bus
    sio_hw->gpio_oe_clr = PSRAM_IO_MASK;
}

// ---------------------------------------------------------------------------
// 1-bit SPI fallback (diagnosis only)
// ---------------------------------------------------------------------------

// Sends one bit serially on IO0. Caller must have IO0 driven and CE low.
static inline void psramBbSendBit(bool bit)
{
    psramClkLow();
    if (bit)
        sio_hw->gpio_set = PSRAM_IO0_MASK;
    else
        sio_hw->gpio_clr = PSRAM_IO0_MASK;
    psramClkHigh();
}

// 1-bit SPI WRITE (0x02): serial command, address and data on IO0.
// The most primitive transaction the chip supports - independent of QE and
// of the IO2/IO3 solder joints, so it can prove the chip is alive when both
// quad paths fail.
static void __no_inline_not_in_flash_func(psramBbSpiWriteBurst)(u32 addr, const u8* buf, u32 len)
{
    const uint chip = PSRAM_CHIP_OF(addr);
    addr = PSRAM_ADDR_IN_CHIP(addr);

    psramClkLow();
    psramCeLow(chip);
    sio_hw->gpio_oe_set = PSRAM_IO0_MASK;
    u32 v = (0x02u << 24) | (addr & 0xFFFFFFu);
    for (int i = 0; i < 32; i++)
    {
        psramBbSendBit(v & 0x80000000u);
        v <<= 1;
    }
    for (u32 i = 0; i < len; i++)
    {
        u8 b = buf[i];
        for (int j = 0; j < 8; j++)
        {
            psramBbSendBit(b & 0x80);
            b <<= 1;
        }
    }
    psramClkLow();
    sio_hw->gpio_oe_clr = PSRAM_IO0_MASK;
    psramCeHigh(chip);
}

// 1-bit SPI READ (0x03): serial command+address on IO0, data back on IO1 (SO).
static void __no_inline_not_in_flash_func(psramBbSpiReadBurst)(u32 addr, u8* buf, u32 len)
{
    const uint chip = PSRAM_CHIP_OF(addr);
    addr = PSRAM_ADDR_IN_CHIP(addr);

    psramClkLow();
    psramCeLow(chip);
    sio_hw->gpio_oe_set = PSRAM_IO0_MASK;
    u32 v = (0x03u << 24) | (addr & 0xFFFFFFu);
    for (int i = 0; i < 32; i++)
    {
        psramBbSendBit(v & 0x80000000u);
        v <<= 1;
    }
    psramClkLow();
    sio_hw->gpio_oe_clr = PSRAM_IO0_MASK; // release SI; chip drives SO (IO1)
    for (u32 i = 0; i < len; i++)
    {
        u8 b = 0;
        for (int j = 0; j < 8; j++)
        {
            psramClkHigh();
            __asm volatile ("nop\n nop\n nop\n nop");
            b = (u8)((b << 1) | ((sio_hw->gpio_in >> PSRAM_PIN_IO1) & 1u));
            psramClkLow();
        }
        buf[i] = b;
    }
    psramCeHigh(chip);
}

// 1-bit SPI Read ID ('h9F): serial command + 24 dummy address bits, then the
// chip streams its EID (vendor ID, KGD, density, MFG) on SO. Per datasheet it
// is only valid right after a global reset. Read-only and independent of the
// write path and of the address decoder - the cleanest possible alive test.
static void __no_inline_not_in_flash_func(psramBbSpiReadId)(uint chip, u8* eid, u32 len)
{
    psramClkLow();
    psramCeLow(chip);
    sio_hw->gpio_oe_set = PSRAM_IO0_MASK;
    u32 v = 0x9Fu << 24; // cmd + 24 dummy address bits
    for (int i = 0; i < 32; i++)
    {
        psramBbSendBit(v & 0x80000000u);
        v <<= 1;
    }
    psramClkLow();
    sio_hw->gpio_oe_clr = PSRAM_IO0_MASK; // release SI; chip drives SO (IO1)
    for (u32 i = 0; i < len; i++)
    {
        u8 b = 0;
        for (int j = 0; j < 8; j++)
        {
            psramClkHigh();
            __asm volatile ("nop\n nop\n nop\n nop");
            b = (u8)((b << 1) | ((sio_hw->gpio_in >> PSRAM_PIN_IO1) & 1u));
            psramClkLow();
        }
        eid[i] = b;
    }
    psramCeHigh(chip);
}

// Write/read-compare at the start and end of every chip over the 1-bit SPI path.
static bool psramSpiSelfTest(void)
{
    u8 pattern[32], readBack[32];
    for (uint chip = 0; chip < PSRAM_CHIP_COUNT; chip++)
    {
        for (u32 off = 0; off < PSRAM_CHIP_SIZE_BYTES; off += PSRAM_CHIP_SIZE_BYTES - sizeof(pattern))
        {
            u32 addr = chip * PSRAM_CHIP_SIZE_BYTES + off;
            for (u32 i = 0; i < sizeof(pattern); i++)
                pattern[i] = (u8)(addr + i * 0x9Du + 0x35u);
            memset(readBack, 0xA5, sizeof(readBack));
            psramBbSpiWriteBurst(addr, pattern, sizeof(pattern));
            psramBbSpiReadBurst(addr, readBack, sizeof(readBack));
            if (memcmp(pattern, readBack, sizeof(pattern)) != 0)
            {
                LOG("PSRAM: SPI self-test FAILED @0x%08lX wrote[0:8]=", addr);
                for (u32 i = 0; i < 8; i++) LOG("%02X", pattern[i]);
                LOG(" read[0:8]=");
                for (u32 i = 0; i < 8; i++) LOG("%02X", readBack[i]);
                LOG("\n");
                return false;
            }
        }
    }
    return true;
}

// ---------------------------------------------------------------------------
// Rework aid: pin probe walk
// ---------------------------------------------------------------------------

// When no PSRAM responds, call this repeatedly: it walks a static level across
// the PSRAM pins (one every ~3 s, printed on the UART) so each U2/U4/U5/U6 pad
// can be probed with a multimeter. A line that never changes at the chip pads
// has an open trace / bad via / misroute between the RP2354A and the chips.
void psram_pin_probe_step(void)
{
    static const u8 bus[] = { PSRAM_PIN_CLK, PSRAM_PIN_IO0, PSRAM_PIN_IO1, PSRAM_PIN_IO2, PSRAM_PIN_IO3 };
    static const char* const busDst[] = { "SCLK (pin 6 of U2/U4/U5/U6)",
                                          "SIO0 (pin 5)", "SIO1 (pin 2)",
                                          "SIO2 (pin 3)", "SIO3 (pin 7)" };
    static const u8 ces[] = { PSRAM_PIN_CE0, PSRAM_PIN_CE1, PSRAM_PIN_CE2, PSRAM_PIN_CE3 };
    static const char* const ceDst[] = { "U2-1 (R3)", "U4-1 (R4)", "U5-1 (R5)", "U6-1 (R13)" };
    enum { BUS_PINS = 5, CE_PINS = 4, TOTAL_STEPS = BUS_PINS + CE_PINS + 1 };
    static u32 step = TOTAL_STEPS; // force immediate first print
    static u32 last = 0;

    u32 now = millis();
    if (step < TOTAL_STEPS && (u32)(now - last) < 3000)
        return;
    last = now;
    step = (step + 1) % TOTAL_STEPS;

    psramMuxToSio();
    sio_hw->gpio_oe_set = PSRAM_IO_MASK | PSRAM_CLK_MASK | PSRAM_CE_ALL_MASK;
    sio_hw->gpio_clr = PSRAM_IO_MASK | PSRAM_CLK_MASK; // bus low, no hw_write_masked on SIO!
    sio_hw->gpio_set = PSRAM_CE_ALL_MASK;              // all CE# deasserted

    if (step < BUS_PINS)
    {
        sio_hw->gpio_set = 1u << bus[step];
        LOG("[PROBE] GPIO%u (-> %s) = HIGH, other PSRAM bus lines LOW, all CE HIGH\n",
            bus[step], busDst[step]);
    }
    else if (step < BUS_PINS + CE_PINS)
    {
        u32 ce = step - BUS_PINS;
        sio_hw->gpio_clr = 1u << ces[ce];
        LOG("[PROBE] CE%u GPIO%u (-> %s) = LOW (idles HIGH via pull-up)\n",
            ce, ces[ce], ceDst[ce]);
    }
    else
    {
        LOG("[PROBE] bus low, all CE high (idle state)\n");
    }
}

// 2x2 isolation of the failing quad path. SPI (serial) read/write already
// works, so cross the directions: quad-write+serial-read isolates the quad
// write path, serial-write+quad-read isolates the quad read path. Marker
// bytes exercise every IO line individually so the readback shows whether
// IO2/IO3 are open (bits read 0/garbage) or swapped on the PCB (bits 2<->3
// exchanged within each nibble).
static void psramQuadCrossCheck(void)
{
    // Nibbles 1/2/4/8 in various positions: every IO line high on its own.
    static const u8 marker[8] = { 0x11, 0x22, 0x44, 0x88, 0x24, 0x42, 0x81, 0x18 };
    u8 rb[8];

    psramBbWriteBurst(0, marker, sizeof(marker)); // quad write (IO0-3)
    memset(rb, 0xA5, sizeof(rb));
    psramBbSpiReadBurst(0, rb, sizeof(rb));       // serial read (IO0/IO1)
    LOG("PSRAM: cross quadW->serialR wrote=");
    for (u32 i = 0; i < 8; i++) LOG("%02X", marker[i]);
    LOG(" read=");
    for (u32 i = 0; i < 8; i++) LOG("%02X", rb[i]);
    LOG("%s\n", memcmp(marker, rb, 8) == 0 ? " MATCH" : " MISMATCH");

    psramBbSpiWriteBurst(8, marker, sizeof(marker)); // serial write
    memset(rb, 0xA5, sizeof(rb));
    psramBbReadBurst(8, rb, sizeof(rb));             // quad read (IO0-3)
    LOG("PSRAM: cross serialW->quadR wrote=");
    for (u32 i = 0; i < 8; i++) LOG("%02X", marker[i]);
    LOG(" read=");
    for (u32 i = 0; i < 8; i++) LOG("%02X", rb[i]);
    LOG("%s\n", memcmp(marker, rb, 8) == 0 ? " MATCH" : " MISMATCH");
}

// ---------------------------------------------------------------------------
// QPI mode bit-bang (diagnosis)
// ---------------------------------------------------------------------------

// QPI-mode bursts: like the SPI-mode quad bursts but the command byte itself
// is sent as two quad nibbles. Some APS6404L silicon/compatibles ignore the
// SPI-mode quad commands ('hEB/'h38 with serial command phase) and only accept
// quad transfers in QPI mode (after EQIO 'h35).
static void __no_inline_not_in_flash_func(psramBbQpiWriteBurst)(u32 addr, const u8* buf, u32 len)
{
    const uint chip = PSRAM_CHIP_OF(addr);
    addr = PSRAM_ADDR_IN_CHIP(addr);

    psramClkLow();
    psramCeLow(chip);
    sio_hw->gpio_oe_set = PSRAM_IO_MASK;
    psramBbSendNibble(PSRAM_CMD_QUAD_WRITE >> 4);
    psramBbSendNibble(PSRAM_CMD_QUAD_WRITE & 0xF);
    for (int s = 20; s >= 0; s -= 4)
        psramBbSendNibble((addr >> s) & 0xF);
    for (u32 i = 0; i < len; i++)
    {
        psramBbSendNibble(buf[i] >> 4);
        psramBbSendNibble(buf[i] & 0xF);
    }
    psramClkLow();
    sio_hw->gpio_oe_clr = PSRAM_IO_MASK;
    psramCeHigh(chip);
}

static void __no_inline_not_in_flash_func(psramBbQpiReadBurst)(u32 addr, u8* buf, u32 len)
{
    const uint chip = PSRAM_CHIP_OF(addr);
    addr = PSRAM_ADDR_IN_CHIP(addr);

    psramClkLow();
    psramCeLow(chip);
    sio_hw->gpio_oe_set = PSRAM_IO_MASK;
    psramBbSendNibble(PSRAM_CMD_FAST_READ_QUAD >> 4);
    psramBbSendNibble(PSRAM_CMD_FAST_READ_QUAD & 0xF);
    for (int s = 20; s >= 0; s -= 4)
        psramBbSendNibble((addr >> s) & 0xF);
    // Same off-by-one as psramBbReadBurst: SCLK is high here; pull it low so
    // every dummy clock produces a rising edge.
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
    psramCeHigh(chip);
}

// Sends a command in QPI mode (command phase is quad).
static void psramBbSendCmdQpi(uint chip, u8 cmd)
{
    psramClkLow();
    psramCeLow(chip);
    sio_hw->gpio_oe_set = PSRAM_IO_MASK;
    psramBbSendNibble(cmd >> 4);
    psramBbSendNibble(cmd & 0xF);
    psramClkLow();
    sio_hw->gpio_oe_clr = PSRAM_IO_MASK;
    psramCeHigh(chip);
}

static bool __attribute__((unused)) psramQpiSelfTest(void)
{
    u8 pattern[32], readBack[32];
    for (uint chip = 0; chip < PSRAM_CHIP_COUNT; chip++)
    {
        for (u32 off = 0; off < PSRAM_CHIP_SIZE_BYTES; off += PSRAM_CHIP_SIZE_BYTES - sizeof(pattern))
        {
            u32 addr = chip * PSRAM_CHIP_SIZE_BYTES + off;
            for (u32 i = 0; i < sizeof(pattern); i++)
                pattern[i] = (u8)(addr + i * 0x9Du + 0x35u);
            memset(readBack, 0xA5, sizeof(readBack));
            psramBbQpiWriteBurst(addr, pattern, sizeof(pattern));
            psramBbQpiReadBurst(addr, readBack, sizeof(readBack));
            if (memcmp(pattern, readBack, sizeof(pattern)) != 0)
            {
                LOG("PSRAM: QPI self-test FAILED @0x%08lX wrote[0:8]=", addr);
                for (u32 i = 0; i < 8; i++) LOG("%02X", pattern[i]);
                LOG(" read[0:8]=");
                for (u32 i = 0; i < 8; i++) LOG("%02X", readBack[i]);
                LOG("\n");
                return false;
            }
        }
    }
    return true;
}

// ---------------------------------------------------------------------------
// IO2/IO3 wiring brute-force scanner (board bring-up last resort)
// ---------------------------------------------------------------------------

// The chip provably accepts serial commands and provably never receives quad
// ones. If the IO2/IO3 nets are misrouted at the design level (wrong GPIOs in
// the schematic that every PCB spin inherits), no amount of rework fixes it.
// This scanner enters QPI mode (serial EQIO always works), then tries to
// leave it with a quad 'hF5 driven on EVERY candidate GPIO pair. After each
// attempt a full 1-bit SPI self-test reports whether the chip is back in SPI
// mode - i.e. whether that pair actually reached the chip's IO2/IO3 inputs.
static void psramSendCmd(uint chip, u8 cmd); // defined below, before psram_init

// Candidate GPIOs for a misrouted IO2/IO3: the free pins (1, 2) and the NDS
// cartridge-bus pins (9-21), which idle unconnected on this debug board.
// The PSRAM (0, 22-29) and SD (3-8) pins must never be scanned.
static const u8 sScanGpio[] = {
    1, 2, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21
};

// Sends one quad nibble with bit0 on GPIO22 (IO0), bit1 on GPIO23 (IO1),
// bit2 on g2, bit3 on g3. Caller holds CE low.
static inline void psramScanSendNibble(u8 nibble, u8 g2, u8 g3)
{
    psramClkLow();
    gpio_put(PSRAM_PIN_IO0, nibble & 1u);
    gpio_put(PSRAM_PIN_IO1, (nibble >> 1) & 1u);
    gpio_put(g2, (nibble >> 2) & 1u);
    gpio_put(g3, (nibble >> 3) & 1u);
    psramClkHigh();
}

static void __attribute__((unused)) psramIo23Scan(void)
{
    // Candidates become SIO outputs, idle low.
    for (u32 i = 0; i < count_of(sScanGpio); i++)
    {
        gpio_init(sScanGpio[i]);
        gpio_put(sScanGpio[i], false);
        gpio_set_dir(sScanGpio[i], true);
    }

    LOG("PSRAM: scanning %u GPIOs for the real IO2/IO3 wiring (quad F5 round-trip)...\n",
        (u32)count_of(sScanGpio));
    bool found = false;
    for (u32 a = 0; a < count_of(sScanGpio) && !found; a++)
    {
        for (u32 b = 0; b < count_of(sScanGpio) && !found; b++)
        {
            u8 g2 = sScanGpio[a], g3 = sScanGpio[b];
            if (g2 == g3)
                continue;

            // (Re-)enter QPI: serial EQIO works in SPI mode and is ignored as
            // an invalid quad command if the chip is already in QPI mode.
            psramSendCmd(0, PSRAM_CMD_ENTER_QUAD);

            // Try to exit QPI with a quad 'hF5 on this candidate pair.
            sio_hw->gpio_oe_set = PSRAM_IO0_MASK | (1u << PSRAM_PIN_IO1);
            psramClkLow();
            psramCeLow(0);
            psramScanSendNibble(0xF, g2, g3);
            psramScanSendNibble(0x5, g2, g3);
            psramClkLow();
            psramCeHigh(0);
            sio_hw->gpio_oe_clr = PSRAM_IO0_MASK | (1u << PSRAM_PIN_IO1);

            // Observer: full 1-bit SPI self-test only passes in SPI mode.
            if (psramSpiSelfTest())
            {
                LOG("PSRAM: *** IO2/IO3 FOUND: IO2 = GPIO%u, IO3 = GPIO%u ***\n",
                    (u32)g2, (u32)g3);
                found = true;
            }
        }
    }
    if (!found)
        LOG("PSRAM: scan found NO working GPIO pair - IO2/IO3 open at the chip "
            "joint/trace, or dead chip inputs\n");

    // Leave every candidate released.
    for (u32 i = 0; i < count_of(sScanGpio); i++)
        gpio_set_dir(sScanGpio[i], false);
}

// QPI-mode cross check, with 1-bit SPI as a reliable observer. The quad 'hF5
// exit is proven to reach the chip (psramIo23Scan found IO2=GPIO24/IO3=GPIO25),
// so every phase ends back in SPI mode where memory can be read reliably.
// Interpretation of test A readback:
//   50..57  -> QPI write works;   A0..A7 -> QPI write ignored;
//   00..00  -> chip latched zeros (phase misalignment);
//   garbage -> chip did not return to SPI mode (F5 failed).
static void __attribute__((unused)) psramQpiCrossCheck(void)
{
    u8 a[8], b[8], rb[8];
    for (u32 i = 0; i < 8; i++)
    {
        a[i] = (u8)(0xA0 + i);
        b[i] = (u8)(0x50 + i);
    }

    // A: QPI write -> serial read (isolates the quad write path).
    psramBbSpiWriteBurst(0, a, sizeof(a));
    psramSendCmd(0, PSRAM_CMD_ENTER_QUAD);
    psramBbQpiWriteBurst(0, b, sizeof(b));
    psramBbSendCmdQpi(0, 0xF5);
    memset(rb, 0, sizeof(rb));
    psramBbSpiReadBurst(0, rb, sizeof(rb));
    LOG("PSRAM: QPI-write -> serial-read: prewrote A0..A7, QPI-wrote 50..57, read back ");
    for (u32 i = 0; i < 8; i++) LOG("%02X", rb[i]);
    LOG("\n");

    // B: serial write -> QPI read (isolates the quad read path).
    psramBbSpiWriteBurst(8, a, sizeof(a));
    psramSendCmd(0, PSRAM_CMD_ENTER_QUAD);
    memset(rb, 0, sizeof(rb));
    psramBbQpiReadBurst(8, rb, sizeof(rb));
    psramBbSendCmdQpi(0, 0xF5);
    LOG("PSRAM: serial-write -> QPI-read: prewrote A0..A7, QPI read back ");
    for (u32 i = 0; i < 8; i++) LOG("%02X", rb[i]);
    LOG("\n");
}

// ---------------------------------------------------------------------------
// Phase-by-phase transaction bisect (bring-up)
// ---------------------------------------------------------------------------

// Quick alive check: is the chip answering 1-bit SPI right now?
static bool psramSpiAlive(void)
{
    u8 w = 0x5A, r = 0;
    psramBbSpiWriteBurst(256, &w, 1);
    psramBbSpiReadBurst(256, &r, 1);
    return r == w;
}

// Sends a truncated QPI write transaction: cmdNibbles of the 'h38 command,
// addrNibbles of address 0, then dataNibbles of data; ends with CE high.
static void psramQpiPartial(uint chip, u32 cmdNibbles, u32 addrNibbles, u32 dataNibbles, const u8* data)
{
    static const u8 cmdN[2] = { PSRAM_CMD_QUAD_WRITE >> 4, PSRAM_CMD_QUAD_WRITE & 0xF };
    psramClkLow();
    psramCeLow(chip);
    sio_hw->gpio_oe_set = PSRAM_IO_MASK;
    for (u32 i = 0; i < cmdNibbles; i++)
        psramBbSendNibble(cmdN[i]);
    for (u32 i = 0; i < addrNibbles; i++)
        psramBbSendNibble(0);
    for (u32 i = 0; i < dataNibbles; i++)
        psramBbSendNibble((i & 1) ? (data[i / 2] & 0xF) : (data[i / 2] >> 4));
    psramClkLow();
    sio_hw->gpio_oe_clr = PSRAM_IO_MASK;
    psramCeHigh(chip);
}

static void psramHexLog(const char* tag, const u8* d, u32 n)
{
    LOG("%s", tag);
    for (u32 i = 0; i < n; i++)
        LOG("%02X", d[i]);
    LOG("\n");
}

static void psramQpiBisect(void)
{
    u8 p1[16], p2[16], rb[16];
    for (u32 i = 0; i < 16; i++)
    {
        p1[i] = (u8)(0xA0 + i);
        p2[i] = (u8)(0x50 + i);
    }

    // [1] SPI baseline: observer works at this point in the flow.
    psramBbSpiWriteBurst(0, p1, 16);
    memset(rb, 0, 16);
    psramBbSpiReadBurst(0, rb, 16);
    psramHexLog("PSRAM bisect[1] spiW->spiR @0 rd=", rb, 16);

    // [2] SPI-mode quad write ('h38, serial cmd, quad addr/data) then serial
    // readback: shows EXACTLY what the quad write put in memory.
    psramBbWriteBurst(0, p2, 16);
    memset(rb, 0, 16);
    psramBbSpiReadBurst(0, rb, 16);
    psramHexLog("PSRAM bisect[2] spiQuadW->spiR @0 rd=", rb, 16);

    // [3] SPI-mode quad read ('hEB) of known content.
    psramBbSpiWriteBurst(0, p1, 16);
    memset(rb, 0, 16);
    psramBbReadBurst(0, rb, 16);
    psramHexLog("PSRAM bisect[3] spiQuadR @0 raw=", rb, 16);

    // [4..7] QPI bisect: EQIO -> truncated write -> F5 -> SPI alive?
    struct { const char* tag; u32 cmdN, addrN, dataN; } static const steps[] = {
        { "[4] EQIO+F5 only",        0, 0, 0  },
        { "[5] +cmd(2 nib)",         2, 0, 0  },
        { "[6] +cmd+addr(8 nib)",    2, 6, 0  },
        { "[7] +cmd+addr+1B data",   2, 6, 2  },
        { "[8] full 16B write",      2, 6, 32 },
    };
    for (u32 s = 0; s < count_of(steps); s++)
    {
        psramSendCmd(0, PSRAM_CMD_ENTER_QUAD);
        if (steps[s].cmdN)
            psramQpiPartial(0, steps[s].cmdN, steps[s].addrN, steps[s].dataN, p2);
        psramBbSendCmdQpi(0, 0xF5);
        LOG("PSRAM bisect%s -> after F5 chip %s\n", steps[s].tag,
            psramSpiAlive() ? "in SPI mode (OK)" : "STUCK (not SPI)");
    }

    // [9] What did the full QPI write @0 actually store? (serial readback)
    memset(rb, 0, 16);
    psramBbSpiReadBurst(0, rb, 16);
    psramHexLog("PSRAM bisect[9] after full QPI write, spiR @0 rd=", rb, 16);

    // [10] serial write known content @16, then QPI read raw.
    psramBbSpiWriteBurst(16, p1, 16);
    psramSendCmd(0, PSRAM_CMD_ENTER_QUAD);
    memset(rb, 0, 16);
    psramBbQpiReadBurst(16, rb, 16);
    psramBbSendCmdQpi(0, 0xF5);
    psramHexLog("PSRAM bisect[10] spiW A0..AF @16, qpiR raw=", rb, 16);
    LOG("PSRAM bisect[10] after QPI read + F5 chip %s\n",
        psramSpiAlive() ? "in SPI mode (OK)" : "STUCK (not SPI)");
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

static void psramSendCmd(uint chip, u8 cmd)
{
    psramClkLow();
    psramCeLow(chip);
    psramSendByteSerial(cmd);
    psramCeHigh(chip);
}

// Bitmask of chips that passed the self-test (bit0 = CE0 ... bit3 = CE3).
// Exported so the heartbeat can show per-chip status.
static u8 g_psramChipOkMask;

u8 psram_chip_ok_mask(void)
{
    return g_psramChipOkMask;
}

// Self-test: write address-dependent patterns at the start, middle and end of
// every chip and read them back. Returns a bitmask of the responding chips;
// a chip that does not answer (not fitted, solder, power or dead die) simply
// clears its bit - the rest of the array keeps testing.
static u8 psramSelfTestMask(void)
{
    u8 okMask = 0;
    for (uint chip = 0; chip < PSRAM_CHIP_COUNT; chip++)
    {
        u32 base = chip * PSRAM_CHIP_SIZE_BYTES;
        const u32 testAddrs[3] = {
            base,
            base + (PSRAM_CHIP_SIZE_BYTES / 2) - PSRAM_PIO_BURST_BYTES,
            base + PSRAM_CHIP_SIZE_BYTES - PSRAM_PIO_BURST_BYTES,
        };

        bool chipOk = true;
        for (int t = 0; t < 3 && chipOk; t++)
        {
            u32 addr = testAddrs[t];
            u8 pattern[PSRAM_PIO_BURST_BYTES];
            u8 readBack[PSRAM_PIO_BURST_BYTES];
            for (u32 i = 0; i < sizeof(pattern); i++)
                pattern[i] = (u8)(addr + i * 0x9Du + 0x35u);

            psram_write(addr, pattern, sizeof(pattern));
            psram_read(addr, readBack, sizeof(readBack));
            if (memcmp(pattern, readBack, sizeof(pattern)) != 0)
            {
                // Locate the first mismatching byte and report it so the failure
                // mode is diagnosable: all-zero readback means no device / not
                // responding; all-FF means stuck lines; shifted data means a
                // clock or address-bit fault.
                u32 firstBad = 0;
                while (firstBad < sizeof(pattern) && pattern[firstBad] == readBack[firstBad])
                    firstBad++;
                LOG("PSRAM: self-test FAILED @0x%08lX byte[%lu] wrote=0x%02X read=0x%02X\n",
                    (u32)addr, firstBad, pattern[firstBad], readBack[firstBad]);
                LOG("PSRAM: wrote[0:8]=");
                for (u32 i = 0; i < 8 && i < sizeof(pattern); i++) LOG("%02X", pattern[i]);
                LOG(" read[0:8]=");
                for (u32 i = 0; i < 8 && i < sizeof(readBack); i++) LOG("%02X", readBack[i]);
                LOG("\n");
                // Quick signature: how many bytes match, and is readback all-00 / all-FF?
                u32 match = 0, zeros = 0, ones = 0;
                for (u32 i = 0; i < sizeof(readBack); i++)
                {
                    if (readBack[i] == pattern[i]) match++;
                    if (readBack[i] == 0x00) zeros++;
                    if (readBack[i] == 0xFF) ones++;
                }
                LOG("PSRAM: %lu/%lu bytes match, %lu zeros, %lu ones (of %lu)\n",
                    match, (u32)sizeof(readBack), zeros, ones, (u32)sizeof(readBack));
                if (sUsePio)
                {
                    // Split TX pump vs RX pump. The SM PCs should be parked at
                    // the blocking pull (the program's wrap target = offset).
                    LOG("PSRAM: txSM pc=%u (off=%u) rxSM pc=%u (off=%u)\n",
                        pio_sm_get_pc(PSRAM_PIO, sTxSm), sTxOffset,
                        pio_sm_get_pc(PSRAM_PIO, sRxSm), sRxOffset);
                    // 1) Did the PIO write land? Re-read via the known-good BB path.
                    u8 probe[8];
                    psramBbReadBurst(addr, probe, 8);
                    LOG("PSRAM: pioW->bbR[0:8]=");
                    for (u32 i = 0; i < 8; i++) LOG("%02X", probe[i]);
                    LOG("\n");
                    // 2) Does the PIO read path work? Write via the BB path first.
                    psramBbWriteBurst(addr, pattern, 8);
                    memset(probe, 0, 8);
                    psramPioReadBurst(addr, probe, 8);
                    LOG("PSRAM: bbW->pioR[0:8]=");
                    for (u32 i = 0; i < 8; i++) LOG("%02X", probe[i]);
                    LOG("\n");
                }
                chipOk = false;
            }
            else
            {
                LOG("PSRAM: self-test OK @0x%08lX\n", (u32)addr);
            }
        }

        if (chipOk)
            okMask |= (u8)(1u << chip);
        else
            LOG("PSRAM: chip CE%u (GPIO%u) NOT responding\n", chip, PSRAM_PIN_CE0 + chip);
    }
    return okMask;
}

// Pin integrity check for rework diagnosis: drive each data/clock line in
// turn (CE# stays high so the chip is deselected and Hi-Z) and verify no
// other line follows. Detects solder bridges between adjacent pins - the
// most common rework defect. Cannot detect open circuits (lines float).
static bool psramPinBridgeTest(void)
{
    static const u8 pins[] = { PSRAM_PIN_IO0, PSRAM_PIN_IO1, PSRAM_PIN_IO2, PSRAM_PIN_IO3, PSRAM_PIN_CLK };
    bool ok = true;

    psramMuxToSio();
    sio_hw->gpio_oe_clr = PSRAM_IO_MASK | PSRAM_CLK_MASK;

    for (u32 i = 0; i < count_of(pins); i++)
    {
        for (u32 j = 0; j < count_of(pins); j++)
        {
            gpio_put(pins[j], i == j);
            gpio_set_dir(pins[j], true);
        }
        busy_wait_us_32(10);
        u32 in = sio_hw->gpio_in;
        for (u32 j = 0; j < count_of(pins); j++)
        {
            bool level = (in >> pins[j]) & 1u;
            if (level != (i == j))
            {
                LOG("PSRAM: pin check FAIL: drove GPIO%u %s but GPIO%u reads %d (bridge?)\n",
                    pins[i], i == j ? "high" : "low", pins[j], (int)level);
                ok = false;
            }
        }
    }

    // Back to idle: data lines released, clock driven low.
    sio_hw->gpio_oe_clr = PSRAM_IO_MASK;
    gpio_put(PSRAM_PIN_CLK, false);
    gpio_set_dir(PSRAM_PIN_CLK, true);
    return ok;
}

bool psram_init(void)
{
    gpio_init_mask(PSRAM_PIN_MASK);

    // CE#s and CLK are outputs, the data pins are inputs until a transfer starts.
    gpio_put_masked(PSRAM_CE_ALL_MASK, PSRAM_CE_ALL_MASK); // all chips deselected
    gpio_put(PSRAM_PIN_CLK, false);
    gpio_set_dir_out_masked(PSRAM_CE_ALL_MASK | PSRAM_CLK_MASK);

    // Fast edges for the high PIO clock rate.
    static const u8 fastPins[] = { PSRAM_PIN_CLK, PSRAM_PIN_IO0, PSRAM_PIN_IO1,
                                   PSRAM_PIN_IO2, PSRAM_PIN_IO3 };
    for (u32 i = 0; i < count_of(fastPins); i++)
    {
        gpio_set_slew_rate(fastPins[i], GPIO_SLEW_RATE_FAST);
        gpio_set_drive_strength(fastPins[i], GPIO_DRIVE_STRENGTH_8MA);
    }

    // The devices need 150 us after power-up before they accept commands.
    sleep_us(200);

    LOG("PSRAM: pin bridge test %s\n", psramPinBridgeTest() ? "OK" : "FAILED (see above)");

    // CE driver check (not covered by the bridge test, which keeps CE high):
    // toggle each CE low and read the pad back. If it does not follow, that
    // GPIO driver is dead (or the CE pull-up/joint is open) and the chip can
    // never be selected - which looks exactly like a dead chip.
    for (uint chip = 0; chip < PSRAM_CHIP_COUNT; chip++)
    {
        gpio_put(PSRAM_PIN_CE0 + chip, false);
        busy_wait_us_32(5);
        int ceLowReads = (sio_hw->gpio_in >> (PSRAM_PIN_CE0 + chip)) & 1u;
        gpio_put(PSRAM_PIN_CE0 + chip, true);
        busy_wait_us_32(5);
        int ceHighReads = (sio_hw->gpio_in >> (PSRAM_PIN_CE0 + chip)) & 1u;
        LOG("PSRAM: CE%u (GPIO%u) driver test %s (drove low reads %d, drove high reads %d)\n",
            chip, PSRAM_PIN_CE0 + chip,
            (ceLowReads == 0 && ceHighReads == 1) ? "OK" : "FAIL", ceLowReads, ceHighReads);
    }

    // Per-chip software reset (RSTEN must be immediately followed by RST) and
    // EID read ('h9F) - per datasheet the EID is only valid right after a
    // global reset. Read-only alive test, independent of writes/addressing. A
    // live chip returns its EID: AP Memory vendor ID with KGD byte 'h5D (PASS
    // die) / 'h55 (FAIL die). All-00 / all-FF / drifting garbage means no chip
    // is listening on that CE.
    for (uint chip = 0; chip < PSRAM_CHIP_COUNT; chip++)
    {
        psramSendCmd(chip, PSRAM_CMD_RESET_ENABLE);
        psramSendCmd(chip, PSRAM_CMD_RESET);
        sleep_us(50);

        u8 eid[16];
        psramBbSpiReadId(chip, eid, sizeof(eid));
        LOG("PSRAM: CE%u EID[0:16]=", chip);
        for (u32 i = 0; i < sizeof(eid); i++)
            LOG("%02X", eid[i]);
        LOG(" (KGD byte: 5D=PASS die, 55=FAIL die)\n");
    }

    // NOTE: do NOT send PSRAM_CMD_ENTER_QUAD ('h35) here. This driver issues
    // the quad commands ('hEB/'h38) in SPI mode - serial command phase, quad
    // address/data phases - which the datasheet explicitly allows (section
    // 11, "SPI Mode Operations"). Sending EQIO would switch the chip to QPI
    // mode where the command phase itself must be quad, and every serial
    // command from then on would be ignored (the chip looks dead).

    sUsePio = psramPioInit();
    LOG("PSRAM: self-test using %s path\n", sUsePio ? "PIO" : "bit-bang");
    u8 mask = psramSelfTestMask();
    if (mask == (u8)((1u << PSRAM_CHIP_COUNT) - 1u))
    {
        g_psramChipOkMask = mask;
        LOG("PSRAM: %s data path, all %d chips OK\n",
            sUsePio ? "PIO" : "bit-bang", PSRAM_CHIP_COUNT);
        return true;
    }
    if (mask != 0)
    {
        // The data path works (at least one chip passes), so the failing
        // chips are genuinely not responding - the PIO-vs-bitbang-vs-SPI
        // fallback diagnostics below exist to tell a pump bug from a dead
        // array when NOTHING answers, and would only add noise here.
        g_psramChipOkMask = mask;
        LOG("PSRAM: %s data path OK, chip mask 0x%X (%d/%d chips responding)\n",
            sUsePio ? "PIO" : "bit-bang", mask,
            __builtin_popcount(mask), PSRAM_CHIP_COUNT);
        return true;
    }

    // dspico-debug: cross-check with the bit-bang path before blaming the chip.
    // If bit-bang works where the PIO pump fails, the bug is in the PIO pump;
    // if both fail identically (e.g. all-zero readback), the PSRAM itself is
    // not responding (not fitted, solder, power or dead chip).
    if (sUsePio)
    {
        LOG("PSRAM: PIO path failed, cross-checking with bit-bang...\n");
        pio_sm_set_enabled(PSRAM_PIO, sTxSm, false);
        pio_sm_set_enabled(PSRAM_PIO, sRxSm, false);
        sUsePio = false;
        psramMuxToSio();
        mask = psramSelfTestMask();
        if (mask != 0)
        {
            g_psramChipOkMask = mask;
            LOG("PSRAM: bit-bang data path, chip mask 0x%X\n", mask);
            return true;
        }
    }

    // Both quad paths failed. Last firmware-level check: 1-bit SPI (0x02/0x03)
    // only needs IO0/IO1/SCLK/CE - it works regardless of QE and of the
    // IO2/IO3 joints. SPI ok + quad broken => IO2/IO3 open or QE issue;
    // SPI broken too => IO0/IO1/SCLK/CE/power/orientation/chip.
    LOG("PSRAM: quad paths failed, cross-checking with 1-bit SPI...\n");
    if (psramSpiSelfTest())
    {
        LOG("PSRAM: 1-bit SPI self-test OK - chip is ALIVE; quad transfers broken\n");
        psramQuadCrossCheck();

        // Some APS6404L silicon/compatible parts ignore the SPI-mode quad
        // commands and only accept quad transfers in QPI mode (command phase
        // also quad). Cross-check both QPI directions with 1-bit SPI as the
        // observer (the quad 'hF5 exit is wiring-proven by psramIo23Scan).
        LOG("PSRAM: trying QPI mode (EQIO + quad command phase)...\n");
        psramQpiBisect();
    }
    else
    {
        LOG("PSRAM: 1-bit SPI also failed - chip not responding at all "
            "(check IO0/IO1/SCLK/CE continuity, VCC, orientation, or dead chip)\n");
    }

    return false;
}
