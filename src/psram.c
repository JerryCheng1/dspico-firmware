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
#define PSRAM_CE_MASK   (1u << PSRAM_PIN_CE)

// When true, the address/data phases are streamed by two pio0 state machines
// (psram_qspi_tx/psram_qspi_rx) at SCLK = sysclk / 3. Falls back to bit-banging
// when the pio0 resources are unavailable (e.g. WRFUXXED builds).
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
    int txSm = pio_claim_unused_sm(PSRAM_PIO, false);
    if (txSm < 0)
        return false;
    int rxSm = pio_claim_unused_sm(PSRAM_PIO, false);
    if (rxSm < 0)
        return false;

    sTxSm = (uint)txSm;
    sRxSm = (uint)rxSm;

    // TX pump: nibbles from the TX FIFO to IO0-IO3, SCLK on side-set.
    pio_sm_config c = psram_qspi_tx_program_get_default_config((uint)txOffset);
    sm_config_set_out_pins(&c, PSRAM_PIN_IO0, 4);
    sm_config_set_in_pins(&c, PSRAM_PIN_IO0);
    sm_config_set_set_pins(&c, PSRAM_PIN_IO0, 4);
    sm_config_set_sideset_pins(&c, PSRAM_PIN_CLK);
    sm_config_set_out_shift(&c, true, true, 32); // MSB first, autopull
    pio_sm_init(PSRAM_PIO, sTxSm, (uint)txOffset, &c);
    pio_sm_set_pindirs_with_mask(PSRAM_PIO, sTxSm, 0, PSRAM_IO_MASK);
    pio_sm_set_pins_with_mask(PSRAM_PIO, sTxSm, 0, PSRAM_CLK_MASK);

    // RX pump: nibbles from IO0-IO3 to the RX FIFO.
    c = psram_qspi_rx_program_get_default_config((uint)rxOffset);
    sm_config_set_in_pins(&c, PSRAM_PIN_IO0);
    sm_config_set_sideset_pins(&c, PSRAM_PIN_CLK);
    sm_config_set_in_shift(&c, true, true, 32); // MSB first, autopush
    sm_config_set_out_shift(&c, true, true, 32);
    pio_sm_init(PSRAM_PIO, sRxSm, (uint)rxOffset, &c);
    pio_sm_set_pindirs_with_mask(PSRAM_PIO, sRxSm, 0, PSRAM_IO_MASK);
    pio_sm_set_pins_with_mask(PSRAM_PIO, sRxSm, 0, PSRAM_CLK_MASK);

    return true;
}

// Starts the TX pump with the given nibble count - 1 in x and the first
// stream words already in the TX FIFO.
static inline void psramTxSmStart(u32 nibbleCount, u32 addr, bool withDummy)
{
    pio_sm_restart(PSRAM_PIO, sTxSm);
    pio_sm_clear_fifos(PSRAM_PIO, sTxSm);
    pio_sm_put(PSRAM_PIO, sTxSm, nibbleCount - 1);
    pio_sm_put(PSRAM_PIO, sTxSm, addr << 8); // 24-bit address in the top 6 nibbles
    if (withDummy)
    {
        pio_sm_put(PSRAM_PIO, sTxSm, 0); // dummy nibbles (top 24 bits are shifted out)
    }
    pio_sm_exec(PSRAM_PIO, sTxSm, pio_encode_out(pio_x, 32));
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
    psramMuxToSio();
    psramClkLow();
    psramCeLow();
    psramSendByteSerial(PSRAM_CMD_QUAD_WRITE);

    psramMuxToPio();
    psramTxSmStart(6 + 2 * len, addr, false);
    const u32* w = (const u32*)buf;
    for (u32 i = 0; i < len / 4; i++)
    {
        pio_sm_put_blocking(PSRAM_PIO, sTxSm, w[i]);
    }
    psramTxSmWait();

    psramMuxToSio();
    psramCeHigh();
}

static void __no_inline_not_in_flash_func(psramPioReadBurst)(u32 addr, u8* buf, u32 len)
{
    psramMuxToSio();
    psramClkLow();
    psramCeLow();
    psramSendByteSerial(PSRAM_CMD_FAST_READ_QUAD);

    // address + dummy clocks via the TX pump
    psramMuxToPio();
    psramTxSmStart(6 + PSRAM_RX_DUMMY_NIBBLES, addr, true);
    psramTxSmWait();

    // data via the RX pump
    pio_sm_restart(PSRAM_PIO, sRxSm);
    pio_sm_clear_fifos(PSRAM_PIO, sRxSm);
    pio_sm_put(PSRAM_PIO, sRxSm, 2 * len - 1);
    pio_sm_exec(PSRAM_PIO, sRxSm, pio_encode_out(pio_x, 32));
    pio_sm_set_enabled(PSRAM_PIO, sRxSm, true);
    u32* w = (u32*)buf;
    for (u32 i = 0; i < len / 4; i++)
    {
        w[i] = pio_sm_get_blocking(PSRAM_PIO, sRxSm);
    }
    pio_sm_set_enabled(PSRAM_PIO, sRxSm, false);

    psramMuxToSio();
    psramCeHigh();
}

// ---------------------------------------------------------------------------
// Bit-bang fallback data path
// ---------------------------------------------------------------------------

static inline void psramBbSendNibble(u8 nibble)
{
    psramClkLow();
    hw_write_masked(&sio_hw->gpio_out, (u32)nibble << PSRAM_PIN_IO0, PSRAM_IO_MASK);
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

    // Self-test: write address-dependent patterns at the start, middle and end
    // of the address space and read them back. Fails when no PSRAM is fitted,
    // in which case the caller must not use the PSRAM.
    for (u32 addr = 0; addr < PSRAM_SIZE_BYTES; addr += (PSRAM_SIZE_BYTES / 2) - PSRAM_PIO_BURST_BYTES)
    {
        u8 pattern[PSRAM_PIO_BURST_BYTES];
        u8 readBack[PSRAM_PIO_BURST_BYTES];
        for (u32 i = 0; i < sizeof(pattern); i++)
            pattern[i] = (u8)(addr + i * 0x9Du + 0x35u);

        psram_write(addr, pattern, sizeof(pattern));
        psram_read(addr, readBack, sizeof(readBack));
        if (memcmp(pattern, readBack, sizeof(pattern)) != 0)
            return false;
    }

    printf("PSRAM: %s data path\n", sUsePio ? "PIO" : "bit-bang");
    return true;
}
