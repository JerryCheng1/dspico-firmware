#include "common.h"
#include <string.h>
#include "hardware/gpio.h"
#include "hardware/structs/sio.h"
#include "psram.h"

// APS6404L command set (see APS6404L_3SQR datasheet).
// In SPI mode (QE=0) the command phase is always serial; address and data
// phases of the quad commands are 4-bit. No mode switch is required.
#define PSRAM_CMD_RESET_ENABLE  0x66
#define PSRAM_CMD_RESET         0x99
#define PSRAM_CMD_FAST_READ_QUAD 0xEB // 6 dummy cycles
#define PSRAM_CMD_QUAD_WRITE    0x38

// The datasheet limits CE# low time to tCEM (max 8 us); longer transfers pause
// the internal DRAM refresh and can corrupt data. All transfers are therefore
// split into bursts that complete well within tCEM.
#define PSRAM_BURST_BYTES   32

#define PSRAM_IO_MASK       (0xFu << PSRAM_PIN_IO0)
#define PSRAM_IO0_MASK      (1u << PSRAM_PIN_IO0)
#define PSRAM_CLK_MASK      (1u << PSRAM_PIN_CLK)
#define PSRAM_CE_MASK       (1u << PSRAM_PIN_CE)

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

// Sends one byte serially on IO0. Leaves IO0 as an output.
static inline void psramSendByteSerial(u8 b)
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
}

// Sends one nibble on IO0-IO3. IO0-IO3 must be outputs.
static inline void psramSendNibble(u8 nibble)
{
    psramClkLow();
    hw_write_masked(&sio_hw->gpio_out, (u32)nibble << PSRAM_PIN_IO0, PSRAM_IO_MASK);
    psramClkHigh();
}

// Receives one nibble from IO0-IO3. IO0-IO3 must be inputs.
static inline u8 psramRecvNibble(void)
{
    psramClkHigh();
    __asm volatile ("nop\n nop\n nop\n nop");
    u8 nibble = (sio_hw->gpio_in >> PSRAM_PIN_IO0) & 0xF;
    psramClkLow();
    return nibble;
}

static void __no_inline_not_in_flash_func(psramReadBurst)(u32 addr, u8* buf, u32 len)
{
    psramClkLow();
    psramCeLow();
    psramSendByteSerial(PSRAM_CMD_FAST_READ_QUAD);

    // 24-bit address, 6 quad clocks
    sio_hw->gpio_oe_set = PSRAM_IO_MASK;
    for (int s = 20; s >= 0; s -= 4)
        psramSendNibble((addr >> s) & 0xF);

    // 6 dummy clocks, IOs released
    sio_hw->gpio_oe_clr = PSRAM_IO_MASK;
    for (int i = 0; i < 6; i++)
    {
        psramClkHigh();
        psramClkLow();
    }

    for (u32 i = 0; i < len; i++)
    {
        u8 hi = psramRecvNibble();
        u8 lo = psramRecvNibble();
        buf[i] = (hi << 4) | lo;
    }
    psramCeHigh();
}

static void __no_inline_not_in_flash_func(psramWriteBurst)(u32 addr, const u8* buf, u32 len)
{
    psramClkLow();
    psramCeLow();
    psramSendByteSerial(PSRAM_CMD_QUAD_WRITE);

    // 24-bit address, 6 quad clocks
    sio_hw->gpio_oe_set = PSRAM_IO_MASK;
    for (int s = 20; s >= 0; s -= 4)
        psramSendNibble((addr >> s) & 0xF);

    for (u32 i = 0; i < len; i++)
    {
        psramSendNibble(buf[i] >> 4);
        psramSendNibble(buf[i] & 0xF);
    }
    psramCeHigh();

    // release the data bus
    sio_hw->gpio_oe_clr = PSRAM_IO_MASK;
}

void psram_read(u32 addr, void* buf, u32 len)
{
    u8* dst = (u8*)buf;
    while (len > 0)
    {
        u32 burst = len < PSRAM_BURST_BYTES ? len : PSRAM_BURST_BYTES;
        psramReadBurst(addr, dst, burst);
        addr += burst;
        dst += burst;
        len -= burst;
    }
}

void psram_write(u32 addr, const void* buf, u32 len)
{
    const u8* src = (const u8*)buf;
    while (len > 0)
    {
        u32 burst = len < PSRAM_BURST_BYTES ? len : PSRAM_BURST_BYTES;
        psramWriteBurst(addr, src, burst);
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
    sio_hw->gpio_oe_clr = PSRAM_IO0_MASK;
}

bool psram_init(void)
{
    gpio_init_mask(PSRAM_PIN_MASK);

    // CE# and CLK are outputs, the data pins are inputs until a transfer starts.
    gpio_put(PSRAM_PIN_CE, true);
    gpio_put(PSRAM_PIN_CLK, false);
    gpio_set_dir_out_masked(PSRAM_CE_MASK | PSRAM_CLK_MASK);

    // The device needs 150 us after power-up before it accepts commands.
    sleep_us(200);

    // Software reset (RSTEN must be immediately followed by RST).
    psramSendCmd(PSRAM_CMD_RESET_ENABLE);
    psramSendCmd(PSRAM_CMD_RESET);
    sleep_us(50);

    // Self-test: write address-dependent patterns at the start, middle and end
    // of the address space and read them back. Fails when no PSRAM is fitted,
    // in which case the caller must not use the PSRAM.
    for (u32 addr = 0; addr < PSRAM_SIZE_BYTES; addr += (PSRAM_SIZE_BYTES / 2) - PSRAM_BURST_BYTES)
    {
        u8 pattern[PSRAM_BURST_BYTES];
        u8 readBack[PSRAM_BURST_BYTES];
        for (u32 i = 0; i < sizeof(pattern); i++)
            pattern[i] = (u8)(addr + i * 0x9Du + 0x35u);

        psram_write(addr, pattern, sizeof(pattern));
        psram_read(addr, readBack, sizeof(readBack));
        if (memcmp(pattern, readBack, sizeof(pattern)) != 0)
            return false;
    }

    return true;
}
