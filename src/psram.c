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

// Spinlock guarding pio0->ctrl read-modify-writes. Both cores touch pio0's
// state-machine control register: core0's resetNtrCard()/gpioIrq() restart
// SM0, core1 (PSRAM probe/test) and core0 (cache hit reads) restart SM2/SM3.
// pio_sm_set_enabled/restart do a read-modify-write of pio->ctrl, so without
// a lock one core can clobber the other's bit. Replaces the old
// save_and_disable_interrupts() shielding: that blocked the NDS cart IRQ for
// ~20 us per burst and caused white screens / "failed to mount SD". The
// spinlock keeps cart IRQs enabled; only the pio0 ctrl word is serialized.
static spin_lock_t* sPioLock;

void psram_init_lock(void)
{
    if (!sPioLock)
        sPioLock = spin_lock_instance(spin_lock_claim_unused(true));
}

static inline uint32_t psramPioLockImpl(void)
{
    return spin_lock_blocking(sPioLock);
}

static inline void psramPioUnlockImpl(uint32_t save)
{
    spin_unlock(sPioLock, save);
}

// Exported wrappers for core0 IRQ paths (resetNtrCard/gpioIrq) that cannot
// see the static inline versions.
uint32_t psramPioLock(void)
{
    return psramPioLockImpl();
}

void psramPioUnlock(uint32_t save)
{
    psramPioUnlockImpl(save);
}

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
    // The whole transaction runs under the pio0 ctrl spinlock (NOT
    // save_and_disable_interrupts): the PSRAM protocol needs CE# low ->
    // command -> address -> data continuous, and pio0 ctrl read-modify-writes
    // must be serialized across cores. The spinlock keeps the NDS cart IRQ
    // enabled, so unlike the old IRQ-shielding this does not stall the cart
    // protocol. The lock is held only for the few-us pio0 portion.
    psramMuxToSio();
    psramClkLow();
    psramCeLow();
    psramSendByteSerial(PSRAM_CMD_QUAD_WRITE);

    uint32_t save = psramPioLock();
    psramMuxToPio();
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
    psramPioUnlock(save);

    psramCeHigh();
}

static void __no_inline_not_in_flash_func(psramPioReadBurst)(u32 addr, u8* buf, u32 len)
{
    // See psramPioWriteBurst: full transaction under the pio0 ctrl spinlock.
    psramMuxToSio();
    psramClkLow();
    psramCeLow();
    psramSendByteSerial(PSRAM_CMD_FAST_READ_QUAD);

    uint32_t save = psramPioLock();
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
    psramPioUnlock(save);

    psramCeHigh();
}

// ---------------------------------------------------------------------------
// Bit-bang fallback data path
// ---------------------------------------------------------------------------

static inline void psramBbSendNibble(u8 nibble)
{
    psramClkLow();
    // Drive the 4 data bits atomically via SET/CLR (two single-write SIO
    // ops) instead of a read-modify-write of GPIO_OUT. An RMW would clobber
    // any bit the other core sets concurrently between the read and the
    // write-back (the historical USB PIN_IRQ toggle raced here; USB is gone
    // but the atomic form is correct regardless). SCLK is low during both
    // writes, so the momentary all-low between CLR and SET is not sampled.
    sio_hw->gpio_clr = PSRAM_IO_MASK;
    sio_hw->gpio_set = (u32)nibble << PSRAM_PIN_IO0;
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

// Hardware init only: GPIO, reset, PIO SM2/SM3 config. No bursts, no probe.
// Safe during boot. Uses busy_wait_us (not sleep_us): this may run on core1
// which has interrupts disabled, where sleep_us's WFI would hang.
void psram_init_hw(void)
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
    busy_wait_us(200);

    // Software reset (RSTEN must be immediately followed by RST).
    psramSendCmd(PSRAM_CMD_RESET_ENABLE);
    psramSendCmd(PSRAM_CMD_RESET);
    busy_wait_us(50);

    // The PIO pump (pio0 SM2/SM3) shares pio0's single round-robin execution
    // slot with the cartridge SM0. An earlier bit-bang-only mode
    // (PSRAM_FORCE_BITBANG=1) was adopted because PIO bursts seemed to corrupt
    // the cart protocol - but that was diagnosed while a separate bug (the USB
    // PIN_IRQ gpio_out race) was active. With USB removed, the PIO pump is safe
    // for the core1 probe (which runs before/during the loader menu) and for
    // cache-OFF gameplay.
    //
    // HOWEVER: once the SD sector cache STORE path runs on the core0 main loop
    // (romCacheSdStoreDrain, async out of the E5 IRQ), a PIO psram_write there
    // bursts SM2/SM3 concurrently with the heavy E3/E4/E5 cart traffic of the
    // loader's SD mount - and that breaks mount ("failed to mount SD card")
    // even though the drain is fully preemptible and takes no contended lock.
    // The bit-bang path (pure SIO, no pio0 SM) is being tested as the drain
    // data path to isolate whether pio0 slot contention is the cause. Set 1 to
    // force bit-bang for ALL PSRAM access (probe + cache).
#ifndef PSRAM_FORCE_BITBANG
#define PSRAM_FORCE_BITBANG 1
#endif
#if PSRAM_FORCE_BITBANG
    // Skip psramPioInit entirely: do NOT load the PIO programs, do NOT claim
    // SM2/SM3, leave pio0 untouched. All access goes through psramBbReadBurst/
    // psramBbWriteBurst (SIO only).
    sUsePio = false;
#else
    sUsePio = psramPioInit();
#endif
    LOG("PSRAM: hw init done (%s ready)\n", sUsePio ? "PIO" : "bit-bang");
}

// Probe with bursts. Must NOT run during NDS boot (bursts block/stall the
// boot command stream). Call after boot from the main loop.
bool psram_probe(void)
{
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

bool psram_init(void)
{
    psram_init_hw();
    return psram_probe();
}
