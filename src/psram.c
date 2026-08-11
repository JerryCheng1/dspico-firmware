#include "common.h"
#include <stdio.h>
#include <string.h>
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "hardware/structs/sio.h"
#include "psram.h"
#if !PSRAM_FORCE_BITBANG
#include "hardware/pio.h"
#include "pioUtil.h"
#include "psram.pio.h"
#endif

// APS6404L command set (see APS6404L_3SQR datasheet).
// In SPI mode (QE=0) the command phase is always serial; address and data
// phases of the quad commands are 4-bit. No mode switch is required.
#define PSRAM_CMD_RESET_ENABLE  0x66
#define PSRAM_CMD_RESET         0x99
#define PSRAM_CMD_FAST_READ_QUAD 0xEB // 6 dummy cycles
#define PSRAM_CMD_QUAD_WRITE    0x38

// The datasheet limits CE# low time to tCEM (max 8 us); longer transfers pause
// the internal DRAM refresh and can corrupt data. All transfers are therefore
// split into bursts that complete well within tCEM. Runtime transactions are
// executed on core1 so the cartridge IRQ on core0 cannot stretch a CE#-low
// burst. Bursts are kept aligned to their size so they never cross a 1K PSRAM
// page boundary.
// At clkdiv 3 a 128-byte PIO write is already close to the 8 us CE# limit.
// PIO uses 16-byte bursts so a complete write fits in its joined TX FIFO.
// Bit-bang uses 32 bytes: reducing it to 16 raised a 512-byte hit fetch from
// ~120 us to ~187 us and made the loader's E4 deadline fail consistently.
// Runtime bit-bang is uninterrupted on core1, and every cache hit is protected
// by the SRAM content digest before E5 can expose it.
#define PSRAM_BB_BURST_BYTES    32
#define PSRAM_PIO_PROBE_ROUNDS  128

#define PSRAM_RX_DUMMY_NIBBLES 6

#define PSRAM_IO_MASK   (0xFu << PSRAM_PIN_IO0)
#define PSRAM_IO0_MASK  (1u << PSRAM_PIN_IO0)
#define PSRAM_CLK_MASK  (1u << PSRAM_PIN_CLK)
#define PSRAM_CE_MASK   (1u << PSRAM_PIN_CE)

#if !PSRAM_FORCE_BITBANG
#define PSRAM_PIO_BURST_BYTES   16
#define PSRAM_PIO       pio1
#define PSRAM_TX_DONE_IRQ 2

_Static_assert(sizeof(psram_qspi_command_tx_program_instructions) /
               sizeof(uint16_t) == 5,
               "PIO1 layout reserves exactly five words for PSRAM");

// Serializes complete direct PSRAM transactions. Production PIO cache requests
// are serviced on core1, but keeping transaction ownership here also protects
// the core1 probe/test and any early diagnostic call. No IRQ handler takes the
// lock; cartridge SM0 uses atomic CTRL writes instead (see pioUtil.h).
static spin_lock_t* volatile sPioTransactionLock;

static inline void psramPioTransactionBegin(void)
{
    spin_lock_unsafe_blocking(sPioTransactionLock);
}

static inline void psramPioTransactionEnd(void)
{
    spin_unlock_unsafe(sPioTransactionLock);
}
#endif

// PIO1 SM0/SM1 implement physical SDIO. Their RX/TX programs dynamically
// share one instruction region, leaving five words and SM2 for PSRAM. This
// keeps PSRAM completely off the timing-critical cartridge PIO0 block.
#if !PSRAM_FORCE_BITBANG
#define PSRAM_TX_SM     2

// When true, one pio1 state machine emits the complete serial-command plus
// quad address/write phase (clkdiv 3 -> SCLK ~= sysclk / 6). Read data remains
// on the proven SIO sampler. Falls back when the reserved five instruction
// slots are unavailable.
static volatile bool sUsePio;
static volatile uint sTxSm;
static volatile uint sTxOffset;
#endif

// Every runtime PSRAM transfer, including the production SIO bit-bang path,
// executes on core1. Core0 submits one request at a time; synchronous reads
// wait with the cartridge IRQ enabled, while cache backfills return and poll
// completion later. This is essential for bit-bang: an IRQ on the same
// core could otherwise pause a live CE#-low transaction beyond tCEM. The probe
// already runs directly on core1 and therefore uses the same uninterrupted
// electrical timing.
enum
{
    PSRAM_C1_IDLE = 0,
    PSRAM_C1_READ,
    PSRAM_C1_WRITE,
    PSRAM_C1_DONE,
};
static volatile u32 sCore1Request;
static volatile bool sCore1ServiceReady;
static u32 sCore1Addr;
static u32 sCore1Len;
static void* sCore1Buf;

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

#if !PSRAM_FORCE_BITBANG
static void psramMuxToPio(void)
{
    gpio_set_function(PSRAM_PIN_IO0, GPIO_FUNC_PIO1);
    gpio_set_function(PSRAM_PIN_IO1, GPIO_FUNC_PIO1);
    gpio_set_function(PSRAM_PIN_IO2, GPIO_FUNC_PIO1);
    gpio_set_function(PSRAM_PIN_IO3, GPIO_FUNC_PIO1);
    gpio_set_function(PSRAM_PIN_CLK, GPIO_FUNC_PIO1);
}
#endif

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

#if !PSRAM_FORCE_BITBANG
static void psramBbReadData(u8* buf, u32 len);

static bool psramPioInit(void)
{
    // SDIO owns PIO1 SM0/SM1 and establishes the shared RX/TX instruction
    // layout before PSRAM initialization reaches this point.
    if (pio_sm_is_claimed(PSRAM_PIO, PSRAM_TX_SM))
        return false;
    if (!pio_can_add_program(PSRAM_PIO, &psram_qspi_command_tx_program))
        return false;
    uint txOffset = pio_add_program(PSRAM_PIO, &psram_qspi_command_tx_program);

    // Claim the SM reserved by the PIO1 SDIO/PSRAM layout.
    pio_sm_claim(PSRAM_PIO, PSRAM_TX_SM);
    sTxSm = PSRAM_TX_SM;
    sTxOffset = txOffset;

    // One state machine emits the serial command byte on IO0 and then changes
    // OUT width for the quad address/data phase. IO1-IO3 stay high during the
    // command. clkdiv 3:
    // one serial/quad unit takes two SM cycles, SCLK ~= sysclk / 6 (33 MHz).
    pio_sm_config c = psram_qspi_command_tx_program_get_default_config((uint)txOffset);
    sm_config_set_out_pins(&c, PSRAM_PIN_IO0, 4);
    sm_config_set_in_pins(&c, PSRAM_PIN_IO0);
    sm_config_set_set_pins(&c, PSRAM_PIN_IO0, 4);
    sm_config_set_sideset_pins(&c, PSRAM_PIN_CLK);
    sm_config_set_clkdiv(&c, 3.0f);
    sm_config_set_out_shift(&c, false, true, 32); // MSB (top nibble) first, autopull
    // The joined 8-word TX FIFO holds a complete 16-byte write burst:
    // command/address word + four data words. Fill it while SM2 is disabled so
    // core1 never polls FSTAT or writes TXF2 while SM2 is live.
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    dspicoPioSmInit(PSRAM_PIO, sTxSm, txOffset, &c);
    pio_sm_set_pins_with_mask(PSRAM_PIO, sTxSm, PSRAM_IO_MASK, PSRAM_IO_MASK);
    pio_sm_set_pindirs_with_mask(PSRAM_PIO, sTxSm, 0, PSRAM_IO_MASK);
    // side-set can only drive pins whose direction is output for this SM;
    // without this SCLK never toggles (pio_sm_set_pins only sets the level)
    pio_sm_set_pindirs_with_mask(PSRAM_PIO, sTxSm, PSRAM_CLK_MASK, PSRAM_CLK_MASK);
    pio_sm_set_pins_with_mask(PSRAM_PIO, sTxSm, 0, PSRAM_CLK_MASK);

    return true;
}

// Starts the complete command transmitter. Y is preloaded through two forced
// instructions while the SM is disabled; the runtime FIFO then contains only
// the packed command/address/data stream. This avoids spending two of the
// five available PIO instructions on count loading.
static inline void psramTxSmStart(u32 quadNibbleCount, u32 firstWord)
{
    pio_sm_restart(PSRAM_PIO, sTxSm);
    pio_sm_clear_fifos(PSRAM_PIO, sTxSm);
    pio_interrupt_clear(PSRAM_PIO, PSRAM_TX_DONE_IRQ);

    // SET can load the fixed eight-bit serial count without consuming FIFO
    // space. Y is loaded with forced PULL/OUT instructions while disabled.
    pio_sm_exec_wait_blocking(PSRAM_PIO, sTxSm, pio_encode_set(pio_x, 7));
    pio_sm_put(PSRAM_PIO, sTxSm, quadNibbleCount - 1);
    pio_sm_exec_wait_blocking(PSRAM_PIO, sTxSm, pio_encode_pull(false, true));
    pio_sm_exec_wait_blocking(PSRAM_PIO, sTxSm, pio_encode_out(pio_y, 32));

    // Forced instructions do not reset the normal PC. Always enter at the
    // serial-command instruction. Set all four lines high before enabling
    // their output drivers; serial OUT then changes IO0 only, leaving the
    // three inactive command-phase lines high as required by the PSRAM.
    pio_sm_exec(PSRAM_PIO, sTxSm, pio_encode_jmp(sTxOffset));
    pio_sm_put(PSRAM_PIO, sTxSm, firstWord);
    pio_sm_set_pins_with_mask(PSRAM_PIO, sTxSm, PSRAM_IO_MASK, PSRAM_IO_MASK);
    pio_sm_set_pindirs_with_mask(PSRAM_PIO, sTxSm, PSRAM_IO_MASK, PSRAM_IO_MASK);
}

// Starts a fully-prefilled FIFO, waits for the exact completion signal, then
// releases the bus. PIO1 is shared with SDIO only at the instruction-memory
// level; PSRAM SM2 has its own pins and completion flag.
static inline void psramTxSmRunAndWait(void)
{
    dspicoPioSmSetEnabled(PSRAM_PIO, sTxSm, true);

    // At 200 MHz / clkdiv 3, the longest 16-byte write is ~1.4 us. A 3 us
    // quiet interval guarantees IRQ2 is already set in the normal case, so
    // core1 normally performs only one PIO IRQ read.
    busy_wait_at_least_cycles(600); // 3 us at the fixed 200 MHz sysclk
    while (!pio_interrupt_get(PSRAM_PIO, PSRAM_TX_DONE_IRQ))
        tight_loop_contents();
    // The program is stalled in IRQ WAIT. Disable it before clearing the flag
    // so it cannot wrap and emit one extra serial clock edge.
    dspicoPioSmSetEnabled(PSRAM_PIO, sTxSm, false);
    pio_interrupt_clear(PSRAM_PIO, PSRAM_TX_DONE_IRQ);
    pio_sm_set_pindirs_with_mask(PSRAM_PIO, sTxSm, 0, PSRAM_IO_MASK);
}

static void __no_inline_not_in_flash_func(psramPioWriteBurst)(u32 addr, const u8* buf, u32 len)
{
    // Own the PSRAM pins and chip for the complete CE#-low transaction. The
    // unsafe lock does not alter PRIMASK. Runtime PIO requests execute on
    // core1, independently of the cartridge's PIO0 SM0 and IRQ handler.
    psramPioTransactionBegin();
    psramMuxToPio();

    // The first FIFO word is exactly command byte + 24-bit address. All data
    // follows on a word boundary, unlike the old SIO-command/PIO-address split.
    psramTxSmStart(6 + 2 * len,
                   ((u32)PSRAM_CMD_QUAD_WRITE << 24) | (addr & 0x00FFFFFFu));
    for (u32 off = 0; off < len; off += 4)
    {
        u32 w = 0;
        for (u32 k = 0; k < 4 && off + k < len; k++)
            w |= (u32)buf[off + k] << (24 - 8 * k);
        // Entire transaction fits in the joined FIFO before SM2 starts.
        pio_sm_put(PSRAM_PIO, sTxSm, w);
    }
    // FIFO/register setup happens with CE# high; only the autonomous PIO wire
    // phase counts against the PSRAM's 8 us maximum CE#-low interval.
    psramCeLow();
    psramTxSmRunAndWait();
    psramMuxToSio();
    psramCeHigh();
    psramPioTransactionEnd();
}

static void __no_inline_not_in_flash_func(psramPioReadBurst)(u32 addr, u8* buf, u32 len)
{
    // See psramPioWriteBurst: the transaction lock leaves cartridge IRQs on.
    psramPioTransactionBegin();
    psramMuxToPio();

    // The same SM emits the serial 0xEB command, quad address and six dummy
    // clocks without a mid-command GPIO mux handoff.
    psramTxSmStart(6 + PSRAM_RX_DUMMY_NIBBLES,
                   ((u32)PSRAM_CMD_FAST_READ_QUAD << 24) | (addr & 0x00FFFFFFu));
    pio_sm_put(PSRAM_PIO, sTxSm, 0);
    psramCeLow();
    psramTxSmRunAndWait();

    // First architecture test keeps the proven SIO sampler for returned data.
    // PIO leaves SCLK low after the final dummy falling edge, so the first SIO
    // rising edge samples the first data nibble normally.
    psramMuxToSio();
    sio_hw->gpio_oe_clr = PSRAM_IO_MASK;
    psramBbReadData(buf, len);
    psramCeHigh();
    psramPioTransactionEnd();
}
#endif

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

// Receive only the quad data phase of an already-open CE# transaction. Used
// by both the all-bit-bang reader and the PIO1 SM2 command architecture.
static void psramBbReadData(u8* buf, u32 len)
{
    for (u32 i = 0; i < len; i++)
    {
        u8 hi = psramBbRecvNibble();
        u8 lo = psramBbRecvNibble();
        buf[i] = (hi << 4) | lo;
    }
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

    psramBbReadData(buf, len);
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

static void psramReadDirect(u32 addr, void* buf, u32 len)
{
#if PSRAM_FORCE_BITBANG
    u8* dst = (u8*)buf;
    while (len > 0)
    {
        u32 burst = PSRAM_BB_BURST_BYTES - (addr & (PSRAM_BB_BURST_BYTES - 1));
        if (burst > len)
            burst = len;

        psramBbReadBurst(addr, dst, burst);
        addr += burst;
        dst += burst;
        len -= burst;
    }
#else
    // Cache/probe traffic is word aligned. Keep odd diagnostic accesses on the
    // all-bit-bang path so the PIO write FIFO never needs cross-call packing.
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
#endif
}

static void psramWriteDirect(u32 addr, const void* buf, u32 len)
{
#if PSRAM_FORCE_BITBANG
    const u8* src = (const u8*)buf;
    while (len > 0)
    {
        u32 burst = PSRAM_BB_BURST_BYTES - (addr & (PSRAM_BB_BURST_BYTES - 1));
        if (burst > len)
            burst = len;

        psramBbWriteBurst(addr, src, burst);
        addr += burst;
        src += burst;
        len -= burst;
    }
#else
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
#endif
}

static void psramSubmitCore1(u32 request, u32 addr, void* buf, u32 len)
{
    // Only the core0 main loop submits requests, and it never has more than
    // one cache drain in flight. PIO0_IRQ_0 may preempt this wait, but IRQ
    // handlers never call the PSRAM API.
    while (sCore1Request != PSRAM_C1_IDLE)
        tight_loop_contents();

    sCore1Addr = addr;
    sCore1Buf = buf;
    sCore1Len = len;
    __dmb();
    sCore1Request = request;
    __sev();

    while (sCore1Request != PSRAM_C1_DONE)
        __wfe();
    __dmb();
    sCore1Request = PSRAM_C1_IDLE;
}

bool psram_core1_async_write(u32 addr, const void* buf, u32 len)
{
    if (!sCore1ServiceReady || get_core_num() != 0 ||
        sCore1Request != PSRAM_C1_IDLE)
        return false;

    sCore1Addr = addr;
    sCore1Buf = (void*)buf;
    sCore1Len = len;
    __dmb();
    sCore1Request = PSRAM_C1_WRITE;
    __sev();
    return true;
}

bool psram_core1_async_finish(void)
{
    if (sCore1Request != PSRAM_C1_DONE)
        return false;
    __dmb();
    sCore1Request = PSRAM_C1_IDLE;
    return true;
}

bool psram_core1_async_idle(void)
{
    return sCore1Request == PSRAM_C1_IDLE;
}

void psram_read(u32 addr, void* buf, u32 len)
{
    if (sCore1ServiceReady && get_core_num() == 0)
    {
        psramSubmitCore1(PSRAM_C1_READ, addr, buf, len);
        return;
    }
    psramReadDirect(addr, buf, len);
}

void psram_write(u32 addr, const void* buf, u32 len)
{
    if (sCore1ServiceReady && get_core_num() == 0)
    {
        psramSubmitCore1(PSRAM_C1_WRITE, addr, (void*)buf, len);
        return;
    }
    psramWriteDirect(addr, buf, len);
}

bool psram_core1_service(void)
{
    // Called at the head of both core1 loops. Publishing readiness here keeps
    // standalone early core0 calls on the direct path until a consumer exists.
    if (!sCore1ServiceReady)
    {
        __dmb();
        sCore1ServiceReady = true;
    }

    u32 request = sCore1Request;
    if (request != PSRAM_C1_READ && request != PSRAM_C1_WRITE)
        return false;

    __dmb();

    // Service exactly one electrical burst per call. During game mode core1
    // also produces the cartridge scrambler stream; monopolising it for an
    // entire 512-byte cache line (about 166 us on the SIO backend) can empty
    // that ring and corrupt the next encrypted ROM response. The core1 loop
    // refills the ring between these short PSRAM bursts.
#if PSRAM_FORCE_BITBANG
    const u32 burstBytes = PSRAM_BB_BURST_BYTES;
#else
    // An unaligned diagnostic request will fall back to SIO inside the direct
    // helper. A 16-byte slice is still one safe SIO transaction in that case.
    const u32 burstBytes = sUsePio ? PSRAM_PIO_BURST_BYTES
                                   : PSRAM_BB_BURST_BYTES;
#endif
    u32 chunk = burstBytes - (sCore1Addr & (burstBytes - 1));
    if (chunk > sCore1Len)
        chunk = sCore1Len;

    if (request == PSRAM_C1_READ)
        psramReadDirect(sCore1Addr, sCore1Buf, chunk);
    else
        psramWriteDirect(sCore1Addr, sCore1Buf, chunk);

    sCore1Addr += chunk;
    sCore1Buf = (u8*)sCore1Buf + chunk;
    sCore1Len -= chunk;

    if (sCore1Len != 0)
        return true;

    __dmb();
    sCore1Request = PSRAM_C1_DONE;
    __sev();
    return true;
}

static void psramSendCmd(u8 cmd)
{
    psramClkLow();
    psramCeLow();
    psramSendByteSerial(cmd);
    psramCeHigh();
}

// Return the chip and bus to a known SPI-mode state. This is also required
// after a failed PIO qualification: merely changing the GPIO mux does not
// guarantee that the last malformed CE#-low transaction left the PSRAM command
// decoder on a byte boundary.
static void psramResetChipSio(void)
{
    psramMuxToSio();
    sio_hw->gpio_oe_clr = PSRAM_IO_MASK;
    psramCeHigh();
    psramClkLow();
    busy_wait_us(1);
    psramSendCmd(PSRAM_CMD_RESET_ENABLE);
    psramSendCmd(PSRAM_CMD_RESET);
    busy_wait_us(50);
}

// Presence/data-path probe at the start, middle and end of the address space.
// The PIO backend is stress-tested because a single successful transfer is not
// enough to qualify a timing-sensitive pump; any PIO mismatch forces the
// known-good bit-bang fallback. Bit-bang retains retries for presence testing.
static bool psramProbe(void)
{
    u32 rounds = 1;
    int attempts = 3;
#if !PSRAM_FORCE_BITBANG
    if (sUsePio)
    {
        rounds = PSRAM_PIO_PROBE_ROUNDS;
        attempts = 1;
    }
#endif

    for (u32 round = 0; round < rounds; round++)
    {
        for (u32 addr = 0; addr < PSRAM_SIZE_BYTES;
             addr += (PSRAM_SIZE_BYTES / 2) - PSRAM_BB_BURST_BYTES)
        {
            u8 pattern[PSRAM_BB_BURST_BYTES];
            u8 readBack[PSRAM_BB_BURST_BYTES];
            for (u32 i = 0; i < sizeof(pattern); i++)
                pattern[i] = (u8)(addr + i * 0x9Du + 0x35u + round * 0x53u);

            bool ok = false;
            for (int attempt = 0; attempt < attempts && !ok; attempt++)
            {
                psramWriteDirect(addr, pattern, sizeof(pattern));
                psramReadDirect(addr, readBack, sizeof(readBack));
                ok = memcmp(pattern, readBack, sizeof(pattern)) == 0;
            }
            if (!ok)
            {
                LOG("PSRAM: probe FAILED round=%lu @0x%08lX wrote[0:4]=",
                    (unsigned long)round, (unsigned long)addr);
                for (u32 i = 0; i < 4; i++) LOG("%02X", pattern[i]);
                LOG(" read[0:4]=");
                for (u32 i = 0; i < 4; i++) LOG("%02X", readBack[i]);
                LOG("\n");
#if !PSRAM_FORCE_BITBANG
                if (sUsePio)
                {
                    // pioW->bbR validates complete PIO writes. The reciprocal
                    // test validates the new PIO command/address phase followed
                    // by the proven bit-bang data sampler.
                    u8 xbuf[8];
                    psramBbReadBurst(addr, xbuf, 8);
                    LOG("PSRAM: pioW->bbR[0:8]=");
                    for (u32 i = 0; i < 8; i++) LOG("%02X", xbuf[i]);
                    LOG("\n");
                    psramBbWriteBurst(addr, pattern, 8);
                    memset(xbuf, 0, 8);
                    psramPioReadBurst(addr, xbuf, 8);
                    LOG("PSRAM: bbW->pioCmdBbR[0:8]=");
                    for (u32 i = 0; i < 8; i++) LOG("%02X", xbuf[i]);
                    LOG("\n");
                }
#endif
                return false;
            }
        }
    }
    return true;
}

// Hardware init only: GPIO and reset. No bursts and no probe.
// Safe during boot. Uses busy_wait_us (not sleep_us): this may run on core1
// which has interrupts disabled, where sleep_us's WFI would hang.
void psram_init_hw(void)
{
    gpio_init_mask(PSRAM_PIN_MASK);

    // CE# and CLK are outputs, the data pins are inputs until a transfer starts.
    gpio_put(PSRAM_PIN_CE, true);
    gpio_put(PSRAM_PIN_CLK, false);
    gpio_set_dir_out_masked(PSRAM_CE_MASK | PSRAM_CLK_MASK);

    // Fast GPIO edges also keep the bit-bang clock transitions well defined.
    for (uint pin = PSRAM_PIN_IO0; pin <= PSRAM_PIN_CLK; pin++)
    {
        gpio_set_slew_rate(pin, GPIO_SLEW_RATE_FAST);
        gpio_set_drive_strength(pin, GPIO_DRIVE_STRENGTH_8MA);
    }

    // The device needs 150 us after power-up before it accepts commands.
    busy_wait_us(200);

    // Software reset (RSTEN must be immediately followed by RST).
    psramResetChipSio();

#if PSRAM_FORCE_BITBANG
    // Production path: keep every PSRAM pin on SIO. The PSRAM PIO program is
    // not generated or linked, and PIO1 SM2 remains unclaimed.
    psramMuxToSio();
    LOG("PSRAM: hw init done (bit-bang ready)\n");
#else
    if (!sPioTransactionLock)
        sPioTransactionLock = spin_lock_instance(spin_lock_claim_unused(true));
    sUsePio = psramPioInit();
    LOG("PSRAM: hw init done (%s ready)\n", sUsePio ? "PIO1 SM2" : "bit-bang");
#endif
}

// Probe with bursts. Must NOT run during NDS boot (bursts block/stall the
// boot command stream). Call after boot from the main loop.
bool psram_probe(void)
{
#if PSRAM_FORCE_BITBANG
    LOG("PSRAM: probing (bit-bang path, core1 runtime service)...\n");
    if (psramProbe())
    {
        LOG("PSRAM: bit-bang data path (uninterrupted core1 service)\n");
        return true;
    }
    return false;
#else
    if (sUsePio)
        LOG("PSRAM: probing (PIO1 SM2 command/write + SIO read, 128-round stress)...\n");
    else
        LOG("PSRAM: probing (bit-bang path)...\n");
    if (psramProbe())
    {
        if (sUsePio)
            LOG("PSRAM: PIO1 SM2 command/write + SIO read data path (core1 service)\n");
        else
            LOG("PSRAM: bit-bang data path\n");
        return true;
    }

    // The PIO pump failing where bit-banging works points at a pump/timing
    // bug, not a missing chip - retry with bit-bang so the cache still works.
    if (sUsePio)
    {
        LOG("PSRAM: PIO probe failed, retrying with bit-bang...\n");
        dspicoPioSmSetEnabled(PSRAM_PIO, sTxSm, false);
        sUsePio = false;
        // The failed write may have ended with a malformed command/address
        // phase. Reset the chip before trusting the bit-bang fallback; a mux
        // switch alone cannot repair the PSRAM command decoder state.
        psramResetChipSio();
        if (psramProbe())
        {
            LOG("PSRAM: bit-bang data path\n");
            return true;
        }
    }
    return false;
#endif
}

bool psram_init(void)
{
    psram_init_hw();
    return psram_probe();
}
