// dspico-debug: RP2354A PSRAM loop-test + one-shot MicroSD detection firmware.
//
// Hardware: RP2354A NDS flash-cart board (see references/hardware/RP2354A_GPIO分配总结.md).
//   - PSRAM  4x APS6404L-3SQR 8 MiB QSPI = 32 MiB total, shared SCLK/SIO bus
//            on GPIO0 (SCLK) + GPIO22-25 (SIO0-3) + GPIO26-29 (CE0-3#)
//   - MicroSD SIM1 6-line SDIO on GPIO3-8 (software/PIO driven)
//   - Debug UART1 muxed onto the Bank-1 USB_DP(TX)/USB_DM(RX) pads @ 115200
//
// Behaviour:
//   1. At boot, mount the SD card once and report filesystem type, total/free
//      capacity and the root-directory file list.
//   2. Initialise the PSRAM array and continuously loop-test it: every pass
//      writes a pattern across all 32 MiB then reads it all back and verifies,
//      cycling through solid/pseudo-random patterns to catch stuck-at,
//      coupling and address-decoding faults.
//   3. A heartbeat on the UART every 3 s prints the live PSRAM test progress
//      together with the captured SD status.
//
// This firmware does not touch dspico-firmware; it links the same unchanged
// PSRAM/SDIO/SdCard/FatFs driver sources (copied under src/).

#include "common.h"
#include "psram.h"
#include "sd/fatfs/ff.h"
#include "sd/fatfs/diskio.h"
#include "hardware/clocks.h"
#include "hardware/structs/io_qspi.h"
#include "hardware/regs/io_qspi.h"
#include "hardware/structs/usb.h"
#include "hardware/regs/usb.h"
#include "pico/stdio_uart.h"
#include "pico/binary_info.h"

#include <stdio.h>
#include <string.h>

#include "hardware/structs/scb.h"

// dspico-debug: distinguish a hard fault from a live hang. The default handler
// just spins silently, which looks exactly like a wedged driver on the UART.
extern "C" __attribute__((naked)) void isr_hardfault(void)
{
    __asm volatile(
        "movs r0, #4\n"
        "mov r1, lr\n"
        "tst r0, r1\n"
        "beq 1f\n"
        "mrs r0, psp\n"
        "b 2f\n"
        "1: mrs r0, msp\n"
        "2: b hardfault_c\n");
}

extern "C" void hardfault_c(u32 *sp)
{
    printf("\n!!! HARDFAULT pc=0x%08lX lr=0x%08lX xpsr=0x%08lX\n",
           (unsigned long)sp[6], (unsigned long)sp[5], (unsigned long)sp[7]);
    while (1)
    {
        busy_wait_ms(1000);
        printf("!!! still in hardfault\n");
    }
}

SdCard gSdCard;          // defined here; declared extern in common.h
static FATFS sFatFs;

// FatFs R0.15 omits an AM_VOL macro; the volume-label attribute bit is 0x08.
#define AM_VOL 0x08u

// ---------------------------------------------------------------------------
// SD detection results (captured once, replayed by the heartbeat)
// ---------------------------------------------------------------------------
static bool   sSdMounted;
static TCHAR  sSdLabel[34];
static char   sSdFsType[8];     // "FAT32", "exFAT", ...
static u64    sSdTotalBytes;
static u64    sSdFreeBytes;
static u32    sSdFileCount;
static u32    sSdDirCount;
static u32    sSdListed;                              // names stored
static char   sSdNames[16][40];                       // first 16 root entries
static char   sSdSummary[96];                          // compact one-line status

// ---------------------------------------------------------------------------
// PSRAM test state (written from the main loop, read by the heartbeat)
// ---------------------------------------------------------------------------
#define PSRAM_BLOCK_BYTES    256u                       // 256-aligned -> two 128B PIO bursts
#define PSRAM_BLOCKS_PER_CHIP (PSRAM_CHIP_SIZE_BYTES / PSRAM_BLOCK_BYTES)
#define PSRAM_TOTAL_BLOCKS   (PSRAM_SIZE_BYTES / PSRAM_BLOCK_BYTES)

static struct {
    volatile u32  pass;
    volatile u32  errors;
    volatile u32  lastErrAddr;
    volatile u32  curBlock;        // blocks tested so far in the current phase
    volatile u32  totalBlocks;     // blocks per phase = good chips * BLOCKS_PER_CHIP
    volatile u32  phase;           // 0 = WRITE, 1 = VERIFY
    volatile u32  addr;            // address of the block being worked on
    u8            chipMask;        // chips the loop test covers (dead chips skipped)
    bool          present;         // result of psram_init() self-test
} sPsram;

static u8 sWrBuf[PSRAM_BLOCK_BYTES] __attribute__((aligned(4)));
static u8 sRdBuf[PSRAM_BLOCK_BYTES] __attribute__((aligned(4)));

// Pattern for a given pass/address. Solid patterns (0x00/0xFF/0x55/0xAA) stress
// stuck-at cells; the two address-dependent pseudo-random passes exercise every
// cell differently so address-aliasing and inter-cell coupling surface.
static inline u8 psramPatternByte(u32 pass, u32 addr)
{
    switch (pass % 6u)
    {
        case 0:  return 0x00u;
        case 1:  return 0xFFu;
        case 2:  return 0x55u;
        case 3:  return 0xAAu;
        case 4:  { u32 x = (addr * 0x01000193u) ^ 0xA5A5A5A5u; x ^= x >> 16; x *= 0x7F4A7C15u; x ^= x >> 8;  return (u8)x; }
        default: { u32 x = (addr * 0x01000193u) ^ 0x5A5A5A5Au; x ^= x >> 16; x *= 0x7F4A7C15u; x ^= x >> 8;  return (u8)x; }
    }
}

// First testable address: base of the lowest-numbered responding chip.
static u32 psramFirstGoodAddr(void)
{
    for (u32 chip = 0; chip < PSRAM_CHIP_COUNT; chip++)
        if (sPsram.chipMask & (1u << chip))
            return chip * PSRAM_CHIP_SIZE_BYTES;
    return 0; // unreachable: the loop test only runs when a chip responds
}

// One test step: write or verify a single block, advancing the pass/phase state.
// Dead chips are skipped a whole 8 MiB region at a time, so the loop only ever
// touches memory that answered the init self-test.
static void psramTestStep(void)
{
    const u32 addr = sPsram.addr;

    if (sPsram.phase == 0) // WRITE: fill the tested region, then verify
    {
        for (u32 i = 0; i < PSRAM_BLOCK_BYTES; i++)
            sWrBuf[i] = psramPatternByte(sPsram.pass, addr + i);
        psram_write(addr, sWrBuf, PSRAM_BLOCK_BYTES);
    }
    else                   // VERIFY: read back and compare against the pattern
    {
        psram_read(addr, sRdBuf, PSRAM_BLOCK_BYTES);
        for (u32 i = 0; i < PSRAM_BLOCK_BYTES; i++)
        {
            if (sRdBuf[i] != psramPatternByte(sPsram.pass, addr + i))
            {
                // One counted error per failing block keeps the count bounded
                // even when the device is absent (every block mismatches).
                sPsram.errors++;
                sPsram.lastErrAddr = addr + i;
                break;
            }
        }
    }

    // Advance to the next block, skipping dead chips' 8 MiB regions wholesale.
    u32 next = addr + PSRAM_BLOCK_BYTES;
    while (next < PSRAM_SIZE_BYTES && !(sPsram.chipMask & (1u << PSRAM_CHIP_OF(next))))
        next = (PSRAM_CHIP_OF(next) + 1u) * PSRAM_CHIP_SIZE_BYTES;

    sPsram.curBlock++;
    if (next >= PSRAM_SIZE_BYTES)
    {
        next = psramFirstGoodAddr();
        sPsram.curBlock = 0;
        if (sPsram.phase == 0)
            sPsram.phase = 1;        // written the whole region -> verify it
        else
        {
            sPsram.phase = 0;        // verified -> next pass with a new pattern
            sPsram.pass++;
        }
    }
    sPsram.addr = next;
}

// ---------------------------------------------------------------------------
// SD card detection (run once at boot)
// ---------------------------------------------------------------------------

static const char* fsTypeName(BYTE t)
{
    switch (t)
    {
        case FS_FAT12: return "FAT12";
        case FS_FAT16: return "FAT16";
        case FS_FAT32: return "FAT32";
        case FS_EXFAT: return "exFAT";
        default:       return "?";
    }
}

static void formatGiB(u64 bytes, char* out, size_t n)
{
    u64 gib  = bytes >> 30;
    u64 frac = ((bytes & ((1ull << 30) - 1)) * 100ull) >> 30; // 2 decimal places, no float
    snprintf(out, n, "%llu.%02llu GiB (%llu MiB)", gib, frac, bytes >> 20);
}

static void sdDetectOnce(void)
{
    memset(&sFatFs, 0, sizeof(sFatFs));
    sSdMounted = false;
    sSdTotalBytes = sSdFreeBytes = 0;
    sSdFileCount = sSdDirCount = sSdListed = 0;
    sSdLabel[0] = 0;
    sSdFsType[0] = 0;

    printf("\n[SD] Detecting card...\n");

    FRESULT fr = FR_NOT_READY;
    for (int attempt = 0; attempt < 3 && fr != FR_OK; attempt++)
    {
        printf("[SD] f_mount attempt %d...\n", attempt + 1);
        fr = f_mount(&sFatFs, "0:", 1);
        printf("[SD] f_mount returned fr=%d fs_type=%u\n", (int)fr, (unsigned)sFatFs.fs_type);
    }

    if (fr != FR_OK)
    {
        const char* why = (fr == FR_NOT_READY)      ? "no card / init failed"
                        : (fr == FR_NO_FILESYSTEM)   ? "card present, no filesystem"
                        : "mount error";
        snprintf(sSdSummary, sizeof(sSdSummary), "no mount (%s)", why);
        printf("[SD] %s (fr=%d)\n", why, (int)fr);
        return;
    }

    sSdMounted = true;
    snprintf(sSdFsType, sizeof(sSdFsType), "%s", fsTypeName(sFatFs.fs_type));

    // Capacity: total = clusters * cluster_sectors * 512; free via f_getfree.
    DWORD freeClust = 0;
    FATFS* fs = &sFatFs;
    FRESULT gfr = f_getfree("0:", &freeClust, &fs);
    u32 csize = sFatFs.csize ? sFatFs.csize : 0;
    u32 totalClust = (sFatFs.n_fatent >= 2) ? (sFatFs.n_fatent - 2) : 0;
    sSdTotalBytes = (u64)totalClust * csize * 512ull;
    sSdFreeBytes  = (gfr == FR_OK) ? (u64)freeClust * csize * 512ull : 0;

    // Root directory listing. The volume label shows up here as an AM_VOL
    // entry, so it is captured without needing f_getlabel() (which this
    // FatFs config compiles out).
    DIR dir;
    FILINFO fno;
    if (f_opendir(&dir, "0:") == FR_OK)
    {
        while (f_readdir(&dir, &fno) == FR_OK && fno.fname[0] != 0)
        {
            if (fno.fattrib & AM_VOL)
            {
                snprintf(sSdLabel, sizeof(sSdLabel), "%s", fno.fname);
                continue;
            }
            if (fno.fattrib & AM_DIR)
                sSdDirCount++;
            else
                sSdFileCount++;

            if (sSdListed < 16)
            {
                snprintf(sSdNames[sSdListed], sizeof(sSdNames[sSdListed]),
                         "%c%s", (fno.fattrib & AM_DIR) ? 'D' : 'F', fno.fname);
                sSdListed++;
            }
        }
        f_closedir(&dir);
    }

    // Compact summary reused by every heartbeat.
    snprintf(sSdSummary, sizeof(sSdSummary), "%s %llu/%llu MiB ok F=%lu D=%lu",
             sSdFsType, sSdFreeBytes >> 20, sSdTotalBytes >> 20,
             sSdFileCount, sSdDirCount);

    // Full one-time report.
    char totalStr[48], freeStr[48];
    formatGiB(sSdTotalBytes, totalStr, sizeof(totalStr));
    formatGiB(sSdFreeBytes,  freeStr,  sizeof(freeStr));
    printf("[SD] Mounted: %s  label='%s'\n",
           sSdFsType, sSdLabel[0] ? sSdLabel : "(none)");
    printf("[SD] Cluster size: %lu sectors. Total: %s. Free: %s.\n",
           (u32)csize, totalStr, freeStr);
    printf("[SD] Root entries: %lu files, %lu directories.\n",
           sSdFileCount, sSdDirCount);
    if (sSdListed)
    {
        printf("[SD] First %lu root entries:\n", sSdListed);
        for (u32 i = 0; i < sSdListed; i++)
            printf("       %s\n", sSdNames[i]);
        if (sSdFileCount + sSdDirCount > sSdListed)
            printf("       ... (%lu more)\n", sSdFileCount + sSdDirCount - sSdListed);
    }
    printf("[SD] OK\n");
}

// ---------------------------------------------------------------------------
// Heartbeat (polled from the main loop to avoid touching the TIMER0_IRQ_0
// that the SdCard driver owns for its sequential-read alarm)
// ---------------------------------------------------------------------------
static void printHeartbeat(u32 idx, u32 elapsedMs)
{
    const char* phase = (sPsram.phase == 0) ? "WR" : "RV";
    u32 pct = sPsram.totalBlocks ? (sPsram.curBlock * 100u) / sPsram.totalBlocks : 0;

    printf("[HB #%lu %lums] PSRAM %s chips=0x%X pass=%lu %s blk=%lu/%lu (%lu%%) errs=%lu",
           (u32)idx, elapsedMs,
           sPsram.present ? "OK" : "FAIL",
           sPsram.chipMask,
           (u32)sPsram.pass, phase,
           (u32)sPsram.curBlock, (u32)sPsram.totalBlocks, pct, (u32)sPsram.errors);
    if (sPsram.errors)
        printf(" last@0x%08lX", (u32)sPsram.lastErrAddr);
    printf(" | SD: %s\r\n", sSdSummary);
}

// ---------------------------------------------------------------------------
// Debug UART: UART1 on the Bank-1 USB_DP/USB_DM pads (GPIO分配总结 §6).
//
// These pads belong to the USB PHY. Three things stand between the FUNCSEL
// mux and an actual signal on the wire:
//
//   1. PHY_ISO (USB MAIN_CTRL bit 2) resets to 1: the PHY - pads included -
//      is isolated from the switched core power domain. Datasheet §12.7:
//      "the MAIN_CTRL.PHY_ISO bit needs to be cleared before the PHY can be
//      used."
//   2. The SDK's runtime_init_usb_power_down() runs before main() and sets
//      SIE_CTRL.TRANSCEIVER_PD (transceiver power-down) when USB is unused,
//      which is our case. SIE_CTRL.PULLDOWN_EN also resets to 1, loading
//      DP/DM with the PHY's pull-downs.
//   3. USB_MUXING.TO_PHY resets to 1, connecting the pads to the (unused)
//      USB controller instead of the digital function path. USBPHY_AS_GPIO
//      reroutes them to the IO_QSPI function mux.
//
// Only then does the IO_QSPI USBPHY_DP/DM_CTRL FUNCSEL = 0x02
// (UART1_TX/UART1_RX) actually reach the pads. stdio_init_all() is NOT used:
// it would also mux the board-default UART pins (GPIO0/1 on the pico2
// header), and GPIO0 is the PSRAM SCLK on this board.
// ---------------------------------------------------------------------------
static void debugUartInit(void)
{
    // 1. Release the PHY from power-domain isolation.
    hw_clear_bits(&usb_hw->main_ctrl, USB_MAIN_CTRL_PHY_ISO_BITS);
    // 2. Power the transceiver back up (cleared by the SDK's USB power-down)
    //    and drop the default DP/DM pull-downs so they don't fight the UART.
    hw_clear_bits(&usb_hw->sie_ctrl,
                  USB_SIE_CTRL_TRANSCEIVER_PD_BITS | USB_SIE_CTRL_PULLDOWN_EN_BITS);
    // 3. Connect the pads to the digital function mux, not the USB SIE.
    usb_hw->muxing = USB_USB_MUXING_USBPHY_AS_GPIO_BITS;
    // 4. Route UART1 onto the pads.
    hw_write_masked(&io_qspi_hw->usbphy_dp_ctrl,
                    IO_QSPI_USBPHY_DP_CTRL_FUNCSEL_VALUE_UART1_TX
                        << IO_QSPI_USBPHY_DP_CTRL_FUNCSEL_LSB,
                    IO_QSPI_USBPHY_DP_CTRL_FUNCSEL_BITS);
    hw_write_masked(&io_qspi_hw->usbphy_dm_ctrl,
                    IO_QSPI_USBPHY_DM_CTRL_FUNCSEL_VALUE_UART1_RX
                        << IO_QSPI_USBPHY_DM_CTRL_FUNCSEL_LSB,
                    IO_QSPI_USBPHY_DM_CTRL_FUNCSEL_BITS);
    // Skip pin muxing inside the SDK (-1 = no Bank 0 GPIO); the pads above
    // were muxed by hand.
    stdio_uart_init_full(uart1, 115200, -1, -1);
}

// ---------------------------------------------------------------------------
// Early GPIO setup for the SDIO pins (matches the firmware's expectations:
// CLK low before init, external 10k pull-ups present so internal pulls off)
// ---------------------------------------------------------------------------
static void sdioPinInit(void)
{
    gpio_init_mask(SDIO_PIN_MASK);
    gpio_disable_pulls(SDIO_CLK);
    gpio_disable_pulls(SDIO_CMD);
    gpio_disable_pulls(SDIO_D0);
    gpio_disable_pulls(SDIO_D1);
    gpio_disable_pulls(SDIO_D2);
    gpio_disable_pulls(SDIO_D3);
    gpio_set_drive_strength(SDIO_CLK, GPIO_DRIVE_STRENGTH_2MA);
    gpio_put(SDIO_CLK, false);
    gpio_set_dir(SDIO_CLK, true);
}

int main(void)
{
    bi_decl(bi_program_description("dspico-debug: PSRAM loop-test + SD detect"));

    // 200 MHz system clock, matching the production firmware. The SDIO driver's
    // init timing (74-card-cycles delay, 400 kHz / 25 MHz clocks) is calibrated
    // for this clock; the PSRAM PIO pump runs at clkdiv 3 (SCLK = sysclk/6 TX,
    // sysclk/12 RX). RP2354 is rated 150 MHz; 200 MHz is a mild, commonly used
    // overclock kept so the reused drivers' timings are unchanged.
    set_sys_clock_pll(1200000000, 6, 1);

    debugUartInit();
    sleep_ms(150); // let the host UART settle

    printf("\n========================================\n");
    printf("  dspico-debug PSRAM + SD test firmware\n");
    printf("  RP2354A  %d MiB PSRAM (4x APS6404L, GPIO0/22-29)\n", PSRAM_SIZE_BYTES / (1024 * 1024));
    printf("  MicroSD SDIO          (GPIO3-8)\n");
    printf("  UART1 on USB_DP/DM    115200 8N1\n");
    printf("========================================\n");

    // SD: one-shot detection (uses pio1 + DMA 2/3).
    sdioPinInit();
    sdDetectOnce();

    // PSRAM: init + self-test (uses pio0). Falls back to bit-bang if no SM free.
    printf("\n[PSRAM] Initialising %d MiB PSRAM (4x APS6404L)...\n",
           PSRAM_SIZE_BYTES / (1024 * 1024));
    sPsram.present = psram_init();
    sPsram.chipMask = psram_chip_ok_mask();
    sPsram.totalBlocks = (u32)__builtin_popcount(sPsram.chipMask) * PSRAM_BLOCKS_PER_CHIP;
    sPsram.addr = sPsram.present ? psramFirstGoodAddr() : 0;
    if (sPsram.present)
    {
        printf("[PSRAM] Init OK, chips responding: 0x%X (%d/%d). Loop test covers %u MiB "
               "(block=%u B, %lu blocks/pass).\n",
               sPsram.chipMask, __builtin_popcount(sPsram.chipMask), PSRAM_CHIP_COUNT,
               __builtin_popcount(sPsram.chipMask) * (PSRAM_CHIP_SIZE_BYTES >> 20),
               PSRAM_BLOCK_BYTES, (u32)sPsram.totalBlocks);
        if (sPsram.chipMask != (u8)((1u << PSRAM_CHIP_COUNT) - 1u))
            printf("[PSRAM] NOTE: dead chip region(s) excluded from the loop test.\n");
    }
    else
        printf("[PSRAM] No chip responds - loop test DISABLED. Pin probe mode (multimeter aid).\n");

    // Heartbeat loop: test PSRAM continuously, report every 3 s.
    const u32 heartIntervalMs = 3000;
    u32 hbIndex = 0;
    u32 lastBeat = millis();
    u32 startMs  = millis();

    while (1)
    {
        // With a working PSRAM: loop-test it. Without one: walk a static probe
        // level across the PSRAM pins instead, so each U2 pad can be checked
        // with a multimeter (the failing writes would tell us nothing new).
        if (sPsram.present)
            psramTestStep();
        else
            psram_pin_probe_step();

        u32 now = millis();
        if ((u32)(now - lastBeat) >= heartIntervalMs)
        {
            lastBeat = now;
            printHeartbeat(++hbIndex, now - startMs);
        }
    }
}
