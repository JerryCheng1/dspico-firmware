# dspico-debug - RP2354A PSRAM loop-test + SD detect firmware

A standalone test firmware for the **RP2354A** NDS flash-cart board
(see `references/hardware/RP2354A_GPIO分配总结.md`, netlist
`Netlist_Schematic1_1_2026-08-18.tel`). It does **not** build or modify
`dspico-firmware`; it reuses that project's PSRAM / SDIO / SdCard / FatFs
driver sources (copied under `src/`, PSRAM driver ported to the 4-chip bus).

Ported from the original RP2040/DSPICOwithPSRAM board (GPIO22-26/29 single
8 MiB PSRAM, UART on GPIO0/1) to the RP2354A pin map:

| Function | RP2354A pins |
|---|---|
| PSRAM SCLK | GPIO0 (R12 22R) |
| PSRAM SIO0-3 | GPIO22-25 |
| PSRAM CE0-3# (U2/U4/U5/U6) | GPIO26-29 (4.7k pull-ups) |
| MicroSD 4-bit SDIO | GPIO3-8 (unchanged) |
| Debug UART1 | Bank-1 USB_DP(TX)/USB_DM(RX) pads, FUNCSEL 0x02, 115200 8N1 |
| NDS cartridge bus | GPIO9-21 (untouched by this firmware) |

## What it does

1. **SD card - one-shot detection at boot**
   - Mounts the FAT volume (`f_mount`, retried 3×).
   - Reports filesystem type (FAT12/16/32/exFAT), volume label, cluster size,
     total and free capacity (GiB + MiB).
   - Lists the root directory (file/dir counts + the first 16 entry names).
   - Result is stored and replayed by the heartbeat.
2. **PSRAM - continuous loop test**
   - 4× APS6404L (32 MiB total) on the shared SCLK/SIO bus, driven via the
     PIO QSPI pump on GPIO0 + GPIO22-25; the target chip is selected by the
     address range (chip = addr / 8 MiB) through its CE# on GPIO26-29.
   - Each pass writes a pattern across the whole 32 MiB array, then reads it
     all back and verifies. Patterns cycle through `0x00 / 0xFF / 0x55 / 0xAA`
     (stuck-at) and two address-dependent pseudo-random passes
     (aliasing/coupling).
   - 256-byte blocks (two 128-byte PIO bursts each), 131072 blocks per pass.
   - `psram_init()` runs a per-chip reset, EID ('h9F) read, CE-driver test and
     self-test (start/middle/end of every chip), with bit-bang and 1-bit SPI
     cross-checks kept from the original bring-up driver.
3. **Heartbeat - every 3 s on UART1 (USB_DP/DM pads, 115200 8N1)**
   - Live PSRAM progress: pass, phase (WR/RV), block/total, %, error count and
     the last failing address.
   - Compact SD status captured at boot.

## UART output example

```
========================================
  dspico-debug PSRAM + SD test firmware
  RP2354A  32 MiB PSRAM (4x APS6404L, GPIO0/22-29)
  MicroSD SDIO          (GPIO3-8)
  UART1 on USB_DP/DM    115200 8N1
========================================

[SD] Detecting card...
[SD] Mounted: FAT32  label='SANDISK'
[SD] Cluster size: 64 sectors. Total: 7.45 GiB (7623 MiB). Free: 5.12 GiB (5243 MiB).
[SD] Root entries: 12 files, 2 directories.
[SD] OK

[PSRAM] Initialising 32 MiB PSRAM (4x APS6404L)...
PSRAM: PIO data path
[PSRAM] Init OK. Loop test running (block=256 B, 131072 blocks/pass).
[HB #1 3001ms] PSRAM OK pass=0 RV blk=24112/131072 (18%) errs=0 | SD: FAT32 5243/7623 MiB ok F=12 D=2
```

With no card / no PSRAM fitted the heartbeat still runs (see `main.cpp`).

## Build

Requirements: `arm-none-eabi-gcc`, `cmake`, the pico-sdk (2.3.0), and a
populated `picotool` (2.3.0, installed to `~/.local`, so the build works
offline).

```bash
./compile.sh
# -> build/DSpico_debug.uf2
```

`compile.sh` honours this env override (default shown):

```
PICO_SDK_PATH=/home/jerry/pico-sdk-2.3.0
```

Build configuration (set in `CMakeLists.txt`):

- `PICO_PLATFORM=rp2350-arm-s` - Cortex-M33 application core.
- `PICO_BOARD=pico2` - board header for SDK boilerplate only; all pins are
  assigned explicitly by the firmware. Its default boot stage 2
  (`boot2_w25q080`) explicitly supports the RP2354's in-package W25Q16JV.

The image is a normal flash XIP build (linked at `0x10000000`): RP2354A
contains a stacked 2 MB Winbond W25Q16JV on the dedicated QSPI pads
(datasheet §14.3) and boots from it exactly like an RP2350 with external
flash.

## Flash

Hold **BOOTSEL** while powering the board (or `picotool reboot -fu`) and copy
`build/DSpico_debug.uf2` to the mounted RPI-RP2 mass-storage volume, or use
`picotool load build/DSpico_debug.uf2 -x`. The firmware XIPs from the
in-package flash at `0x10000000` and **survives power cycles** (reload only
to update it). The debug output is on the UART1 pads (U1.52 = TX / USB_DP,
U1.51 = RX / USB_DM); USB is unavailable while those pads carry the UART.

## Files

```
src/main.cpp            test firmware (SD detect + PSRAM loop + heartbeat)
src/common.h            trimmed, API-compatible common header (+RP2350 shims)
src/psram.*  src/psram.pio   4-chip APS6404L driver (ported from dspico-firmware)
src/sd/rp2350_sdio.*    src/sd/rp2350_sdio.pio   SDIO PIO driver
src/sd/SdCard.*         high-level SD state machine + capacity
src/sd/SdCardInfo.h     CSD/CID register definitions
src/sd/fatfs/*          FatFs R0.15 + diskio glue
CMakeLists.txt          build definition (rp2350-arm-s, XIP flash image)
compile.sh              configure + build
```
