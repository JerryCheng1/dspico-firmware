# DSpico Firmware
This is the repository for the DSpico firmware. The firmware emulates a DS cartridge, with extended features for SD access. PIO is used for an SDIO interface for the SD card and for interfacing the DS cartridge bus.

For an overview of the supported card commands, see [commands.md](docs/commands.md).

## Features
- Emulates a retail DS(i) cartridge
- Interfaces with an SD card using SDIO and exposes card commands to access it from the DS side
- Can emulate an R4 to support software such as the Wood R4 kernel
- Supports having a separate rom for DS and DSi/3DS systems
- Supports emulating the IS-SPI-USB-ADAPTER for the WRFUxxed exploit
- Optional ROM/SD sector cache in external APS6404 PSRAM, speeding up repeated reads (e.g. the loader's FAT/directory sectors)
- Optimized for minimal power use when idle

## Pinout
| **Peripheral**  | **Pin name - Peripheral** | **Pin name - RP2040** |
|-------------|-----------------------|-------------------|
| **DS Slot** | D0                    | GPIO12            |
|             | D1                    | GPIO13            |
|             | D2                    | GPIO14            |
|             | D3                    | GPIO15            |
|             | D4                    | GPIO16            |
|             | D5                    | GPIO17            |
|             | D6                    | GPIO18            |
|             | D7                    | GPIO19            |
|             | CLK_DS                | GPIO11            |
|             | ROM_CS                | GPIO10            |
|             | SPI_CS                | GPIO21            |
|             | IRQ                   | GPIO20            |
|             | RST_DS                | GPIO09            |
| **SDIO**    | CLK_SD                | GPIO03            |
|             | DAT0                  | GPIO05            |
|             | DAT1                  | GPIO06            |
|             | DAT2                  | GPIO07            |
|             | DAT3                  | GPIO08            |
|             | CMD                   | GPIO04            |
| **PSRAM**   | IO0                   | GPIO22            |
|             | IO1                   | GPIO23            |
|             | IO2                   | GPIO24            |
|             | IO3                   | GPIO25            |
|             | SCLK                  | GPIO26            |
|             | CE                    | GPIO29            |

## Setup & configuration
We recommend using WSL (Windows Subsystem for Linux), or a Unix-based machine to compile this repository.
The steps provided will assume a Linux environment. Alternatively, you can run the `setup_environment.sh` bash script.

 1. Run `sudo  apt  update && sudo  apt  install  cmake  gcc-arm-none-eabi  build-essential  git`
 2. Clone this repository
 3. Run the following commands:
    ```bash
    git submodule update --init
    cd pico-sdk
    git submodule update --init
    cd ..
    ```
    Note that you shouldn't use `--recursive` because it draws in a lot of unnecessary submodules inside the pico-sdk.

### CMake options & compile-time defines
The firmware is configured through CMake `option()`s (set on the `cmake` command line with `-D`, e.g. `-DENABLE_UART_LOG=ON`) and a few `add_compile_definitions` / `#ifndef`-guarded macros. The CMake options are read from `CMakeLists.txt`; the source-level switches have sensible defaults and rarely need changing.

#### CMake options (`CMakeLists.txt`)
   * `ENABLE_R4_MODE` *(on by default)* - Enables R4 emulation. This allows you to use R4 software, such as the Wood R4 kernel. As R4 emulation can be used together with regular DSpico software, it can usually be kept enabled.
      * Note that to be able to use R4 software, your SD card must be at most 4 GB, or have a single partition in the first 4 GB of the SD card. R4 card commands cannot address SD sectors above 4 GB!
   * `ENABLE_PSRAM_CACHE` *(on by default)* - Builds in the external APS6404L PSRAM SD-sector cache. Requires the PSRAM fitted on GPIO22-26/29. The cache is a direct-mapped 512 B/line, 16384-line (8 MB) table on the E3/E5 SD-sector read path used by pico-loader. Each line has a 16-bit SRAM integrity digest; a mismatch invalidates the line and falls back to physical SD before data is served. Turn off for a build that never touches the PSRAM.
   * `ENABLE_UART_LOG` *(off by default)* - Queues boot, PSRAM probe and cache hit/miss logs into a 2 KB SRAM ring and drains only available UART FIFO space from the main loop. This avoids the former blocking `printf` pauses that starved SD updates during loader mount. Logging still costs some CPU and keeps the power-saving clock changes off, so leave it off for daily-use builds.
   * `PSRAM_USE_PIO` *(off by default)* - Experimental PIO1 SM2 PSRAM command/write pump. SDIO RX and TX dynamically share instructions 5..13, leaving 0..4 for the five-word PSRAM program; the NDS cartridge remains alone on PIO0. Reads use the PIO command/dummy phase and the proven SIO data sampler. A complete 16-byte write is prefilled into the joined TX FIFO before SM2 starts. All runtime PSRAM transactions execute uninterrupted on core1, including the default SIO bit-bang backend; qualification starts asynchronously after the physical SD card finishes its final PIO1 initialization. A failed PIO qualification resets the chip and falls back automatically.
   * `PSRAM_CACHE_ENABLE_ON_PROBE` *(on by default)* - Enables the E3/E5 sector cache after PSRAM qualification. Set it to `OFF` for a probe-only diagnostic build: PSRAM is still tested and logged, but every loader sector is served directly from the physical SD card.
   * `DSPICO_ENABLE_WRFUXXED` *(commented out in CMakeLists.txt)* - Enables emulation of the IS-SPI-USB-ADAPTER to support the WRFUxxed exploit. This requires <code>uartBufv060.bin</code> to be placed in the `data/` folder. Enable by uncommenting the line in `add_compile_definitions`.
   * `ENABLE_PREVENT_DSI_AUTOBOOT` *(commented out)* - Experimental feature that prevents DSi consoles from autobooting when the autoboot flag is set. It was intended to be used with WRFU Tester, which has the autoboot flag set. It is generally not recommended to use this, as it does not work properly with the 3DS and has not been tested much.
   * `DETECT_CONSOLE_TYPE` *(auto)* - Set automatically when both `roms/default.nds` and `roms/dsimode.nds` are present, enabling the firmware to switch the rom based on which console is detected. You shouldn't change this manually.

#### Stack sizes (`add_compile_definitions`)
   * `PICO_STACK_SIZE=0x600` - core0 stack.
   * `PICO_CORE1_STACK_SIZE=0x400` - core1 stack.

#### PSRAM / cache source-level switches (`src/psram.c`, `src/romCache.c`)
These are `#ifndef`-guarded internal switches. Defaults reflect the working configuration; prefer the CMake options above instead of defining them manually.
   * `PSRAM_FORCE_BITBANG` *(default `1` through CMake)* - Derived from `PSRAM_USE_PIO`. At `1`, every PSRAM access uses pure SIO bit-bang and the PIO transfer functions/SM2 program are excluded entirely. Probe and runtime cache transfers execute on core1; core0 submits runtime requests while retaining cartridge IRQ service, so an IRQ cannot stretch a CE#-low bit-bang burst. In game mode each 512 B request is serviced one 32 B electrical burst at a time, with the core1 scrambler ring refilled to full between bursts; the experimental PIO backend uses the same scheduling with 16 B bursts.
   * `SD_CACHE_ENABLE` *(default `1`)* - Gates the SD-sector cache **hit** path (E3 tag check + async fill). At boot, both cache reads and backfills remain disabled for `SD_CACHE_IO_WARMUP_MS`, so pico-loader mount/open sees the same runtime path as a probe-only build. Set to `0` to disable cache reads while keeping post-warmup backfill.
   * `SD_CACHE_IO_WARMUP_MS` *(default `3000`)* - Quiet period after successful cache qualification before either PSRAM hits or backfills may run. A cold-boot trace showed that even a non-blocking 69 us backfill during mount/open doubled E4 polling and stopped the loader before its next E5. All timer arithmetic stays on the core0 main loop; E3/E5 IRQs read only a one-byte gate.
   * `SD_CACHE_STORE` *(default `1`)* - Gates the SD-sector cache **backfill** path. E5 queues a sector; the main loop submits a non-blocking PSRAM write to core1, continues advancing SDIO, and publishes the tag only after a later completion poll. Set to `0` to disable caching of newly read sectors.
   * `PSRAM_CACHE_ENABLE_ON_PROBE` *(default `1`)* - When `1`, a successful core1 PSRAM probe immediately enables the cache. When `0`, the probe still runs (confirming the PSRAM is present) but the cache stays disabled - SD sectors are served purely from the SD card, identical to an `ENABLE_PSRAM_CACHE=OFF` build. Useful for isolating whether a problem comes from the probe bursts or from cache use.
   * `PSRAM_FULL_CHIP_TEST` *(default `0`)* - When `1`, core1 runs an 8 MB write+verify full-chip test after the probe before enabling the cache, one chunk per scrambler-idle slot. Off by default because the probe already validates the data path and the full pass delays cache availability.
   * `PSRAM_CACHE_HEARTBEAT_LOG` *(default `1`)* - Periodically prints cache hit/miss/usage statistics in UART builds. Used-line accounting is maintained incrementally in O(1); it does not scan all 16384 tags. Set to `0` when measuring completely log-free runtime timing.

### Setting up the rom(s)
To compile and properly use the firmware, you will need to place a valid DS rom in the `roms/` folder, named `default.nds`. Additionally, you may include a second rom in the `roms/` folder named `dsimode.nds`, if you wish to have a different rom for DS consoles and DSi/3DS consoles.
<table>
    <tr>
        <th>Usage</th>
        <th>default.nds</th>
        <th>dsimode.nds</th>
        <th>Notes</th>
    </tr>
    <tr>
        <td>Single rom for DS and/or DSi</td>
        <td>Your rom</td>
        <td>-</td>
        <td>The rom must contain NTR blowfish keys.<br>If the rom is hybrid or DSi exclusive, it must additionally contain TWL blowfish keys.</td>
    </tr>
    <tr>
        <td>Hybrid bootloader</td>
        <td>Bootloader</td>
        <td>-</td>
        <td>The bootloader rom must be patched with the DSpico DLDI.<br>The bootloader rom must contain NTR and TWL blowfish keys.</td>
    </tr>
    <tr>
        <td>DSi ntrboot</td>
        <td>GCD rom</td>
        <td>-</td>
        <td>The rom must contain GCD blowfish keys and must be properly signed.<br>The DSpico must be using USB power, such that the firmware is booted before starting the DSi. Without external power, the firmware currently does not boot fast enough to keep up with DSi ntrboot.</td>
    </tr>
    <tr>
        <td>3DS ntrboot</td>
        <td>3DS ntrboot rom</td>
        <td>-</td>
        <td>A 3DS ntrboot rom consists of a header, the blowfish keys and the firm to boot.</td>
    </tr>
    <tr>
        <td>Separate rom for DS and DSi</td>
        <td>Your DS rom</td>
        <td>Your DSi rom</td>
        <td>The DS rom must contain NTR blowfish keys.<br>The DSi rom must contain both NTR and TWL blowfish keys.</td>
    </tr>
    <tr>
        <td>WRFUxxed</td>
        <td>Bootloader</td>
        <td>WRFU Tester v0.60</td>
        <td>The bootloader rom and <code>uartBufv060.bin</code> must be patched with the DSpico DLDI.<br><code>uartBufv060.bin</code> must be placed in the <code>/data</code> folder.<br>The bootloader rom must contain NTR blowfish keys.<br>The <code>DSPICO_ENABLE_WRFUXXED</code> define in <code>CMakeLists.txt</code> must be enabled.</td>
    </tr>
</table>

## Compiling
Simply run `./compile.sh` to compile the firmware. Once it is complete, you will be able to find `DSpico.uf2` in the `build/` folder, which you can use to flash your DSpico board with.

> [!IMPORTANT]
> The firmware only works correctly when built with optimization. Recommended is `RelWithDebInfo`.

### Host regression tests

The cache-consistency tests use host stubs and do not require a ROM, the Pico SDK, or an ARM toolchain:

```bash
cmake -S tests -B build-host-tests
cmake --build build-host-tests
ctest --test-dir build-host-tests --output-on-failure
```

## License

The firmware for the DSpico project includes code that is licensed under the following:

SPDX-License-Identifier: Zlib
- Miscellaneous source files

SPDX-License-Identifier: BSD-3-Clause
- Pico SDK

SPDX-License-Identifier: GPL-3.0-or-later
- ZuluSCSI

For details, see the `license` directory, as well as `LICENSE.txt`.

## Contributors
- [@Gericom](https://github.com/Gericom)
- [@XLuma](https://github.com/XLuma)
- [@Dartz150](https://github.com/Dartz150)
- [@pedro-javierf](https://github.com/pedro-javierf)
