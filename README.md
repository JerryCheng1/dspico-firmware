# DSpico Firmware
This is the repository for the DSpico firmware. The firmware emulates a DS cartridge, with extended features for SD access and USB. PIO is used for an SDIO interface for the SD card and for interfacing the DS cartridge bus.

For an overview of the supported card commands, see [commands.md](docs/commands.md).

## Features
- Emulates a retail DS(i) cartridge
- Interfaces with an SD card using SDIO and exposes card commands to access it from the DS side
- Exposes card commands to the DS side to allow interfacing with the USB port of the RP2040
- Can emulate an R4 to support software such as the Wood R4 kernel
- Supports having a separate rom for DS and DSi/3DS systems
- Supports emulating the IS-SPI-USB-ADAPTER for the WRFUxxed exploit
- Easy updating; starting the firmware with an ejected SD card reboots to BOOTSEL
- Optimized for minimal power use when idle
- Optional ROM read cache in external APS6404 PSRAM (DSPICOwithPSRAM hardware)

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
| **PSRAM** (DSPICOwithPSRAM only) | SIO0     | GPIO22            |
|             | SIO1                  | GPIO23            |
|             | SIO2                  | GPIO24            |
|             | SIO3                  | GPIO25            |
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

### CMakeList
The `CMakeList.txt` file contains a couple of options that you can configure.

Options passed on the cmake command line with `-D<name>=ON/OFF`:

   * `ENABLE_PSRAM_ROM_CACHE` - Caches ROM blocks read through the R4 protocol in the external 64 Mbit APS6404 PSRAM (GPIO22-26/29, see the DSPICOwithPSRAM hardware), so repeated reads no longer hit the SD card. **Default: ON.** The firmware falls back to normal behaviour when no working PSRAM is detected at boot. Mutually exclusive with `DSPICO_ENABLE_WRFUXXED` (see below); in WRFUXXED builds the cache automatically uses a slower bit-banged PSRAM data path.
   * `ENABLE_UART_LOG` - Prints firmware logs (including PSRAM detection and cache hit-rate statistics) on the debug UART (GPIO0/1, 115200 8N1). **Default: OFF.** When disabled, all log statements are compiled out and the UART stdio is not initialized.

Defines toggled by (un)commenting them in the `add_compile_definitions` block of `CMakeLists.txt`:

   * `ENABLE_R4_MODE` - Enables R4 emulation. This allows you to use R4 software, such as the Wood R4 kernel. As R4 emulation can be used together with regular DSpico software, it can usually be kept enabled. **Default: enabled.**
      * Note that to be able to use R4 software, your SD card must be at most 4 GB, or have a single partition in the first 4 GB of the SD card. R4 card commands cannot address SD sectors above 4 GB!
   * `DSPICO_ENABLE_WRFUXXED` - Enables emulation of the IS-SPI-USB-ADAPTER to support the WRFUxxed exploit. This requires <code>uartBufv060.bin</code> to be placed in the `data/` folder. **Default: disabled.** Uses pio0 sm1 and fills the remaining pio0 instruction memory, so the PSRAM ROM cache falls back to its bit-banged data path when this is enabled.
   * `ENABLE_PREVENT_DSI_AUTOBOOT` - Experimental feature that prevents DSi consoles from autobooting when the autoboot flag is set. It was intended to be used with WRFU Tester, which has the autoboot flag set. It is generally not recommended to use this, as it does not work properly with the 3DS and has not been tested much. **Default: disabled.**

Defines set automatically by the build system (do not set these yourself):

   * `DETECT_CONSOLE_TYPE` - Enabled automatically when both `roms/default.nds` and `roms/dsimode.nds` exist. Switches the served rom based on the detected console type.
   * `PICO_STACK_SIZE` / `PICO_CORE1_STACK_SIZE` - Stack sizes for core0/core1, tuned for this firmware.

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
> The firmware only works correctly when build with optimization. Recommended is `RelWithDebInfo`.

To pass cmake options or use your own pico-sdk checkout, configure and build manually:

```bash
# configure (set PICO_SDK_PATH if the sdk is not the pico-sdk/ submodule)
PICO_SDK_PATH=/path/to/pico-sdk \
    cmake -DCMAKE_BUILD_TYPE:STRING=RelWithDebInfo -B build/ .

# build
PICO_SDK_PATH=/path/to/pico-sdk \
CMAKE_BUILD_PARALLEL_LEVEL=$(nproc) \
    cmake --build build
```

Examples with options (see the CMakeList section above for the full list):

```bash
# default build: PSRAM ROM cache on, UART logging off
cmake -DCMAKE_BUILD_TYPE:STRING=RelWithDebInfo -B build/ .

# debug build with UART logs on GPIO0/1 (115200 8N1)
cmake -DCMAKE_BUILD_TYPE:STRING=RelWithDebInfo -DENABLE_UART_LOG=ON -B build/ .

# build without the PSRAM ROM cache (e.g. for the stock DSpico board)
cmake -DCMAKE_BUILD_TYPE:STRING=RelWithDebInfo -DENABLE_PSRAM_ROM_CACHE=OFF -B build/ .
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
