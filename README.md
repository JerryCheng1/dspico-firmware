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

   * `ENABLE_R4_MODE` - Enables R4 emulation. This allows you to use R4 software, such as the Wood R4 kernel. As R4 emulation can be used together with regular DSpico software, it can usually be kept enabled.
      * Note that to be able to use R4 software, your SD card must be at most 4 GB, or have a single partition in the first 4 GB of the SD card. R4 card commands cannot address SD sectors above 4 GB!
   * `DSPICO_ENABLE_WRFUXXED` - Enables emulation of the IS-SPI-USB-ADAPTER to support the WRFUxxed exploit. This requires <code>uartBufv060.bin</code> to be placed in the `data/` folder.
   * `ENABLE_PREVENT_DSI_AUTOBOOT` - Experimental feature that prevents DSi consoles from autobooting when the autoboot flag is set. It was intended to be used with WRFU Tester, which has the autoboot flag set. It is generally not recommended to use this, as it does not work properly with the 3DS and has not been tested much.
   * `ENABLE_SD_WRITE_PROTECT` - SD card write protection. When enabled, every SD card write (DS-side SD/R4 card commands and the FatFs `disk_write()` path) is silently swallowed: the write is reported as successful without any data being sent to the card, so its contents cannot be modified. Reads are unaffected. Configure with `-DENABLE_SD_WRITE_PROTECT=ON`.

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
This branch (`v1.0.1-rp2350`) targets the RP2354A (RP2350A) and requires **pico-sdk 2.3.0 or newer**. The RP2040-oriented `compile.sh` from the main branch does not apply here; build with CMake directly.

The SDK ships as the `pico-sdk` submodule (checked out at 2.3.0), and `CMakeLists.txt` already defaults `PICO_SDK_PATH` to it and sets `PICO_PLATFORM=rp2350` and `PICO_BOARD=dspico_rp2354a` (board definition in `src/boards/`). So a full build is a single command:

```bash
git submodule update --init pico-sdk && cmake -DCMAKE_BUILD_TYPE=Release -B build && cmake --build build -j$(nproc)
```

Outputs land in `build/`: `DSpico.uf2` (flash via BOOTSEL) and `DSpico.elf`.

To build against a different SDK checkout instead of the submodule, set `PICO_SDK_PATH` (environment variable or `-DPICO_SDK_PATH=...`) at configure time.

> [!IMPORTANT]
> The firmware only works correctly when built with optimization. Use `Release` (or `RelWithDebInfo`).

### Build options (pass with `-D` at configure time)
| Option | Effect |
|---|---|
| `ENABLE_UART_LOG=ON` | Diagnostic build: deferred UART log (GPIO0 TX, 115200 8N1), failure/quiet dumps, RX/E4-FIFO/SD-event trace rings, logic-analyzer trigger on GPIO0. The log drain only runs while the cartridge bus is quiet, so it does not disturb normal operation. |
| `TRACE_QUIET_LOG=ON` | Requires `ENABLE_UART_LOG=ON`. Keeps every tracer and the dumps but silences the per-event log stream, so the main loop runs at nodebug speed. Use it to catch failures that only reproduce in the nodebug build. |
| `UART_LOG_BUILD_TAG=<name>` | Tag printed on the UART boot banner so a flashed build is identifiable from its log. Bump it on every build. |

The three variants used during development:

```bash
# nodebug (shipping candidate): no options
cmake -DCMAKE_BUILD_TYPE=Release -B build-nodebug .

# diag: full logging
cmake -DCMAKE_BUILD_TYPE=Release \
    -DENABLE_UART_LOG=ON -DUART_LOG_BUILD_TAG=my-diag -B build-diag .

# traceq: nodebug speed, tracers only
cmake -DCMAKE_BUILD_TYPE=Release \
    -DENABLE_UART_LOG=ON -DTRACE_QUIET_LOG=ON -DUART_LOG_BUILD_TAG=my-traceq -B build-traceq .
```

For the diagnostic build, connect a UART adapter to GPIO0 (TX, 115200 8N1) to read the log output.

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
