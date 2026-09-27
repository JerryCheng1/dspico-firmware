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
| `ENABLE_UART_LOG=ON` | Enables UART1 on the USB_DP pad, 115200 8N1, and boot logging for SD bring-up, `f_mount` and prewarm. `CACHE_SUMMARY_LOG=ON` suppresses routine boot logs. Runtime trace and cache logs require their separate switches below. This option alone does not enable PSRAM caching. |
| `ENABLE_CART_TRACE=ON` | Requires `ENABLE_UART_LOG=ON`. Non-intrusive cartridge diagnostics: a PIO0 input listener observes CEB/WREB/D0..D7, an IRQ-less DMA ring stores the raw command words, and core1 decodes them and prints bounded, non-blocking UART text with SD/cache snapshots. Nothing is added to `PIO0_IRQ_0`, the GPIO/DMA/timer IRQs or the E3/E4/E5 handlers, so a trace build is instruction-identical to the boot-log build on the real-time cartridge path. |
| `UART_LOG_BUILD_TAG=<name>` | Tag printed on the normal UART boot banner; the compact cache summary suppresses that banner. Bump it on every diagnostic build. |

The variants used during development:

```bash
# nodebug (shipping candidate): no options
cmake -DCMAKE_BUILD_TYPE=Release -B build-nodebug .

# boot log only: SD bring-up / f_mount / prewarm, no runtime tracing
cmake -DCMAKE_BUILD_TYPE=Release \
    -DENABLE_UART_LOG=ON -DUART_LOG_BUILD_TAG=my-bootlog -B build-bootlog .

# non-intrusive cartridge trace: PIO listener + DMA ring + core1 decode
cmake -DCMAKE_BUILD_TYPE=Release \
    -DENABLE_UART_LOG=ON -DENABLE_CART_TRACE=ON \
    -DUART_LOG_BUILD_TAG=my-diag -B build-diag .
```

For the logging builds, connect a UART adapter to the USB_DP pad (UART1 TX, 115200 8N1) to read the log output.

### RP2354A 四芯片 PSRAM 缓存

新建构建目录默认使用 `CACHE_STAGE=0`，不编入 PSRAM 缓存。`build/` 的名字和 UART 标签都不会开启缓存；CMake 会保留已有构建目录的选项，切换配置时应使用新目录或明确重设选项。

目前的完整缓存配置为 stage 3、M5、四芯片 4 KiB 页目录。每颗 APS6404L 的标称容量为 8 MiB；每颗可用于缓存数据的页组为 2031 组，每组 4 KiB，四颗合计 **33,275,904 B（31.73 MiB）**。每页另有 32 B 校验表，芯片末端保留区域不计入数据容量。这是固件配置的净容量，不表示所有地址已在实板逐字节验证。

以下命令构建完整缓存和精简 UART 统计版：

```bash
cmake -S . -B build-cache-m5-full -DCMAKE_BUILD_TYPE=Release \
    -DCACHE_STAGE=3 -DL2_DIAG_MODE=M5 \
    -DCACHE_FULL_PAGE_4CHIP=ON -DCACHE_PAGE_M5_EXPERIMENT=ON \
    -DCACHE_PAGE_M5_SET_LIMIT=2031 -DCACHE_FILL_QUIET_EXPERIMENT=ON \
    -DCACHE_WATCH_LOG=ON -DCACHE_SUMMARY_LOG=ON \
    -DCACHE_BOOT_SECTOR0_SEED=ON -DENABLE_SD_WRITE_PROTECT=ON \
    -DENABLE_UART_LOG=ON -DUART_LOG_BUILD_TAG=cache-m5-full
cmake --build build-cache-m5-full -j$(nproc)
```

产物为 `build-cache-m5-full/DSpico.uf2`。UART1 TX 使用 USB_DP，速率 115200 8N1。精简模式在启动保护期结束、卡带总线空闲后输出；统计发生变化时最短间隔 5 秒，没有变化则不重复输出。正常运行只保留一行总览，例如：

```text
[cache] total=31.73MiB used=60928B occupancy=0.18% hitRate=1.02% hits=138/13573
```

`total` 是已就绪芯片提供的净数据容量；`used` 是当前代次有效的 512 B 扇区数乘以 512，`occupancy=used/total`。`hitRate` 是成功提交的 PSRAM 读取数除以所有成功提交的读取数；SD 读取和启动预热扇区计入分母，取消或过期的请求不计入。此处统计的是固件提交给卡带响应槽的结果，不是主机已完整接收数据的独立证明。挂载或预热失败仍会输出错误信息。

**写保护提醒：** 上述构建要求 `ENABLE_SD_WRITE_PROTECT=ON`。游戏发起的 SD 写入会被报告成功，但不会写入存储卡，因此新存档不会持久化。当前四芯片页缓存路径仍以该选项作为构建约束；不要把它当作可保存游戏进度的发行配置。

| 选项或源码宏 | 当前含义 |
|---|---|
| `CACHE_STAGE=3` | 编入请求/响应槽状态机、SD 缓存后端和 PSRAM 驱动；`CACHE_STAGE=0` 保持无缓存构建。仅实现 `0..3`。 |
| `L2_DIAG_MODE=M5` | 允许经过校验的 PSRAM 命中为读取供数。M4 仍由 SD 供数，用于后台比对。 |
| `CACHE_FULL_PAGE_4CHIP=ON` | 使用四芯片页目录及每页 CRC 表；不开启时保留旧扇区索引后端。 |
| `CACHE_PAGE_M5_EXPERIMENT=ON` | 允许 M5 从四芯片页目录读取；要求上一选项为 ON。 |
| `CACHE_PAGE_M5_SET_LIMIT=2031` | 每颗芯片启用的页组数，范围 `1..2031`，默认 2031。减少此值可限制容量。 |
| `CACHE_FILL_QUIET_EXPERIMENT=ON` | 在符合门控条件的空闲窗口逐片段回填；完整页缓存配置要求开启。 |
| `CACHE_WATCH_LOG=ON` / `CACHE_SUMMARY_LOG=ON` | Core1 的空闲门控和单行统计。后者需要前者、完整页后端及 UART；关闭后者可恢复详细诊断日志。 |
| `CACHE_BOOT_SECTOR0_SEED=ON` | 利用启动预热的 SD 扇区 0 处理第一次相应读取。 |
| `CACHE_SD_SECTORS=4096` | 仅为旧扇区索引后端的 2 MiB 索引大小；**不是**完整页后端的容量上限。 |
| `PSRAM_CACHE_FRAG_BYTES=32` / `PSRAM_PIO_FRAG_BYTES=32` | PSRAM 传输的最大数据片段。 |
| `PSRAM_PIO_CLKDIV=3.0f` | 当前运行期 PIO 分频值。 |

`CACHE_PAGE_ACTIVE_SETS` 由 CMake 的 `CACHE_PAGE_M5_SET_LIMIT` 派生，通常不需要直接传编译器宏。容量布局定义在 [cachePageMap.h](src/cachePageMap.h)，缓存状态和计数定义在 [cacheSd.h](src/cacheSd.h)。固件主体仅从显式列出的 `src/` 文件构建；`tests/` 和实验记录 `docs/` 不进入 UF2。独立的 `PsramQual` 目标通过 `PSRAM_QUAL_FIRMWARE=ON` 构建，不属于游戏固件。

要核对某个构建目录的实际配置：

```bash
rg '^(CACHE_STAGE|L2_DIAG_MODE|CACHE_FULL_PAGE_4CHIP|CACHE_PAGE_M5_EXPERIMENT|CACHE_PAGE_M5_SET_LIMIT|CACHE_SUMMARY_LOG|ENABLE_SD_WRITE_PROTECT):' build-cache-m5-full/CMakeCache.txt
sha256sum build-cache-m5-full/DSpico.uf2
```

精简模式不打印 UART 启动标签；请通过构建目录配置和 UF2 哈希区分固件版本。缓存自检、运行统计和游戏正常运行提供不同层面的证据，不能仅从 `total` 判断完整物理地址空间的可靠性。

#### 缓存机制

卡带请求以 512 B 扇区为单位。固件把 `sector >> 3` 作为逻辑页号，`sector & 7` 作为页内扇区；每页包含 8 个扇区。逻辑页映射到 `page % 2031` 号集合，每个集合有四路，分别固定在四颗 PSRAM。新页优先占用空路或旧代次路，满组时使用有界 CLOCK 替换；正在读写的页不会被替换。同一页的各扇区可以分别有效。

一次未命中先从 SD 读入独立的 512 B 暂存区，确认传输有效后复制到 SRAM 响应槽并交给 E5/DMA0。后台回填使用另一份 512 B 快照，只在该响应传输结束并满足空闲门控后推进，每次 PSRAM 操作最多 32 B。回填先写数据，再更新该页的 8 项 CRC32 表并回读验证，最后才将该扇区标记为有效。命中时先固定目录项，读取并校验 CRC 表及数据，再将确认过的 512 B 交给响应槽；CRC 不符时使该页失效，PSRAM 传输故障时降级缓存，两者均从独立缓冲区回退到 SD。写入序列会推进缓存代次，旧代次的页与未完成回填不能作为新请求的命中。详细目录和传输状态分别见 [cachePageMap.c](src/cachePageMap.c)、[cachePageStore.c](src/cachePageStore.c) 和 [cacheSd.c](src/cacheSd.c)。

PSRAM 每颗按 2031 个连续页组划分：一个页组为 `4096 B 数据 + 32 B CRC 表 = 4128 B`，末端保留 4640 B。四颗共 8124 个物理页组，净数据容量为 `8124 × 4096 = 33,275,904 B`；CRC 表和保留区均不计入 `total`。`CACHE_PAGE_M5_SET_LIMIT` 只控制启用前多少组，目录仍按完整 2031 组分配 SRAM。

#### SRAM 布局

缓存的**目录和临时数据**位于片内 SRAM；已回填的扇区数据与 CRC 表位于 PSRAM。以下为完整四芯片配置的固定分配，单位均为字节：

| SRAM 对象 | 大小 | 用途 |
|---|---:|---|
| 页目录 `cachePageStore.map` | 163,000 | 8124 个 20 B 目录项、每组 2 bit CLOCK 指针、有效扇区计数和芯片掩码 |
| 四个响应槽 `sSlotBuf` | 2,048 | E5/DMA0 持有的 512 B 扇区响应 |
| 需求、回退、回填、扇区 0 暂存区 | 2,048 | 四份互不覆盖的 512 B 缓冲区；扇区 0 缓冲区仅在启用 `CACHE_BOOT_SECTOR0_SEED` 时存在 |
| CRC32 查表 `sCrcTab` | 1,024 | 固件计算扇区 CRC 的加速表 |

除上述对象外，缓存还有两个任务状态和少量计数/状态；SD、FAT、卡带协议和两核栈也占用 SRAM。例如 ROM 簇表为 65,536 B，ROM 大块缓冲区和存档簇表各为 16,384 B，均不属于 PSRAM 缓存容量。`PICO_STACK_SIZE=0x600`、`PICO_CORE1_STACK_SIZE=0x400` 是构建时的栈配置。不要把 `total` 当成片内 SRAM 剩余量：它只表示可用 PSRAM 的净数据空间。需要审计某一构建的实际链接占用时，检查其 `DSpico.elf` 的 `.data`、`.bss`、`.scratch_x`、`.scratch_y` 和栈段；运行时栈余量需要另行测量。

以上构建配置的链接示例中，SRAM 向量表为 272 B，`.data` 为 25,984 B，`.bss` 为 279,532 B，固定分配结束于 `0x2004AA7C`；独立的 `.scratch_x`、`.scratch_y` 分别占 352 B、2,200 B。上述地址和段大小随 SDK、编译器及选项变化，不是运行时可用堆栈空间的测量值。可用以下命令查看自己构建的结果：

```bash
arm-none-eabi-size -A build-cache-m5-full/DSpico.elf
arm-none-eabi-nm -S --size-sort build-cache-m5-full/DSpico.elf
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
