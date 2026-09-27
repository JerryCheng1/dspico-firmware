# DSpico 固件

DSpico 模拟 Nintendo DS/DSi 卡带，并通过 SDIO 访问存储卡。当前分支针对 RP2354A（RP2350A 引脚配置）；卡带接口和 SDIO 接口由 PIO 驱动。支持普通卡带、R4 模式，以及可选的四芯片 PSRAM 读取缓存。卡带命令见[命令说明](docs/commands.md)。

> 本分支的 RP2354A 板载 USB 焊盘用于 UART 日志，固件不提供旧 RP2040 版本的 USB 功能或通过拔出 SD 卡进入 BOOTSEL 的更新方式。

## 功能与硬件连接

- 模拟零售 DS/DSi 卡带，也可模拟 R4 以运行 Wood R4 等软件。
- 通过四位 SDIO 访问 SD 卡，并向主机提供扩展 SD 命令。
- 可为 DS 与 DSi/3DS 分别嵌入 ROM。
- 可选用四颗 APS6404L PSRAM 作为只读扇区缓存。
- 可选启用 WRFUxxed 所需的 IS-SPI-USB-ADAPTER 模拟；此功能需要额外数据文件及源码宏。
- 空闲时降低功耗。

引脚分配以 [common.h](src/common.h) 和 [psram.h](src/psram.h) 为准：

| 接口 | 信号 | RP2354A GPIO |
|---|---|---:|
| 卡带 | 数据 D0–D7 | 12–19 |
| 卡带 | 时钟 WREB | 11 |
| 卡带 | ROM 片选 CEB | 10 |
| 卡带 | SPI 片选 CS2 | 21 |
| 卡带 | IRQ | 20 |
| 卡带 | 复位 | 9 |
| SDIO | CLK、CMD | 3、4 |
| SDIO | DAT0–DAT3 | 5–8 |
| PSRAM | SCLK、IO0–IO3 | 0、22–25 |
| PSRAM | U2、U4、U5、U6 片选 | 26、27、28、29 |

启用 UART 日志时，UART1 TX 使用 USB_DP 焊盘，速率为 115200、8N1。该焊盘不能同时作为本固件的 USB 数据接口。

## 准备开发环境

建议在 Linux 或 WSL 中构建。需要 CMake、Git、构建工具和 Arm 裸机工具链；在 Debian/Ubuntu 上可安装：

```bash
sudo apt update
sudo apt install cmake gcc-arm-none-eabi build-essential git
```

克隆仓库后初始化 SDK 子模块：

```bash
git submodule update --init pico-sdk
git -C pico-sdk submodule update --init
```

无需对整个 SDK 执行 `--recursive`。也可参考仓库中的 [setup_environment.sh](setup_environment.sh)。默认使用仓库内的 `pico-sdk`；若要使用其他检出目录，在配置时设置环境变量或传入 `-DPICO_SDK_PATH=...`。当前板级配置见 [dspico_rp2354a.h](src/boards/dspico_rp2354a.h)。

## 准备 ROM

构建前将合法的 DS ROM 放在 `roms/default.nds`。如需按主机类型切换 ROM，可再放入 `roms/dsimode.nds`；只有两个文件都存在时，构建才会启用主机类型检测。ROM 会在编译时嵌入固件。

| 用途 | `default.nds` | `dsimode.nds` | 要求 |
|---|---|---|---|
| DS/DSi 使用同一 ROM | 游戏或启动器 | 不放置 | ROM 需包含 NTR Blowfish 密钥；混合或 DSi 专用 ROM 还需 TWL 密钥。 |
| DS 与 DSi 使用不同 ROM | DS ROM | DSi ROM | DS ROM 需包含 NTR 密钥；DSi ROM 需包含 NTR 和 TWL 密钥。 |
| 混合模式启动器 | 已打 DSpico DLDI 补丁的启动器 | 不放置 | 启动器需包含 NTR 和 TWL 密钥。 |
| DSi ntrboot | 正确签名的 GCD ROM | 不放置 | 需包含 GCD 密钥，并使固件在 DSi 启动前获得外部供电。 |
| 3DS ntrboot | 3DS ntrboot ROM | 不放置 | ROM 包含头部、Blowfish 密钥和待启动的 FIRM。 |
| WRFUxxed | 已打 DSpico DLDI 补丁的启动器 | 已打补丁的 WRFU Tester v0.60 | 还需将打补丁的 `uartBufv060.bin` 放到 `data/`，并启用 `DSPICO_ENABLE_WRFUXXED` 宏。 |

R4 模式已在当前 [CMakeLists.txt](CMakeLists.txt) 中启用。使用 R4 软件时，SD 卡容量应不超过 4 GB，或其唯一分区位于前 4 GB；R4 命令不能寻址 4 GB 以上的 SD 扇区。

## 构建普通固件

当前分支使用 pico-sdk 2.3.0 或更新版本，目标为 `PICO_PLATFORM=rp2350`、`PICO_BOARD=dspico_rp2354a`。请使用 `Release` 或 `RelWithDebInfo` 优化构建：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

生成的 `build/DSpico.uf2` 是刷写文件，`build/DSpico.elf` 可用于检查链接结果。新构建目录默认设置 `CACHE_STAGE=0`，此时不编入 PSRAM 缓存；目录名和 UART 标签都不会改变该选项。CMake 会记住已有构建目录的设置，切换模式时应使用新目录或明确重设选项。

常用配置如下：

| 选项或源码宏 | 作用 |
|---|---|
| `ENABLE_UART_LOG=ON` | 启用 UART1 和启动日志；单独启用不会开启 PSRAM 缓存。 |
| `ENABLE_CART_TRACE=ON` | 启用运行时卡带总线旁路诊断；需同时启用 UART。PIO 监听器和无中断 DMA 环形缓冲区采集数据，由 Core1 输出日志。 |
| `UART_LOG_BUILD_TAG=<名称>` | 在普通启动日志中标识构建；精简缓存日志会隐藏启动横幅。 |
| `ENABLE_SD_WRITE_PROTECT=ON` | 接受并报告 SD 写入成功，但不向存储卡写入数据。 |
| `DSPICO_ENABLE_WRFUXXED` | [CMakeLists.txt](CMakeLists.txt) 中默认注释的编译宏；启用前须准备相应数据文件。 |
| `ENABLE_PREVENT_DSI_AUTOBOOT` | 默认注释的实验宏；与部分 3DS 不兼容，不建议常规使用。 |

仅记录启动过程的示例：

```bash
cmake -S . -B build-bootlog -DCMAKE_BUILD_TYPE=Release \
    -DENABLE_UART_LOG=ON -DUART_LOG_BUILD_TAG=bootlog
cmake --build build-bootlog -j$(nproc)
```

## 四芯片 PSRAM 缓存

当前完整缓存配置使用阶段 3、M5 模式和四芯片 4 KiB 页目录。每颗 APS6404L 标称 8 MiB，提供 2031 个数据页组；四颗共 **33,275,904 B（31.73 MiB）净数据容量**。该值是固件可寻址配置，并不表示所有物理地址都经过逐字节实板验证。

以下配置启用完整缓存和单行 UART 统计：

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

刷写产物为 `build-cache-m5-full/DSpico.uf2`。单行统计在启动保护期结束且卡带总线空闲后输出；有变化时至少间隔 5 秒，没有变化则不重复输出。示例：

```text
[cache] total=31.73MiB used=60928B occupancy=0.18% hitRate=1.02% hits=138/13573
```

`total` 表示已就绪芯片的净数据容量；`used` 为当前代次有效扇区数乘以 512 B；`occupancy=used/total`。`hitRate` 为成功提交给卡带响应槽的 PSRAM 读取数，除以所有成功提交的读取数。SD 读取和启动预热扇区计入分母；取消或过期的请求不计入。这是固件侧的提交统计，不是主机完整接收数据的独立证明。挂载或预热失败仍会报告错误。

**此配置启用了写保护。** 游戏发起的 SD 写入会显示成功，但不会落盘，新存档不能持久化。当前四芯片页缓存构建仍要求此选项，不应将其视为可正常保存游戏进度的发行配置。

### 缓存工作流程

卡带请求以 512 B 扇区为单位。固件以 `sector >> 3` 为逻辑页号，以 `sector & 7` 为页内扇区；一个 4 KiB 页对应 8 个扇区。逻辑页映射到 `page % 2031` 号集合，每组四路分别固定在四颗 PSRAM。新页先使用空路或旧代次路，满组后以有界 CLOCK 算法选择替换对象；正在读取或回填的页不会被替换。同一页的各扇区可分别标记有效。

未命中时，SD 将扇区读入独立的 512 B 暂存区。确认传输有效后，固件复制数据到 SRAM 响应槽，由 E5/DMA0 发送。后台回填另存一份快照，在该响应传输完成且满足空闲门控后进行；每次 PSRAM 操作最多 32 B。回填完成后，固件更新该页的 8 项 CRC32 表并回读验证，最后才公布该扇区有效。

M5 命中时，固件固定对应目录项，读取并验证 CRC 表及扇区数据，再将确认过的数据交给响应槽。CRC 不符会使该页失效；PSRAM 传输故障会降级缓存。两种情况都回退到 SD，并使用与失败的 PSRAM 读取隔离的缓冲区。写入序列会推进缓存代次，使旧页和未完成回填不能作为新请求的命中。M4 用于后台比对，读取仍由 SD 供数；M5 才允许经校验的 PSRAM 命中直接供数。实现分别见[页目录](src/cachePageMap.c)、[PSRAM 页传输](src/cachePageStore.c)和[SD 缓存状态机](src/cacheSd.c)。

每颗 PSRAM 按 2031 个连续页组划分，每组为 **4096 B 数据 + 32 B CRC 表 = 4128 B**，芯片末端保留 4640 B。四颗共有 8124 个物理页组，净数据容量为 `8124 × 4096 = 33,275,904 B`；CRC 表和保留区不计入 `total`。限制 `CACHE_PAGE_M5_SET_LIMIT` 只会减少启用的页组，SRAM 目录仍按完整 2031 组分配。

### 构建参数

| 选项或宏 | 含义 |
|---|---|
| `CACHE_STAGE=3` | 编入请求/响应槽状态机、SD 缓存后端和 PSRAM 驱动；`0` 为无缓存构建。当前仅实现阶段 `0..3`。 |
| `L2_DIAG_MODE=M5` | 允许校验过的 PSRAM 命中为读取供数；M4 仍由 SD 供数。 |
| `CACHE_FULL_PAGE_4CHIP=ON` | 使用四芯片页目录及每页 CRC 表；关闭时保留旧扇区索引后端。 |
| `CACHE_PAGE_M5_EXPERIMENT=ON` | 允许 M5 从完整页后端读取。 |
| `CACHE_PAGE_M5_SET_LIMIT=2031` | 每颗芯片启用的页组数，范围 `1..2031`，默认 2031。 |
| `CACHE_FILL_QUIET_EXPERIMENT=ON` | 在符合空闲门控的窗口逐片段回填。 |
| `CACHE_WATCH_LOG=ON`、`CACHE_SUMMARY_LOG=ON` | 启用 Core1 空闲门控及单行缓存统计；关闭后者可恢复详细诊断日志。 |
| `CACHE_BOOT_SECTOR0_SEED=ON` | 将启动时预热的 SD 扇区 0 用于首次对应读取。 |
| `CACHE_SD_SECTORS=4096` | 旧扇区索引后端的 2 MiB 索引规模，不是完整页后端的容量上限。 |
| `PSRAM_CACHE_FRAG_BYTES=32`、`PSRAM_PIO_FRAG_BYTES=32` | PSRAM 传输的数据片段上限。 |
| `PSRAM_PIO_CLKDIV=3.0f` | 当前运行期 PIO 分频值。 |

`CACHE_PAGE_ACTIVE_SETS` 由 CMake 的 `CACHE_PAGE_M5_SET_LIMIT` 派生，无需单独设置。独立的 `PsramQual` 目标可通过 `PSRAM_QUAL_FIRMWARE=ON` 构建，用于 PSRAM 资格测试，不属于游戏固件。

### SRAM 与 PSRAM 布局

缓存目录和临时数据位于片内 SRAM；已回填的扇区数据和 CRC 表位于 PSRAM。下面的容量和百分比按四颗芯片全部就绪、每颗启用 2031 组计算。

**PSRAM 物理布局**（每颗 8,388,608 B，四颗共 33,554,432 B）：

| 区域 | 每颗芯片 | 四颗合计 | 占四颗物理容量 | 说明 |
|---|---:|---:|---:|---|
| 扇区数据区 | 8,318,976 B | 33,275,904 B | 99.17% | 8124 个 4 KiB 页；日志的 `total` 只计算此项。 |
| 每页 CRC32 表 | 64,992 B | 259,968 B | 0.77% | 每页 32 B，存放 8 个扇区的 CRC32。 |
| 芯片末端保留区 | 4,640 B | 18,560 B | 0.06% | 不分配给缓存页。 |
| **物理总量** | **8,388,608 B** | **33,554,432 B** | **100.00%** | 含数据区、校验表和保留区。 |

上述 99.17% 是**地址空间分配比例**，不是运行时填充率。实际有效数据量由日志的 `used` 给出，运行时占用率为 `used / total`；例如 `used=60928B` 时，净缓存占用率约为 0.18%。未就绪的芯片不会计入日志中的 `total`。

**SRAM 主要缓存对象**（完整配置的固定分配）：

| 对象 | 大小 | 用途 |
|---|---:|---|
| `cachePageStore.map` 页目录 | 163,000 B | 8124 个 20 B 目录项、每组 2 bit CLOCK 指针、计数和芯片掩码。 |
| 四个 `sSlotBuf` 响应槽 | 2,048 B | E5/DMA0 使用的四份 512 B 扇区响应。 |
| 需求、回退、回填、扇区 0 暂存区 | 2,048 B | 四份独立的 512 B 缓冲区；扇区 0 缓冲区需启用 `CACHE_BOOT_SECTOR0_SEED`。 |
| `sCrcTab` | 1,024 B | 计算扇区 CRC32 的查表。 |

缓存还有两个各 120 B 的任务状态及少量计数。SD、FAT 和卡带协议也占用 SRAM：例如 ROM 簇表为 65,536 B，ROM 大块缓冲区与存档簇表各为 16,384 B。上表是**部分对象清单**，不可与下表的段大小相加；这些对象已包含在 `.data` 或 `.bss` 中。

**SRAM 链接占用**以本页构建选项的一次 `DSpico.elf` 为例。链接脚本分配主 SRAM 512 KiB，以及各 4 KiB 的 Scratch X、Scratch Y，共 **532,480 B（520 KiB）**：

| 区域 | 容量 | 已分配或预留 | 占用率 | 构成 |
|---|---:|---:|---:|---|
| 主 SRAM | 524,288 B | 307,836 B | 58.72% | 向量表 272 B、`.data` 25,984 B、`.bss` 279,532 B、`.heap` 预留 2,048 B。 |
| Scratch X | 4,096 B | 1,376 B | 33.59% | `.scratch_x` 352 B、Core1 栈预留 1,024 B。 |
| Scratch Y | 4,096 B | 3,736 B | 91.21% | `.scratch_y` 2,200 B、Core0 栈预留 1,536 B。 |
| **SRAM 合计** | **532,480 B** | **312,948 B** | **58.77%** | 尚未由链接段分配或预留 219,532 B，占 41.23%。 |

本次链接的主 SRAM 固定对象结束于 `0x2004AA7C`。构建使用 `PICO_STACK_SIZE=0x600`、`PICO_CORE1_STACK_SIZE=0x400`。表中的“占用”是链接时分配或预留的量；运行时堆和栈的实际峰值需另行测量，不能把 219,532 B 直接当作已验证的运行时余量。地址及段大小也会随 SDK、编译器和构建选项变化。可检查自己构建的 ELF：

```bash
arm-none-eabi-size -A build-cache-m5-full/DSpico.elf
arm-none-eabi-nm -S --size-sort build-cache-m5-full/DSpico.elf
```

核对构建选项和刷写文件：

```bash
rg '^(CACHE_STAGE|L2_DIAG_MODE|CACHE_FULL_PAGE_4CHIP|CACHE_PAGE_M5_EXPERIMENT|CACHE_PAGE_M5_SET_LIMIT|CACHE_SUMMARY_LOG|ENABLE_SD_WRITE_PROTECT):' build-cache-m5-full/CMakeCache.txt
sha256sum build-cache-m5-full/DSpico.uf2
```

精简模式不会打印 UART 构建标签，应通过 CMake 配置和 UF2 哈希区分版本。容量数字、缓存统计和游戏运行情况分别提供不同证据；单凭 `total` 不能证明全部 PSRAM 地址可靠。

## 许可证

本项目所含代码分别采用以下许可证：

- Zlib：部分项目源码。
- BSD-3-Clause：Pico SDK。
- GPL-3.0-or-later：ZuluSCSI。

具体条款见 [LICENSE.txt](LICENSE.txt) 和 [licenses](licenses) 目录。

## 贡献者

- [@Gericom](https://github.com/Gericom)
- [@XLuma](https://github.com/XLuma)
- [@Dartz150](https://github.com/Dartz150)
- [@pedro-javierf](https://github.com/pedro-javierf)
