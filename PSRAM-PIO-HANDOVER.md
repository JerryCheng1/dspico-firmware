# PSRAM PIO1 架构与验证 HANDOVER

## 当前结论

PIO0 方案已经停止继续优化。硬件日志证明多版 PIO0 pump 都能通过 128 轮 PSRAM 写读自检，但 NDS 端仍会随机出现 `Failed to mount sd card`、`Failed to open pico loader file`、进菜单白屏或进游戏关机。自检只能证明 PSRAM 线上的数据正确，不能证明它与时序敏感的 NDS 卡带状态机能够长期共存。

PIO 实验方案把 PSRAM 完全移到 PIO1；默认方案则不为 PSRAM 使用任何 PIO 指令，而是在 core1 上连续执行 SIO bit-bang：

- PIO0 只运行 NDS 卡带程序，PSRAM 不再访问 PIO0 的 CTRL、FIFO、IRQ 或指令内存。
- PIO1 SM0 运行 SDIO command/clock，SM1 运行 SDIO data，SM2 运行 PSRAM。
- SDIO RX 和 TX 不会同时执行，因此动态复用同一段 9-word 指令区。
- 空出的 5-word 指令区固定给 PSRAM command/write pump。
- 默认构建使用 SIO bit-bang；运行期读写由 core0 提交给 core1，避免 cartridge IRQ 抢占并拉长 CE#-low 事务。只有显式设置 `PSRAM_USE_PIO=ON` 才启用 PIO1 SM2。

新方案已通过编译和主机缓存一致性测试，但尚需真机冷启动和游戏负载验证，不能把“PSRAM probe OK”当成整机稳定结论。

### 2026-08-11 A/B 结果

- `bit-bang + probe-only`（探测 PSRAM、但不启用 E3/E5 cache）可以进入游戏；SDIO CMD18、E4 ready 和 E5 均持续推进。
- 同一固件仅开启 cache 后，在第一次 E5 回填之后复现 `Failed to mount sd card`；此时后续物理 SD 读取仍为 `all done`，日志停在 `E5=1`。
- 因此 PSRAM 探测、物理 SD mount 和 CMD18 均不是触发条件，触发点被缩小到第一次运行期 cache backfill。
- 旧 bit-bang 回填在 core0 上执行，cartridge IRQ 可在 CE# 为低时抢占它。新架构把 bit-bang 运行期读写也提交给 core1，保证每个 PSRAM burst 不被 core0 IRQ 截断；PSRAM 仍不使用任何 PIO 指令。
- 后续真机日志证明连续 core1 回填能在约 99 us 内完成，但同步等待期间下一次 E3 已到达而 core0 无法推进 SDIO；E4 poll 从约 166 增至 314 后 loader 仍失败。因此 backfill 现在使用真正的非阻塞提交：core1 写入期间 core0 继续调用 `gSdCard.Update()`，完成事件到达后才发布 tag。
- 3 秒 mount/open warmup 后 loader 可以进入游戏，但第一次 PSRAM hit fetch 连续占用 core1 约 166 us，随后在厂商动画后黑屏；`verifyfail=0` 排除了已检测到的 PSRAM 内容损坏。此时 core1 同时负责生成 NDS 加扰字流，整条 512-byte transfer 的绝对优先级可能耗空加扰环。因此运行期 PSRAM 请求现拆成单个电气 burst：bit-bang 每次最多 32 bytes，PIO 每次最多 16 bytes；游戏阶段始终先填满 1024-word 加扰环，只在环满的空档执行一个 PSRAM burst。
- burst-interleaved 版本仍在 mount/open 阶段失败：第一次非阻塞回填只需约 69 us，且并行的 SDIO `read#3` 已 `all done`，但 E4 poll 从首次请求的 166 增到 308，loader 在 `E5=1` 后停止。这证明“只禁用 hit、启动期立即回填”仍不够安全。现改为 probe 完成后的 3 秒内完全禁止运行期 PSRAM 读写；E5 不复制、不排队回填，E3 固定 miss。窗口结束后才同时开放 backfill 和 hit。

## 为什么放弃 PIO0

已依次验证过以下 PIO0 变体：

1. PIO command/address/write + PIO read；
2. PIO command/write + SIO read；
3. 单 SM 完整 command/write，运行期由 core1 服务；
4. 16-byte burst、joined 8-word TX FIFO、启动前一次性预填；
5. 原子 SM enable/disable，避免 Pico SDK `pio_sm_set_enabled()` 的 CTRL read-modify-write 竞争；
6. 在 cartridge SM0 启用前执行 128 轮 PSRAM 资格测试。

这些修改能让 PSRAM 自检稳定通过，却不能消除 NDS 端的随机 loader 错误。最近一次失败日志中，PSRAM 128 轮 probe 成功、物理 SD 也成功 mount，但 NDS 仍报告 `Failed to mount sd card`。工程上可确定的是：PIO0 运行时共用仍会扰动卡带服务；无法仅凭现有 UART 日志把根因进一步归结为某一个 RP2040 总线周期。因此新架构的硬约束是“PSRAM 绝不使用 PIO0”，而不是继续给 PIO0 方案增加等待或重试。

## PIO 资源布局

### PIO0

| 资源 | 用途 |
|---|---|
| SM0 | NDS 卡带总线 |
| SM1 | 可选 WRFUXXED/SPI 程序 |
| SM2/SM3 | PSRAM 不使用 |
| 指令内存 | NDS 卡带相关程序 |

### PIO1（`PSRAM_USE_PIO=ON`）

| 指令地址 | 长度 | 程序 | 状态机 |
|---|---:|---|---|
| 0..4 | 5 | `psram_qspi_command_tx` | SM2 |
| 5..13 | 最多 9 | `sdio_data_rx` 或 `sdio_data_tx` | SM1 |
| 14..31 | 18 | `sdio_cmd_clk` | SM0 |

原 SDIO 布局中 command 18 条、RX 5 条、TX 9 条，总计正好 32 条。RX/TX 实际共享一个 SM，且同一时刻只需要其中一个程序，所以将二者固定到 5..13：

1. 原子禁用 SDIO SM1；
2. 从 Pico SDK 的 PIO allocator 中移除当前 RX/TX program；
3. 在 offset 5 装入目标 program，PIO SDK 同时完成 JMP relocation；
4. 重新初始化 SM1；
5. 原子启用 SM1。

连续读的 `rp2040_sdio_rx_continue()` 不切换程序；只有读写方向改变时才覆盖 5..13。PSRAM 的 0..4 始终驻留，不会被 SDIO 切换覆盖。

bit-bang 构建继续使用原来的 SDIO 常驻 RX+TX 布局，不执行动态覆盖。

## 5-word PSRAM pump

`src/psram.pio` 只负责 command/address/write 和 read dummy phase，read data 仍使用已验证的 SIO sampler。

运行前 CPU 在 SM2 禁用状态下完成：

- `X=7`：控制 8-bit serial command；
- `Y=quad nibble count - 1`：控制 address/data/dummy；
- IO0..IO3 先置高再切为输出；
- joined TX FIFO 预填完整 16-byte write transaction。

PIO 的五条指令依次完成 serial OUT、serial loop、quad OUT、quad loop 和 `irq wait 2` 精确停车。serial 阶段只有 IO0 被 OUT 修改，IO1..IO3 保持高；quad 阶段四根线一起输出。CPU 在 IRQ2 到达后先禁用 SM2，再清 IRQ2并释放数据线，避免 wrap 多打一拍时钟。

PIO burst 保持 16 bytes，以便完整 write 预填入 joined FIFO。bit-bang 保持 32 bytes：真机把它降为 16 bytes 后，512-byte hit fetch 从约 120 us 增至 187 us，并稳定触发 loader E4 超时。bit-bang 运行期事务在 core1 上不被 IRQ 打断，cache hit 另由 SRAM 摘要保护。

每个有效 cache line 另在 SRAM 中保存 16-bit 内容摘要。PSRAM hit 读取后必须先验证摘要；不一致时立即作废该行并回退物理 SD，数据不会通过 E5 交给 loader。UART heartbeat 的 `verifyfail` 记录此类回退次数。

启动策略为 cache probe 完成后 3 秒内完全静默运行期 PSRAM data path：E3 固定走物理 SD，E5 不复制或排队 backfill。计时和 64-bit 时间换算只在 core0 主循环执行；E3/E5 IRQ 只读取一个布尔 gate，使 mount/open 阶段与 probe-only 固件保持一致。窗口结束后同时启用 backfill 和 hit；因此缓存从空表开始逐步预热，而不是在 loader 启动期预热。

## 初始化顺序

`SdCard::TryInitialize()` 会先后两次调用 `rp2040_sdio_init()`，并且每次都会执行 `pio_clear_instruction_memory(pio1)`。因此顺序必须是：

1. 初始化并启用 PIO0 cartridge；
2. FatFs 初始化物理 SD；
3. SDIO 完成最终高速 `rp2040_sdio_init(1)`；
4. `romCacheInit()` 在 PIO1 安装 PSRAM program；
5. core1 异步执行 128 轮 PSRAM probe；
6. probe 成功前 cache 保持 unavailable，E3 全部按 miss 从 SD 读取。

不能把 PSRAM PIO 初始化移回物理 SD 初始化之前，否则下一次 `pio_clear_instruction_memory(pio1)` 会直接擦掉 PSRAM 的五条指令。

预期 UART 顺序为：

```text
[BOOT] SD card: mounted
PSRAM: hw init done (PIO1 SM2 ready)
PSRAM: probing (PIO1 SM2 command/write + SIO read, 128-round stress)...
PSRAM: PIO1 SM2 command/write + SIO read data path (core1 service)
PSRAM: probe OK (...), 8192 KB chip
PSRAM: SD cache enabled ...
[cache] SD hit=... miss=... rate=... | used=.../16384 ...
```

probe 是异步的，所以 NDS 已开始发命令时仍可能看到 probe 日志；此时 PSRAM 只在 PIO1 上运行，PIO0 cartridge 独立工作。

## 并发规则

- core0 负责 cartridge IRQ、SDIO 状态机和 cache drain 提交。
- core1 负责 PSRAM probe、所有后端（bit-bang/PIO）的运行期 PSRAM transaction，以及游戏阶段的加扰环生成。运行期 512-byte PSRAM 请求必须拆成单个 16/32-byte burst；每个 burst 之间优先把加扰环重新填满，禁止恢复整条 cache line 的连续服务。
- cache backfill 为非阻塞提交；core0 不等待，继续推进 SDIO，随后轮询完成并发布 tag。
- cache hit fetch 需要等待数据，但等待时保持 `PIO0_IRQ_0` 可抢占；若 core1 正在 backfill，则该 hit 回退到物理 SD，避免排队阻塞。
- E3/E5 IRQ 只做 tag/counter/memcpy，不允许在 IRQ 中同步访问 PSRAM。
- PIO1 的 SDIO SM0/SM1 与 PSRAM SM2 必须用 `dspicoPioSmSetEnabled()` / `dspicoPioSmInit()`；不能重新引入共享 CTRL 的非原子 read-modify-write。
- SDIO 只能覆盖 5..13；PSRAM 只能执行 0..4。
- `rp2040_sdio_init()` 不得在 PSRAM 安装完成后再次清空 PIO1，除非同时实现 PSRAM 停机和重新装载流程。

## UART 日志时序

UART 诊断构建不能直接在 core0 调用阻塞式 `printf`。115200 baud 下，probe 完成时的多行摘要约占用 20–30 ms；卡带 IRQ 虽仍能进入，但这段时间 `gSdCard.Update()` 无法推进，pico-loader 会连续得到 E4 not-ready 并可能显示 `Failed to mount sd card`。

当前 `LOG()` 先格式化到短临时缓冲，再尝试写入 2 KB SRAM ring，不等待 UART。core0 主循环每次只填充 UART 硬件 FIFO 当前可用的空间，随后立即继续 SDIO 和 cache drain；队列未清空时不进入休眠。core1 写日志时通过 `SEV` 唤醒 core0。新固件首行包含 `UART async log OK`，用来区分旧的阻塞日志固件。

日志队列满或跨核短锁正被占用时允许丢弃诊断字符，协议时序优先于日志完整性。UART 仍为 115200 baud。

## 回退行为

以下任一情况会让 PSRAM 后端退回 bit-bang：

- PIO1 SM2 已被占用；
- PIO1 没有可装入的 5-word 空间；
- 128 轮 PIO probe 失败。

probe 失败后先禁用 SM2并通过 SIO reset PSRAM，再用 bit-bang 重新 probe，避免把异常 command decoder 状态带入回退路径。

注意：SDIO 动态 RX/TX 布局由编译选项决定，即使 PSRAM 在运行时回退 bit-bang，SDIO 仍使用动态布局。若动态 SDIO 布局本身在真机上不稳定，应直接刷 `PSRAM_USE_PIO=OFF` 固件，而不是依赖 PSRAM probe 回退。

## 真机验证清单

PIO1 架构至少需要完成：

1. 断电冷启动 30 次，不能出现 loader mount/open 错误或白屏；
2. 菜单连续停留 10 分钟，3 秒 cache heartbeat 持续输出；
3. 连续进入、退出多个 NDS 游戏；
4. 游戏运行 30 分钟以上，观察命中率和是否关机；
5. 测试存档写入并重新读取，确认 cache invalidation 没有返回旧扇区；
6. 若失败，保留从上电第一行到故障后的完整 UART 日志，并注明冷启动/热重启、菜单或具体游戏阶段。

成功标准不是单次 `probe OK`，而是上述整机路径均不再复现 NDS 端错误。

## 构建配置

PIO1 + UART 诊断固件：

```powershell
cmake -S . -B build -DPSRAM_USE_PIO=ON -DENABLE_PSRAM_CACHE=ON -DENABLE_UART_LOG=ON
cmake --build build --parallel
```

稳定 bit-bang + UART 回退固件：

```powershell
cmake -S . -B build-codex-bitbang -DPSRAM_USE_PIO=OFF -DENABLE_PSRAM_CACHE=ON -DENABLE_UART_LOG=ON
cmake --build build-codex-bitbang --parallel
```

默认 `PSRAM_USE_PIO=OFF`。在 PIO1 方案完成真机压力测试前，不要修改这个默认值。

## 关键文件

- `src/psram.pio`：5-word PIO1 SM2 command/write pump。
- `src/psram.c`：PIO1 初始化、core1 服务、probe 和 bit-bang 回退。
- `src/sd/rp2040_sdio.cpp`：SDIO RX/TX 动态指令区以及 PIO1 原子 SM 控制。
- `src/pioUtil.h`：共享 PIO block 的原子 enable/disable 和 SM init。
- `src/main.cpp`：物理 SD 完成后再安装 PSRAM、core0/core1 调度。
- `src/romCache.c` / `src/romCache.h`：异步 E3/E5 sector cache 和 UART heartbeat。
