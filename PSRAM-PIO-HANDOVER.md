# PSRAM PIO 提速 — core0 自旋锁问题 HANDOVER

## 目标
把 PSRAM 数据路径从 bit-bang(64µs/512B)切回 PIO 泵(8µs/512B,8x 提速),用于 core0 的 SD 缓存命中/回填路径。当前因自旋锁会关 core0 中断而无法使用 PIO,只能退回 bit-bang。本文记录根因、所有约束、可行技术方向及取舍,供后续实现。

## 现状(2026-08-09,bit-bang 已验证可用)
- `PSRAM_FORCE_BITBANG=1`:所有 PSRAM 访问走 bit-bang(纯 SIO,不加锁、不关中断)。
- SD 缓存全异步:IRQ 里只做 ~1µs tag 检查/memcpy,真正的 psram_read/write 在 core0 main loop(`romCacheSdStoreDrain` / `ntrc_sdCacheFetchDrain`)。bit-bang 在 main loop 可被 PIO0_IRQ_0 随时抢占,不破坏卡带协议。
- PIO 泵(`psram_qspi_tx/rx`,pio0 SM2/SM3,clkdiv 3)**仅用于 core1 的 probe**(4ms,probe OK)。core1 上用 PIO 安全。
- 菜单/游戏正常,SD 缓存命中率 ~58%。

## 根因:PIO 路径的自旋锁关 core0 中断

### 证据链(已闭合)
1. PIO drain 在 core0 main loop(async,可抢占)仍破坏 mount("failed to mount SD card");bit-bang drain 不破坏。两者唯一区别:PIO 加锁,bit-bang 不加。
2. `psramPioLock()` → `spin_lock_blocking()` → SDK `spin_lock.h:301` `save_and_disable_interrupts()`(置 PRIMASK),**禁用调用核的全部中断,包括 PIO0_IRQ_0**。
3. 锁在 `psramTxSmWait()` / `pio_sm_put_blocking` / `pio_sm_get_blocking` **整个 burst 期间持有**(psram.c:242-254 write, 267-290 read)。每个 32B burst 关中断 ~4µs,512B = 16 次。
4. PIO0_IRQ_0 由 `pis_sm0_rx_fifo_not_empty` 触发(main.cpp:341):SM0 每把一个 NDS 命令字读入 RX FIFO 就触发。中断被关 → 命令字堆积未处理 → 协议失步 → loader 的 E3/E4/E5 SD mount 超时 → "failed to mount SD card"。
5. bit-bang 路径(`psramBbReadBurst`/`psramBbWriteBurst`)**完全不加锁、不关中断**(grep 确认为空),所以 core0 安全。
6. core1 probe 用 PIO 不破坏:`spin_lock_blocking` 在 core1 只关 core1 中断,core0 的 PIO0_IRQ_0 照常触发。这是 probe 一直正常的线索。
7. ntr_card.pio 的 SM0 大量 `wait gpio`(line 11-12,16-17,29-30,41-44),stall 时不占 pio0 执行槽 → PIO SM2/SM3 与 SM0 **无执行槽竞争**(早先"pio0 SM 竞争"假设被推翻)。

**结论:PIO0 硬件没问题,问题是 PIO 路径的自旋锁在 core0 关了 PIO0_IRQ_0。**

### 取锁点全景(争用方)
自旋锁 `sPioLock` 保护 pio0->ctrl 的 read-modify-write(SM0/SM2/SM3 的 enable/restart 共享一个 ctrl 寄存器)。取锁点:

| 位置 | 核 | 上下文 | 锁内做什么 | 是否跨等待 |
|---|---|---|---|---|
| psram.c:242 `psramPioWriteBurst` | core0 或 core1 | main loop / probe | MuxToPio, TX pump 填充, **psramTxSmWait**, MuxToSio | **是**(整个 burst) |
| psram.c:267 `psramPioReadBurst` | core0 或 core1 | main loop / probe | MuxToPio, TX pump, **psramTxSmWait**, RX pump 填充, **pio_sm_get_blocking** | **是**(整个 burst) |
| main.cpp:64 `resetNtrCard` | core0 | gpioIrq(RST 边沿) | SM0 disable/restart/clkdiv/jmp/enable | 否(快) |
| main.cpp:101 `gpioIrq` RST fall | core0 | IO_IRQ_BANK0 | SM0 disable + pindirs | 否(快) |
| main.cpp:117 `gpioIrq` DSI autoboot | core0 | IO_IRQ_BANK0 | SM0 disable | 否(快) |

core0 的三个取锁点都在**卡带 reset 边沿**(RST 下降/上升),频率低、锁内无等待。core0 上**唯一**跨等待持锁的是 PSRAM burst(若改用 PIO)。

### psramTxSmWait(锁内等待的实现)
```
while (pio_sm_get_tx_fifo_level(PSRAM_PIO, sTxSm) != 0);  // 等 TX FIFO 排空
for (int i = 0; i < 64; i++) __asm("nop");                 // 等 OSR 余量走完
pio_sm_set_enabled(PSRAM_PIO, sTxSm, false);               // 关 SM(RMW ctrl)
pio_sm_set_pindirs_with_mask(PSRAM_PIO, sTxSm, 0, PSRAM_IO_MASK);
```
锁必须跨这个等待:CE# 已拉低、命令/地址已发,PIO 正在出数据,必须等它出完才能关 SM、拉高 CE、交还引脚。中途放锁 = 另一核改 ctrl/引脚 = PSRAM 事务损坏。

## 技术方向(按推荐度排序)

### 方向 A:换成不关中断的 unsafe 自旋锁(最小改动,推荐先试)
**思路**:用 `spin_lock_unsafe_blocking` / `spin_unlock_unsafe` 替换 `spin_lock_blocking` / `spin_unlock`。unsafe 版**不调 `save_and_disable_interrupts`**,只做原子锁获取 + memfence,中断保持开启。

**改动**:psram.c 的 `psramPioLockImpl`/`psramPioUnlockImpl`(line 50-58)换实现。core0 的 resetNtrCard/gpioIrq 三处也跟着换(它们用同一对 wrapper,自动生效)。

**为什么对 pio0->ctrl 仍安全**:unsafe 锁仍是互斥的(原子 RMW 硬件 spinlock 寄存器),两个核不会同时 RMW ctrl。关中断本来只是为了"防止同核 IRQ 重入抢锁"——但 core0 的 IRQ 路径(resetNtrCard/gpioIrq)取锁时,如果被 PIO0_IRQ_0 抢占……**这就是死锁点,必须处理**:

**死锁风险(必须解决)**:
- core0 main loop 持 unsafe 锁跑 `psramTxSmWait`(等 SM)。
- 此时 RST 边沿触发 `gpioIrq`(IO_IRQ_BANK0)→ 它调 `psramPioLock()` → 同核重入 → **自死锁**(自己等自己持有的锁)。
- 或 PIO0_IRQ_0 抢占后,handler 里若再触发 resetNtrCard(走 RST rise)→ 同样重入死锁。

**解法**:
1. **core0 PSRAM burst 期间禁用 IO_IRQ_BANK0**(RST 中断),burst 结束恢复。RST 边沿抖动在 ~µs 级 PSRAM burst 期间丢失可接受(卡带 reset 本就有去抖/重试)。PIO0_IRQ_0 不碰 PSRAM 锁(ntrc_pioIrq 只读写 txf/rxf,见下),所以可保持开启。
2. 或:**PSRAM burst 持锁期间关 core0 中断,但用 `save_and_disable_interrupts` 替代 spinlock 关中断的"全程"语义**——只关极短的 ctrl RMW 段(pio_sm_set_enabled 那几条),burst 主体等待段不关。但 ctrl RMW 与 burst 主体必须原子(CE#/引脚连续性),分段会破坏 PSRAM 事务。**所以这条不可行**,只能走"禁 RST 中断"。

**关键前提验证(动手前先确认,已查证)**:
1. **PIO0_IRQ_0 handler(`ntrc_pioIrq`,ntrCardIrq.S)是否取 PSRAM 锁?** 已查:ntrc_pioIrq(line 12-42)只读 `PIO0_RXF0`(line 16)、写 `PIO0_TXF0`(经 handler),**不碰 pio0->ctrl,不取 PSRAM 锁**。✅ 所以 PIO0_IRQ_0 在 PSRAM burst 期间触发是安全的,不会死锁,也不会和 burst 争 ctrl。
2. **IO_IRQ_BANK0 上还有别的中断源吗?** 已查:只有 `PIN_RST` 启用(main.cpp:348 `gpio_set_irq_enabled(PIN_RST, ...)`,无其他 gpio),`gpioIrq` 也只处理 `PIN_RST`(main.cpp:96)。✅ 所以方向 A 里"禁 IO_IRQ_BANK0"**实际只影响 RST 中断**,无其他副作用,约束比预想更干净。
3. **结论**:方向 A 只需在 PSRAM burst 期间禁用 **IO_IRQ_BANK0**(RST),保留 PIO0_IRQ_0。

**预期收益**:core0 SD 缓存命中/回填 64µs→8µs(8x)。probe 不变(本就 PIO)。
**风险**:RST 中断在 PSRAM burst(~4µs/32B)期间被屏蔽,极端情况下 RST 边沿丢失。需实测 reset 可靠性。

### 方向 B:把 PSRAM burst 挪到 core1(最干净,改动大)
**思路**:core0 永不直接跑 PSRAM burst。core0 把"要读/写哪个扇区"丢给 core1,core1 用 PIO(关 core1 中断,无害)完成后通过标志/缓冲通知 core0。

**为什么干净**:core1 上 `spin_lock_blocking` 只关 core1 中断,core0 的 PIO0_IRQ_0 / IO_IRQ_BANK0 全程不受影响。彻底消除 core0 中断屏蔽。

**难点**:
- core1 当前职责:scrambler ring 填充(`scr_getNext32`,游戏 secure 模式下持续运行)。PSRAM 服务要和它分时。
- 需要一个 core0→core1 的请求队列(SD 扇区号 / 方向)+ core1→core0 的完成通知(数据就绪)。
- SD 缓存命中路径:E3(IRQ)记录请求 → core1 读 PSRAM 填 buffer → E4 报 ready。延迟 = core1 响应时间 + PIO 8µs,需保证不超 NDS 的 E4 轮询容忍度。
- core1 在 scrambler 忙时(secure 游戏)能否及时服务 PSRAM?DQ9 汉化走 R4/B6 非 secure,scrambler 可能不忙,有窗口。但需确认。
- 双核共享 `sSdSectorBuf` / async 缓冲的同步(已有 volatile 标志基础,可扩展)。

**预期收益**:同方向 A,且无 RST 中断屏蔽的副作用。
**风险**:core1 时序编排复杂,scrambler 与 PSRAM 服务的优先级/抢占需设计。工作量最大。

### 方向 C:拆分 PIO 事务,锁只护 ctrl RMW(不可行,记录排除)
**思路**:锁只在 `pio_sm_set_enabled`/`restart` 这几条 ctrl RMW 时持有,burst 主体(等 FIFO/SM)不持锁。
**为何不可行**:PSRAM 事务要求 CE# 低 → 命令 → 地址 → 数据 连续不可中断;引脚在 SIO(命令/CE)和 PIO(数据)间 mux。burst 主体放锁期间,另一核若 mux 引脚或改 ctrl,会撕裂事务。除非整个事务独占引脚+ctrl,否则损坏。**已排除。**

## 推荐实施路径

1. **先做方向 A**(unsafe 锁 + burst 期间禁 IO_IRQ_BANK0)。改动集中在 psram.c 的 lock wrapper + 两个 burst 函数加 RST 中断屏蔽。约 20-40 行。
2. 实测:PIO+cache 全开,菜单/游戏/reset 是否正常;对比 bit-bang 的 SD 缓存命中率(应接近,延迟降低)。
3. 若 RST 中断屏蔽导致 reset 不可靠(方向 A 的唯一风险),再转**方向 B**(core1 服务)。

## 待确认/已确认事项
（“关键前提验证”已查证 2 件；本节第 1、3 项已确认/分析可接受，真正待实测的只有第 2 项：burst 期间 RST 中断丢失）
1. **IO_IRQ_BANK0 屏蔽 API（已确认）**:`irq_set_enabled(IO_IRQ_BANK0, false/true)` 在 burst 前后调用。RST 确认挂在 BANK0（main.cpp:349, 348），且是 BANK0 唯一中断源。
2. **burst 持锁期间 RST 中断丢失的影响**:RST 上升沿触发 `resetNtrCard`(重新 init 卡带)。若丢失,NDS 侧 reset 后卡带状态可能不对。看是否 NDS 会重发 RST 或有超时重试。可能需在 burst 结束后补查 RST 电平。
3. **core1 probe 并发（已分析，可接受）**:probe 在 core1（关 core1 中断），方向 A 后 core0 缓存 burst 也取 unsafe 锁。unsafe 锁跨核互斥，probe(4ms) 与 core0 burst(~4µs) 互相串行化——probe 会被切成 ~4µs 片，core0 burst 偶尔等 probe 让出。两者都跨 psramTxSmWait 持锁，无 starvation（硬件 spinlock 公平轮转）。可接受，无需特殊处理。

## 相关文件
- `src/psram.c`:lock wrapper(50-70)、PIO burst(229-293)、`psramTxSmWait`(219)、`psram_init_hw`/`PSRAM_FORCE_BITBANG`(516-535)。
- `src/main.cpp`:core0 取锁点 resetNtrCard(64)、gpioIrq(101,117);PIO0_IRQ_0 注册(341-342)。
- `src/romCache.c`:`romCacheSdStoreDrain` / `romCacheSdReadCached`(core0 main loop PSRAM 访问点)。
- `src/ntrCardIrq.S`:`ntrc_pioIrq`(已确认不碰 ctrl/不取锁)。
- `src/ntrCard.pio`:SM0 `wait gpio`(确认无执行槽竞争)。
- SDK:`/home/jerry/pico-sdk/src/rp2_common/hardware_sync_spin_lock/include/hardware/sync/spin_lock.h`(`spin_lock_unsafe_blocking`/`spin_unlock_unsafe`)。

## 当前配置(回退基线)
若方向 A 失败,回退到 `PSRAM_FORCE_BITBANG=1`(已验证可用)。三档:`build`(log+psram)、`build-nolog`、`build-nopsram`。env: `PICO_SDK_PATH=/home/jerry/pico-sdk`,`PICOTOOL_FETCH_FROM_GIT_PATH=/home/jerry/PS2-CTL-BRIDGE/build/_deps`。
