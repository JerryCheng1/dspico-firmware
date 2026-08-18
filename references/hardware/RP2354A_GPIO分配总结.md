# RP2354A GPIO 分配总结

## 1. 分析来源

- 网表文件：`Netlist_Schematic1_1_2026-08-18.tel`
- MCU：RP2354A
- 封装：QFN-60，7 mm × 7 mm
- 整理日期：2026-08-18

RP2354A 与 RP2350A 的 QFN-60 封装引脚定义相同。本文中的 GPIO 编号由网表内的 U1 实体脚号，按 Raspberry Pi 官方 RP2350/RP2354 数据表换算得到。

官方资料：

- [RP2350/RP2354 Datasheet](https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf)
- [Hardware design with RP2350](https://datasheets.raspberrypi.com/rp2350/hardware-design-with-rp2350.pdf)

## 2. GPIO 总体分配

| 引脚资源 | 用途 |
|---|---|
| GPIO0 | PSRAM 时钟 |
| GPIO1～GPIO2 | 空闲 |
| GPIO3～GPIO8 | SD 卡，4-bit SDIO |
| GPIO9～GPIO21 | NDS 模拟卡带接口 |
| GPIO22～GPIO29 | PSRAM 数据与片选 |
| USB_DM / U1.51 | 复用为 UART1_RX |
| USB_DP / U1.52 | 复用为 UART1_TX |

RP2354A 的 30 个 Bank 0 GPIO 中已使用 28 个，剩余 GPIO1、GPIO2。USB_DP、USB_DM 属于额外的 Bank 1 多功能引脚，不计入这 30 个 GPIO，目前被合法复用为 UART1。

## 3. SD 卡 GPIO 分配

SD 卡使用 4-bit SDIO 接口，连接器为 `SIM1`。

| SD 信号 | GPIO | U1 实体脚 | SIM1 引脚 | 外围电路 |
|---|---:|---:|---:|---|
| CLK_SD | GPIO3 | 5 | 5 | 串联 R11，22Ω |
| CMD_SD | GPIO4 | 7 | 3 | R6，10kΩ 上拉到 3.3V |
| D0_SD | GPIO5 | 8 | 7 | R9，10kΩ 上拉到 3.3V |
| D1_SD | GPIO6 | 9 | 8 | R10，10kΩ 上拉到 3.3V |
| D2_SD | GPIO7 | 10 | 1 | R8，10kΩ 上拉到 3.3V |
| D3_SD | GPIO8 | 12 | 2 | R7，10kΩ 上拉到 3.3V |

补充：

- `SIM1.4` 接 3.3V。
- `SIM1.6` 及屏蔽/固定脚接 GND。
- CMD 和 D0～D3 均设置了外部上拉。
- RP2354A 没有专用 SDIO 控制器，这组接口通常需要使用 PIO 或软件时序实现。
- GPIO3～GPIO8 连续排列，便于固件组织 SDIO 信号。

## 4. NDS 模拟卡带接口 GPIO 分配

本设计中 RP2354A 作为 NDS Slot-1 模拟卡带，而不是卡带读取器。NDS/DSi 主机是总线主机，RP2354A 必须接收主机命令、模拟 ROM 与存档芯片，并在规定时序内向主机返回数据。

下表中的方向均以 RP2354A（模拟卡带）为参照：

| NDS 信号 | GPIO | U1 实体脚 | RP2354A 方向 | 模拟卡带作用 |
|---|---:|---:|---|---|
| RST_DS | GPIO9 | 13 | 输入 | 接收主机的卡带复位信号 |
| ROM_CS | GPIO10 | 14 | 输入 | 接收主机的 ROM 总线片选，通常低有效 |
| CLK_DS | GPIO11 | 15 | 输入 | 接收主机提供的卡带总线时钟 |
| D0_DS | GPIO12 | 16 | 双向 | ROM 命令及响应数据位 0 |
| D1_DS | GPIO13 | 17 | 双向 | ROM 命令及响应数据位 1 |
| D2_DS | GPIO14 | 18 | 双向 | ROM 命令及响应数据位 2 |
| D3_DS | GPIO15 | 19 | 双向 | ROM 命令及响应数据位 3 |
| D4_DS | GPIO16 | 27 | 双向 | ROM 命令及响应数据位 4 |
| D5_DS | GPIO17 | 28 | 双向 | ROM 命令及响应数据位 5 |
| D6_DS | GPIO18 | 29 | 双向 | ROM 数据位 6；存档 SPI 模式下主要作为卡带到主机的数据 |
| D7_DS | GPIO19 | 31 | 双向 | ROM 数据位 7；存档 SPI 模式下主要作为主机到卡带的数据 |
| IRQ | GPIO20 | 32 | 卡带侧输出/低电平指示 | 模拟卡带存在/IRQ 行为；零售卡通常将该线保持低电平 |
| SPI_CS | GPIO21 | 33 | 输入 | 接收主机的存档 EEPROM/Flash 片选，通常低有效 |

模拟卡带工作流程：

1. 主机拉低 `ROM_CS`，并提供 `CLK_DS`。
2. RP2354A 先将 D0～D7 配置为输入，接收主机发送的 8 字节 ROM 命令。
3. 命令阶段结束后，总线方向反转；RP2354A 将 D0～D7 配置为输出，返回 ROM 数据或协议响应。
4. 主机释放 `ROM_CS` 后，RP2354A 应及时将数据总线恢复为高阻输入，避免总线冲突。
5. 主机拉低 `SPI_CS` 时，RP2354A 模拟卡带内的存档 EEPROM/Flash；D7 接收主机命令/写入数据，D6 返回读取数据。

实现注意事项：

- NDS 模拟卡带接口完整占用 GPIO9～GPIO21。
- D0～D7 对应 GPIO12～GPIO19，八位数据总线连续，适合 PIO 进行并行输入、方向切换和响应输出。
- `CLK_DS`、`ROM_CS`、`SPI_CS`、`RST_DS` 都是主机驱动的输入，固件不能反向驱动这些线路。
- D0～D7 的输入/输出切换时机很关键；进入响应阶段前才能打开 RP2354A 输出，片选释放后应立即进入高阻状态。
- `IRQ` 不宜默认作为普通推挽高电平输出。应根据目标主机兼容性，模拟零售卡的低电平/接地检测行为，必要时采用只拉低或开漏方式。
- 当前 `.tel` 网表中，这些信号只显示连接到 U1，没有显示卡带金手指或连接器端点。
- `3V3_DS` 在网表中只显示连接到保险丝 F1 输出。因此该表确认的是 MCU 端预定分配，尚未核实卡带金手指实体脚序及完整供电连接。

NDS 卡带协议参考：

- [GBATEK - NDS Gamecard Bus](https://mgba-emu.github.io/gbatek/)
- [DS Game Card pinout](https://www.hardwarebook.info/DS_Game_Card)

## 5. PSRAM GPIO 分配

设计包含四片 USON-8 PSRAM：`U2`、`U4`、`U5`、`U6`。四片共用时钟及四根数据线，每片具有独立片选。

| PSRAM 信号 | GPIO | U1 实体脚 | 连接对象 | 外围电路 |
|---|---:|---:|---|---|
| SCLK_PSRAM | GPIO0 | 2 | 四片 PSRAM 的 pin 6 | 串联 R12，22Ω |
| SIO0_PSRAM | GPIO22 | 34 | 四片 PSRAM 的 pin 5 | 直接连接 |
| SIO1_PSRAM | GPIO23 | 35 | 四片 PSRAM 的 pin 2 | 直接连接 |
| SIO2_PSRAM | GPIO24 | 36 | 四片 PSRAM 的 pin 3 | 直接连接 |
| SIO3_PSRAM | GPIO25 | 37 | 四片 PSRAM 的 pin 7 | 直接连接 |
| CE0_PSRAM | GPIO26 | 40 | U2 pin 1 | R3，4.7kΩ 上拉 |
| CE1_PSRAM | GPIO27 | 41 | U4 pin 1 | R4，4.7kΩ 上拉 |
| CE2_PSRAM | GPIO28 | 42 | U5 pin 1 | R5，4.7kΩ 上拉 |
| CE3_PSRAM | GPIO29 | 43 | U6 pin 1 | R13，4.7kΩ 上拉 |

特点与注意事项：

- GPIO22～GPIO25 是连续的四位 SIO 数据总线。
- GPIO26～GPIO29 是连续的四路独立片选。
- 每路 CE 均有外部 4.7kΩ 上拉，能够保证上电期间 PSRAM 保持未选中状态。
- 这组 PSRAM 接在普通 Bank 0 GPIO 上，不是 RP2354A 的专用 QSPI/QMI XIP 引脚，通常需要 PIO 或自定义软件驱动。
- GPIO0 可作为 PIO side-set 时钟，GPIO22～GPIO25 可作为连续的四位数据组。

## 6. UART1 复用确认

网表中的 UART 分配是有效的，不需要改到 GPIO1、GPIO2。

| 网表信号 | U1 实体脚 | 芯片专用名称 | Bank 1 复用功能 | FUNCSEL |
|---|---:|---|---|---:|
| UART_RX | 51 | USB_DM | UART1_RX | `0x02` |
| UART_TX | 52 | USB_DP | UART1_TX | `0x02` |

官方数据表第 9 章的 Bank 1 功能表及寄存器定义明确给出：

- `USBPHY_DP_CTRL.FUNCSEL = 0x02`：选择 `UART1_TX`。
- `USBPHY_DM_CTRL.FUNCSEL = 0x02`：选择 `UART1_RX`。

使用限制：

- 这里连接的是硬件 UART1，不是 UART0。
- UART1 与 USB 功能不能在这两个引脚上同时使用。
- U1.51、U1.52 是封装实体脚号，不能称为 GPIO51、GPIO52。
- USB_DP、USB_DM 属于 Bank 1 特殊多功能引脚，其电气特性与普通 Bank 0 GPIO 不完全相同。
- 这些引脚不是 FT 类型，不应按 5V 容忍 GPIO 使用；外部 UART 应使用兼容的 3.3V 逻辑电平。
- 网表中 U1.53 `USB_OTP_VDD` 已连接 3.3V，满足该引脚域供电要求。
- 数据表规定的 27Ω 串联电阻是 USB 工作时的要求，复用为普通 UART 时并非强制。

## 7. 最终资源汇总

| 外设 | 使用资源 | 数量 |
|---|---|---:|
| PSRAM | GPIO0、GPIO22～GPIO29 | 9 个 Bank 0 GPIO |
| SD 卡 | GPIO3～GPIO8 | 6 个 Bank 0 GPIO |
| NDS 模拟卡带 | GPIO9～GPIO21 | 13 个 Bank 0 GPIO |
| UART1 | USB_DM、USB_DP | 2 个 Bank 1 特殊引脚 |
| 空闲 | GPIO1、GPIO2 | 2 个 Bank 0 GPIO |

Bank 0 使用总数：`9 + 6 + 13 = 28`。

整体分配特点是：SD、NDS 模拟卡带和 PSRAM 数据/片选均按连续 GPIO 分组，适合 PIO 实现；GPIO1、GPIO2 可保留给调试、额外控制信号或其他低速外设。NDS 部分的固件角色必须按“卡带从设备”设计，而不能按卡带读取器或总线主机设计。
