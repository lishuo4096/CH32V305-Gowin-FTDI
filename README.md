# CH32V305 Gowin FTDI

使用 **CH32V305GBU6** 实现 FTDI 兼容 USB/JTAG 接口，尝试替代原装 FT232HQ 在高云 FPGA Programmer 和 GAO 在线逻辑分析中的作用。

当前为实板验证中的实验版本。**嵌入式 Flash 建议使用 2.5MHz 请求档**：本轮完整擦除、编程、校验 3/3 通过；3MHz 为 3/4，仍有偶发擦除失败。请求时钟与实际 TCK 的关系见下文。

## 当前验证结果

测试器件为 GW1N-9C（板上 GW1N9C），正常 IDCODE `0x1100481B`；高云软件版本为 V1.9.12.02 SP2。

| 功能 | 验证结果 |
|---|---|
| 枚举与器件识别 | FTDI 驱动、Gowin Programmer 可识别，IDCODE 正确 |
| SRAM | 已通过烧写及独立校验；GAO 镜像另在 5/10/15MHz 请求档下载通过 |
| 嵌入式 Flash，2.5MHz 请求 | 最近一轮完整擦除、编程、校验 3/3，通过耗时约 23～24 秒 |
| 嵌入式 Flash，3MHz 请求 | 3/4 完整通过，1 次擦除失败，暂不推荐作为工作档 |
| 嵌入式 Flash，3.333333MHz 请求 | 原生测试擦除失败；带观测测试曾通过，原因未确定 |
| GAO，2.5MHz 请求 | 100 次独立打开、采集、关闭，102,400 个样本校验通过 |
| GAO，3.75/5/7.5/10/15MHz 请求 | 每档 100 次采集通过；批量 SPI 实际为 4.5/4.5/9/9/18MHz |

这些结果针对当前板卡和有限次数测试；GAO 的高速通过不能代替 Flash 烧写验证。UART 通道已实现，本轮没有作为主要验证目标。详细过程和日志见 [验证记录](docs/validation.md)。

## 硬件连接

MCU 使用 16MHz 外部晶振，系统时钟为 144MHz。接口信号按当前 3.3V 板卡连接并共地。

| CH32V305 引脚 | FPGA / 功能 | 逻辑分析仪 |
|---|---|---|
| PA5 | TCK | D0 |
| PA7 | TDI | D1 |
| PA6 | TDO | D2 |
| PB12 | TMS | D3 |
| GND | GND | GND |
| PB10 | UART TX | 可选 |
| PB11 | UART RX | 可选 |

实测板上原装 FT232 固定焊接，测试时通过 JTAG 飞线连接 CH32；原装 FT232 不上电，已在该板确认其 JTAG 引脚处于高阻。换用其他板卡时需确认总线不会被两个适配器同时驱动。

USBHS 使用芯片原生 USB 接口。固件通过 WCH-Link 的 RISC-V 调试接口烧入 CH32，JTAG 飞线则连接 FPGA；两者用途不同。

## USB 与驱动

当前描述符为 `VID:PID = 0403:6010`、`bcdDevice = 0500`，采用 FT2232D 风格的双接口枚举和 FTDI 状态前缀，并实现用于高云软件识别的字符串与 EEPROM 响应。项目名中的 FTDI/FT232HQ 表示替代目标，枚举身份并非原装 FT232HQ。

通道 A 为 MPSSE/JTAG（OUT `0x02`、IN `0x81`），通道 B 为 UART（OUT `0x04`、IN `0x83`）。驱动使用用户已安装的 FTDI 驱动，本仓库不分发驱动安装包。

## 获取与编译

预编译的实测固件见 [firmware/](firmware/)；同一份固件也提供在 [Releases](https://github.com/lishuo4096/CH32V305-Gowin-FTDI/releases)。这是发布时的当前版本，没有为发布修改 JTAG 实现。

工程保留 MounRiver 的 `.project`、`.cproject` 和 `.template`。使用 MounRiver Studio 2 自带的 **RISC-V Embedded GCC 8.2.0** 和 GNU make，也可直接命令行编译。工程参数为 `-march=rv32imacxw -mabi=ilp32`，启动文件为 `Startup/startup_ch32v30x_D8C.S`；链接布局为 128KB Flash、32KB RAM，栈 2KB。

以下 PowerShell 命令使用默认安装路径；安装位置不同时修改 `$mrs`：

```powershell
$mrs = 'C:/MounRiver/MounRiver_Studio2'
$toolchain = "$mrs/resources/app/resources/win32/components/WCH/Toolchain/RISC-V Embedded GCC/bin"
$make = "$mrs/resources/app/resources/win32/others/Build_Tools/Make/bin/make.exe"
& $make -j8 "TC=$toolchain" all
```

输出位于 `build/Alternative_FT232H.{elf,hex,bin,map,lst}`。Makefile 使用 Windows 命令行创建输出目录，请在 Windows 构建；尚未验证 Linux 构建。发布前独立全量重建无错误、无告警，BIN 与实板验证的固件逐字节一致，校验和见 [firmware/SHA256SUMS.txt](firmware/SHA256SUMS.txt)。

将 HEX 或 ELF 通过支持 CH32V305 的 WCH-Link 工具烧入 MCU 并复位，然后用高云 Programmer 选择 GW1N-9C，先从 2.5MHz 请求档开始测试。

## 实现结构

- `User/usb_desc.c`、`ftdi_usb.c`：USB 描述符、FTDI 控制请求、状态前缀和通道收发。
- `User/mpsse.c`：MPSSE 命令解析、TAP 状态跟踪、GPIO/SPI/TIM 组合执行。
- `User/ch32v30x_usbhs_device.c`：USBHS 端点与 DMA 缓冲。
- `User/cdc_uart.c`：通道 B 与 USART3 桥接。
- `Peripheral/`、`Core/`、`Startup/`、`Debug/`：随工程保留的 WCH SDK 源文件。

Flash 的关键改动是将一个完整的单字 JTAG 编程事务缓存后，通过 TIM1 + 两路 DMA 连续驱动 TCK/TDI/TMS；长数据移位仍使用 SPI，短控制操作使用 GPIO。擦除所需的长 Run-Test/Idle 时钟有专用连续输出路径。更多细节见 [架构说明](docs/architecture.md)。

请求频率不等于所有路径的实际频率：2.5MHz 请求对应单字 TIM 路径约 2.483MHz、SPI 路径 2.25MHz；TIM 编程路径当前上限为 4MHz。不要将 GAO 测得的 18MHz 批量 SPI 速度理解为 Flash 全流程速度。

## License

项目自有代码和文档采用 [MIT](LICENSE)。WCH SDK 文件及修改后的 SDK 文件保留原版权与使用声明，详见 [NOTICE.md](NOTICE.md)。本项目参考 FT-LINK、RV-Debugger-BL702 等实现；本地参考副本、原始 USB/逻辑分析仪大文件与旧实验代码不随仓库发布。

## English

An experimental CH32V305GBU6 USB/JTAG adapter for Gowin Programmer and GAO, using FTDI-compatible control requests and MPSSE commands. The current firmware enumerates as a dual-interface FT2232D-style device (`0403:6010`), while the replacement target is the board's FT232HQ-based cable.

The latest native embedded-Flash tests passed erase/program/verify **3/3 at a requested 2.5MHz**. At 3MHz, 3/4 passed and one erase failed. GAO bulk transfers reached 18MHz actual SPI TCK, but this does not establish Flash operation at that speed. Start Flash tests at 2.5MHz. See [validation](docs/validation.md), [architecture](docs/architecture.md), [firmware](firmware/) and the [MIT license](LICENSE) with retained [third-party notices](NOTICE.md).
