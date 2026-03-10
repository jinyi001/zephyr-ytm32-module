# Zephyr YTM32 模块

[🇨🇳 中文](README_zh.md) | [🇬🇧 English](README.md)

本仓库是一个 Zephyr 的“树外（out-of-tree）”模块，包含针对 **YTMicro (YTM32)** 系列微控制器的系统级芯片 (SoC) 支持、开发板定义、驱动程序以及设备树 (DTS) 文件。

通过将 YTM32 的实现作为独立模块维护，您可以将其无缝集成到上游的 Zephyr 中，而无需直接修改 Zephyr 核心代码库。

## 支持的硬件

目前支持以下开发板：
- **YTM32B1MC0 EVB** (`ytm32b1mc0_evb`)

## 前提条件

在开始之前，请确保您已经配置好了可工作的 Zephyr 工具链和开发环境。如果尚未配置，请参考官方的 [Zephyr 入门指南](https://docs.zephyrproject.org/latest/develop/getting_started/index.html)。

## 如何使用此模块

### 1. 更新 `west.yml` 清单

要将此模块包含到您的 Zephyr 工作区中，您需要将本模块连同厂商的硬件抽象层 (`hal_ytmicro`) 一起添加到工作区的 `west.yml` 清单文件中。

打开 `zephyr/west.yml`（或您的工作区清单文件），并在 `projects:` 部分下添加以下内容：

```yaml
    - name: hal_ytmicro
      path: modules/hal/ytmicro
      url: https://github.com/jinyi001/hal_ytmicro.git
      revision: main
    - name: zephyr-ytm32-module
      path: zephyr-ytm32-module
      url: https://github.com/jinyi001/zephyr-ytm32-module.git
      revision: main
```

### 2. 更新 West 工作区

更新清单文件后，使用 `west update` 获取新添加的代码库：

```bash
# 在您的 zephyrproject 根目录下执行
west update
```

这将会把 `hal_ytmicro` 和 `zephyr-ytm32-module` 克隆到您的工作区中。

## 编译示例程序

您可以编译本模块提供的示例，或是针对 YTM32 开发板标准的 Zephyr 示例。

从本仓库构建 `hello_world` 示例：

```bash
# 在 zephyrproject 根目录下执行
west build -b ytm32b1mc0_evb zephyr-ytm32-module/samples/hello_world
```

如果您希望进行一次干净的构建（清除先前的构建产物），可以加上 `-p` 标志：
```bash
west build -p -b ytm32b1mc0_evb zephyr-ytm32-module/samples/hello_world
```

## 烧录与调试

### 终端输出
默认情况下，`ytm32b1mc0_evb` 的终端输出被映射到板载的 UART（通常是通过 USB 的 UART1）。请确保您打开了正确的 COM 端口，并设置了开发板设备树中配置的波特率（通常为 `115200`）。

另外，当 UART 不可用时，本项目支持通过 **SEGGER RTT** 路由打印终端以便进行调试。您可以通过在配置中设置以下选项来开启：
```kconfig
CONFIG_USE_SEGGER_RTT=y
CONFIG_RTT_CONSOLE=y
CONFIG_UART_CONSOLE=n
```

### 烧录
您可以使用标准的 SWD 调试器（如 Segger J-Link）来烧录代码。具体取决于您的设置和 `west flash` 配置，运行：

```bash
west flash
```
*(请确保您的调试器已正确连接到开发板的调试排针上。)*

## 仓库结构

- `boards/` - 开发板配置及设备树定义（例如 `ytm32b1mc0_evb`）
- `soc/` - YTM32 SoC 特定的初始化及胶水层代码
- `drivers/` - 针对 YTMicro 外设的树外驱动程序（例如中断控制器）
- `dts/` - 专用于 YTMicro 硬件的设备树绑定说明
- `samples/` - 演示相关功能的示例应用程序
- `zephyr/` - Zephyr 模块配置 (`module.yml`) 及 Kconfig 文件

## 参与贡献
当需要修改胶水代码或移植相关部分时，请直接将提交通知提交到本仓库。这样可以保证上游的 Zephyr 组件的整洁，任何专门为 YTM32 增加的内容应保存在本仓库或 `hal_ytmicro` 中。
