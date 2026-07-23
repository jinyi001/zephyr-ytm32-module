# Zephyr YTM32 模块

[🇨🇳 中文](README_zh.md) | [🇬🇧 English](README.md)

> **实验性项目声明**
>
> 本仓库是实验性、社区维护的 YTM32 Zephyr 模块，非云途官方支持项目，目前仍在持续开发中。API、开发板支持、驱动实现和仓库结构都可能发生变化。

本仓库是 Zephyr 的树外（out-of-tree）模块，包含 YTMicro（YTM32）系列微控制器的 SoC 支持、开发板定义、驱动和设备树文件。

## 从 `ytm32-zephyr-starter` 开始

请使用 [`ytm32-zephyr-starter`](https://github.com/jinyi001/ytm32-zephyr-starter) 创建完整的 YTM32 Zephyr 工作区。不要从手动复制本仓库到 Zephyr 目录开始，也不要手动编辑上游 `west.yml`。

按照 starter 的 README 执行以下步骤：

1. 从模板仓库创建独立的应用工程；
2. 使用 Workbench for Zephyr 或 `west` 初始化 West workspace；
3. 自动安装 Zephyr、YTMicro HAL、本模块、SDK 和所需工具；
4. 构建、烧录和调试目标 YTM32 开发板。

starter 的 manifest 会固定 Zephyr、HAL 和 YTM32 module 的版本，保证新建工作区可复现：

```text
https://github.com/jinyi001/ytm32-zephyr-starter
```

本仓库是 starter 的平台依赖，不是应用入口。应用代码和产品相关的板级集成应放在从 starter 创建的独立应用仓库中。

## 支持的硬件

当前支持：

- **YTM32B1MC0 EVB**（`ytm32b1mc0_evb`）

更多芯片和开发板仍在开发中。

## 在 starter 工作区中编译模块示例

完成 starter 工作区初始化后，在 workspace 根目录执行命令。本模块通常位于 `modules/ytmicro/zephyr-ytm32`。

编译本模块的 `hello_world` 示例：

```bash
west build \
  -b ytm32b1mc0_evb \
  modules/ytmicro/zephyr-ytm32/samples/hello_world \
  -d build/ytm32-module-hello
```

执行 pristine build：

```bash
west build -p always \
  -b ytm32b1mc0_evb \
  modules/ytmicro/zephyr-ytm32/samples/hello_world \
  -d build/ytm32-module-hello
```

验证 GPIO 兼容层时，建议直接编译上游 `blinky` 示例：

```bash
west build -p always \
  -b ytm32b1mc0_evb \
  zephyr/samples/basic/blinky \
  -d build/ytm32-blinky
```

## 烧录和调试

开发板使用 SWD 和 J-Link runner。按照 starter README 安装 SEGGER J-Link 软件及 YTM32 device patch，然后执行：

```bash
west flash -r jlink -d build/ytm32-module-hello
west debug -d build/ytm32-module-hello
```

默认终端输出走板载 UART。当 UART 不可用时，可在应用配置中启用 SEGGER RTT：

```conf
CONFIG_USE_SEGGER_RTT=y
CONFIG_RTT_CONSOLE=y
CONFIG_UART_CONSOLE=n
```

## 仓库结构

- `boards/` — 开发板配置和设备树定义；
- `soc/` — YTM32 SoC 初始化和胶水层代码；
- `drivers/` — YTMicro 外设驱动；
- `dts/` — 设备树绑定和 SoC 描述；
- `samples/` — 模块验证示例；
- `zephyr/` — Zephyr 模块元数据和 Kconfig 文件。

## 开发边界

SoC 支持、开发板定义和通用 YTM32 驱动放在本仓库。厂商 HAL 更新应在 `hal_ytmicro` 中用新的原厂版本替换旧快照；不要在本仓库放置应用逻辑。

修改本模块后，需要更新 starter manifest 中固定的 revision，并在全新的 starter 工作区中完成验证，再发布新的 starter 版本。
