# Zephyr YTM32 Module

[🇨🇳 中文](README_zh.md) | [🇬🇧 English](README.md)

> **Experimental project**
>
> This is an experimental, community-maintained YTM32 Zephyr module. It is not an official YTMicro/云途 supported project and is still under active development. APIs, board support, drivers, and repository layout may change.

This repository is an out-of-tree Zephyr module containing the System-on-Chip (SoC) support, board definitions, drivers, and device-tree files for YTMicro (YTM32) microcontrollers.

## Start with `ytm32-zephyr-starter`

Use [`ytm32-zephyr-starter`](https://github.com/jinyi001/ytm32-zephyr-starter) to create a complete YTM32 Zephyr workspace. Do not start by manually copying this repository into a Zephyr tree or by editing an upstream `west.yml`.

Follow the starter README to:

1. Create an independent project from the template repository;
2. Initialize the West workspace with Workbench for Zephyr or `west`;
3. Install Zephyr, the YTMicro HAL, this module, the SDK, and the required tools;
4. Build, flash, and debug the selected YTM32 board.

The starter manifest pins the Zephyr, HAL, and YTM32 module revisions so that a new workspace is reproducible:

```text
https://github.com/jinyi001/ytm32-zephyr-starter
```

This module is a platform dependency of the starter, not the application entry point. Keep application code and product-specific board integration in the independent application repository created from the starter.

## Supported hardware

Currently supported:

- **YTM32B1MC0 EVB** (`ytm32b1mc0_evb`)

Additional boards and SoCs are still being developed.

## Build module samples after initializing the starter workspace

After the starter workspace has been initialized, run commands from the workspace root. The module is normally located at `modules/ytmicro/zephyr-ytm32`.

Build the module `hello_world` sample:

```bash
west build \
  -b ytm32b1mc0_evb \
  modules/ytmicro/zephyr-ytm32/samples/hello_world \
  -d build/ytm32-module-hello
```

For a pristine build:

```bash
west build -p always \
  -b ytm32b1mc0_evb \
  modules/ytmicro/zephyr-ytm32/samples/hello_world \
  -d build/ytm32-module-hello
```

To validate the GPIO compatibility layer with the upstream sample, build:

```bash
west build -p always \
  -b ytm32b1mc0_evb \
  zephyr/samples/basic/blinky \
  -d build/ytm32-blinky
```

## Flashing and debugging

The board uses SWD and a J-Link runner. Install the SEGGER J-Link software and the YTM32 device patch as described by the starter README, then run:

```bash
west flash -r jlink -d build/ytm32-module-hello
west debug -d build/ytm32-module-hello
```

The default console is routed to the board UART. SEGGER RTT can be enabled by the application configuration when UART is unavailable:

```conf
CONFIG_USE_SEGGER_RTT=y
CONFIG_RTT_CONSOLE=y
CONFIG_UART_CONSOLE=n
```

## Repository structure

- `boards/` — board configuration and device-tree definitions;
- `soc/` — YTM32 SoC initialization and glue code;
- `drivers/` — YTMicro peripheral drivers;
- `dts/` — device-tree bindings and SoC descriptions;
- `samples/` — module validation samples;
- `zephyr/` — Zephyr module metadata and Kconfig files.

## Development boundary

Put SoC support, board definitions, and generic YTM32 drivers in this repository. Put vendor HAL updates in `hal_ytmicro` by replacing the vendor snapshot with a new upstream release; do not add application logic here.

When changing this module, update the pinned revision in the starter manifest and test a clean starter workspace before publishing a new starter version.
