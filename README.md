# Zephyr YTM32 Module

[🇨🇳 中文](README_zh.md) | [🇬🇧 English](README.md)

This repository is an out-of-tree Zephyr module containing the System-on-Chip (SoC) support, board definitions, drivers, and device tree (DTS) files for the **YTMicro (YTM32)** series microcontrollers.

By keeping the YTM32 implementation as an out-of-tree module, you can seamlessly integrate it with upstream Zephyr without modifying the core Zephyr repository directly. 

## Supported Hardware

Currently, the following development boards are supported:
- **YTM32B1MC0 EVB** (`ytm32b1mc0_evb`)

## Prerequisites

Before getting started, make sure you have a working Zephyr toolchain and development environment. If not, follow the official [Zephyr Getting Started Guide](https://docs.zephyrproject.org/latest/develop/getting_started/index.html).

## How to use this module

### 1. Update your `west.yml` manifest

To include this module in your Zephyr workspace, you need to add it along with the vendor HAL (`hal_ytmicro`) directly to your workspace `west.yml` manifest file. 

Open `zephyr/west.yml` (or your workspace manifest) and add the following entries under the `projects:` section:

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

### 2. Update West workspace

After updating the manifest, fetch the newly added repositories using `west update`:

```bash
# In your zephyrproject root directory
west update
```

This will clone both the `hal_ytmicro` and the `zephyr-ytm32-module` into your workspace.

## Building Samples

You can compile samples provided in this module, or standard Zephyr samples targeting the YTM32 boards.

To build the `hello_world` sample from this repository:

```bash
# From the zephyrproject root directory
west build -b ytm32b1mc0_evb zephyr-ytm32-module/samples/hello_world
```

If you wish to do a pristine build (clearing previous build artifacts), you can append the `-p` flag:
```bash
west build -p -b ytm32b1mc0_evb zephyr-ytm32-module/samples/hello_world
```

To validate the GPIO compatibility layer with the standard Zephyr `blinky` sample, prefer building the upstream sample directly:

```bash
# From the zephyrproject root directory
west build -b ytm32b1mc0_evb zephyr/samples/basic/blinky
```

The local `zephyr-ytm32-module/samples/blinky` sample is kept aligned with the upstream `blinky` logic, but the upstream sample should be treated as the primary bring-up target.

## Flashing and Debugging

### Console Output
The default console output on `ytm32b1mc0_evb` maps to the onboard UART (usually UART1 over USB). Make sure you have the correct COM port open at the baudrate configured in the board's device tree (typically `115200`).

Alternatively, the project supports routing the print console via **SEGGER RTT** for debugging when UART is unavailable. You can enable this by setting:
```kconfig
CONFIG_USE_SEGGER_RTT=y
CONFIG_RTT_CONSOLE=y
CONFIG_UART_CONSOLE=n
```

### Flashing
You can flash the code using a standard SWD debugger (such as a Segger J-Link). Depending on your setup and `west flash` configuration, run:

```bash
west flash
```
*(Ensure your debugger is properly connected to the board's debug headers.)*

## Repository Structure

- `boards/` - Board configuration and device tree definitions (e.g., `ytm32b1mc0_evb`)
- `soc/` - YTM32 SoC-specific initialization and glue code
- `drivers/` - Out-of-tree drivers specific to YTMicro peripherals (e.g., interrupts)
- `dts/` - Device tree bindings specifically for YTMicro hardware
- `samples/` - Example applications demonstrating capability
- `zephyr/` - Zephyr module configuration (`module.yml`) and Kconfig files

## Contributing
When making changes to the glue code or the port, commit them directly to this repository. The upstream zephyr components remain clean, and any additions specifically needed for YTM32 should be captured here or in `hal_ytmicro`.
