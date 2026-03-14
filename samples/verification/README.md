# YTM32 Compatibility-Layer Verification Samples

This directory keeps the board-verification apps inside the compatibility layer,
so you do not need to jump around the upstream Zephyr sample tree.

Target board:

- `ytm32b1mc0_evb`

Workspace root:

- `C:\Users\JinYi\zephyrproject`

Required `west`:

- `C:\Users\JinYi\zephyrproject\.venv\Scripts\python.exe -m west`

## Sample Set

Use these apps in the recommended order:

1. `zephyr-ytm32-module/samples/hello_world`
2. `zephyr-ytm32-module/samples/blinky`
3. `zephyr-ytm32-module/samples/verification/uart_poll_smoke`
4. `zephyr-ytm32-module/samples/verification/gpio_loopback`
5. `zephyr-ytm32-module/samples/verification/counter_alarm_smoke`
6. `zephyr-ytm32-module/samples/verification/uart_irq_scope`

## Build Commands

### 1. hello_world

```powershell
& 'C:\Users\JinYi\zephyrproject\.venv\Scripts\python.exe' -m west build -b ytm32b1mc0_evb zephyr-ytm32-module/samples/hello_world -d build-board-hello -p always
```

### 2. blinky

```powershell
& 'C:\Users\JinYi\zephyrproject\.venv\Scripts\python.exe' -m west build -b ytm32b1mc0_evb zephyr-ytm32-module/samples/blinky -d build-board-blinky -p always
```

### 3. uart_poll_smoke

```powershell
& 'C:\Users\JinYi\zephyrproject\.venv\Scripts\python.exe' -m west build -b ytm32b1mc0_evb zephyr-ytm32-module/samples/verification/uart_poll_smoke -d build-board-uart-poll -p always
```

### 4. gpio_loopback

Wire `PTD2` to `PTD3` before flashing.

```powershell
& 'C:\Users\JinYi\zephyrproject\.venv\Scripts\python.exe' -m west build -b ytm32b1mc0_evb zephyr-ytm32-module/samples/verification/gpio_loopback -d build-board-gpio-loopback -p always
```

### 5. counter_alarm_smoke

```powershell
& 'C:\Users\JinYi\zephyrproject\.venv\Scripts\python.exe' -m west build -b ytm32b1mc0_evb zephyr-ytm32-module/samples/verification/counter_alarm_smoke -d build-board-counter-alarm -p always
```

### 6. uart_irq_scope

```powershell
& 'C:\Users\JinYi\zephyrproject\.venv\Scripts\python.exe' -m west build -b ytm32b1mc0_evb zephyr-ytm32-module/samples/verification/uart_irq_scope -d build-board-uart-irq-scope -p always
```

## Flash Command Pattern

After each build, flash with the matching build directory:

```powershell
& 'C:\Users\JinYi\zephyrproject\.venv\Scripts\python.exe' -m west flash -d <build-dir>
```

## What Each App Verifies

- `hello_world`: boot path, early init, console TX
- `blinky`: GPIO output, pinctrl, board LED alias
- `uart_poll_smoke`: UART polling TX and RX on real hardware
- `gpio_loopback`: GPIO output/input/edge interrupt using a jumper wire
- `counter_alarm_smoke`: `lptmr0` counter start plus alarm callback path
- `uart_irq_scope`: confirms interrupt-driven UART is still outside the MVP scope
