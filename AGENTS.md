# AGENTS.md — driver authoring conventions for zephyr-ytm32-module

This file is the in-repo source of truth for **how YTM32 Zephyr glue drivers are
written**. It exists because the same conventions kept living only in a
maintainer's head or in an AI assistant's private memory, where neither a human
reviewer nor the next contributor could see them. If you add or refactor a
driver, follow this file; if you intentionally deviate, say why in the commit.

Companion docs:
- Boot flow + who-owns-which-peripheral: `BRINGUP_RESOURCE_OWNERSHIP.md` (superproject root).
- User-facing overview: `README.md` / `README_zh.md`.

---

## 1. Register access — use the vendor HAL, never re-`#define` it

Do **not** hand-write `#define`s for register offsets, bit masks/shifts, or
peripheral layouts. The vendor HAL device headers already define them, and
duplicates drift out of sync with the silicon.

Rules:
- `#include "device_registers.h"` — the umbrella header. It auto-selects the
  correct device header (`YTM32B1MD1.h` / `YTM32B1MC0.h`) from the build-defined
  `CPU_YTM32B1MD1` / `CPU_YTM32B1MC0` macro. **Never** hard-code `<YTM32B1MD1.h>`
  — that breaks the moment a second MCU shares the driver.
- Use the HAL peripheral struct types (`ADC_Type`, `WDG_Type`, `lpTMR_Type`, …)
  and `<PERIPH>_<FIELD>_MASK` / `_SHIFT` macros. No literals like `0x4C` or
  `BIT(0)` for register fields.
- The register **base** comes from devicetree (`DT_INST_REG_ADDR`), so the field
  macros (layout) come from HAL and the address (instance) comes from DT.
- **grep case-insensitively** before concluding the HAL "doesn't model" a
  peripheral — vendor typedefs are mixed-case (`lpTMR_Type`, not `LPTMR_Type`;
  `eTMR_Type`). When unsure, read the factory
  `…/platform/drivers/src/<periph>/*_hw_access.c` for the exact struct/macro names.
- Uppercase **protocol constants** that are *not* register layout (e.g. watchdog
  unlock/service codes `WDG_UNLOCK_VALUE_*`, timeout limits) are fine to keep as
  local `#define`s — they are not part of the silicon register map.

### HAL struct mapping (direct register-struct drivers)

| Driver | HAL struct(s) |
|---|---|
| `adc/adc_ytm32.c` | `ADC_Type` (+ HAL `ADC_DRV_*` for the errata path) |
| `gpio/gpio_ytm32.c` | `GPIO_Type`, `PCTRL_Type` |
| `serial/uart_ytm32.c` | `UART_Type` |
| `watchdog/wdt_ytm32.c` | `WDG_Type` |
| `counter/counter_ytm32_lptmr.c` | `lpTMR_Type` |
| `can/ytm32_can_hal.c` | `CAN_Type` |
| `pwm/pwm_ytm32_etmr.c` | `eTMR` / `SCU` singleton pointers (via `etmr_common.h`) |

Other drivers (clock_control, dma, spi, flash, tmu, intc, pinctrl) reach the
silicon through the vendor HAL **driver** API rather than raw struct pokes; the
same "don't redefine, include `device_registers.h`" rule still applies to any
register they touch directly.

---

## 2. Function naming — infix `<api>_ytm32[_<variant>]_<verb>`

Every function in a driver file shares **one** prefix derived from the file
name, matching upstream Zephyr style (`uart_ns16550_*`). This includes internal
`static` helpers, not just the public API — one prefix per file is what makes
the code greppable and predictable for both humans and AI.

| File | Prefix |
|---|---|
| `adc/adc_ytm32.c` | `adc_ytm32_` |
| `watchdog/wdt_ytm32.c` | `wdt_ytm32_` |
| `tmu/tmu_ytm32.c` | `tmu_ytm32_` |
| `counter/counter_ytm32_lptmr.c` | `counter_ytm32_lptmr_` (keep the variant token; a future `counter_ytm32_pit.c` must not collide) |
| `clock_control/clock_control_ytm32.c` | `clock_control_ytm32_` |

**The one exception:** the optional HAL-wrapper layer (see §3) lives in
`ytm32_<periph>_hal.c` and uses the prefix `ytm32_<periph>_hal_*`. That prefix is
intentional — it is *not* Zephyr API, it is a vendor-HAL adapter, and the name
mirrors the file.

Uppercase macro constants are not function names; the §1 rule governs them.

---

## 3. File layout — single file by default, split only when complex

Default: one file `<api>_ytm32[_<variant>].c` containing both the register work
and the Zephyr glue.

There are two sanctioned ways to split, for two different reasons. Both are
exceptions — do not reach for them on a simple driver, where a two-file skeleton
for a 200-line driver is pure overhead.

**(a) HAL-wrapper layer — `ytm32_<periph>_hal.c`.** Split this out **only** when
the driver is genuinely complex *and* the HAL-wrapping is reused across
modes/paths (roughly: large drivers where the register/HAL detail would
otherwise drown the glue). Today only **can**, **dma**, and **spi** meet that
bar. The wrapper keeps the `ytm32_<periph>_hal_*` prefix (the §2 exception).

**(b) Optional-feature extension — `<api>_ytm32_<feature>.c`.** Split this out
when a driver carries a **self-contained optional feature** that is large,
behind its own public header, and roughly independent of the core data path —
so a reader of the standard driver need not wade through it, and vice versa.
The two TUs share their per-instance config/data through a private
`<api>_ytm32_priv.h` (structs + shared constants + the few cross-TU prototypes;
*not* a public API). All functions keep the file's normal §2 infix prefix.
Today only **adc** meets this bar: `adc_ytm32.c` (Phase 1, standard
interrupt-driven API) + `adc_ytm32_dma.c` (Phase 2, the hardware-triggered DMA
extension behind `<zephyr/drivers/adc/adc_ytm32.h>`) + `adc_ytm32_priv.h`.

When in doubt, stay single-file. Line count alone is **not** a reason to split:
`can_ytm32.c` is ~950 lines but is a flat list of mandatory Zephyr CAN callbacks
with the register layer already in `ytm32_can_hal.c` — splitting it further
would only scatter the one-prefix-per-file greppability §2 exists to create.

---

## 4. Single-instance drivers

`counter`, `spi`, `serial`, `watchdog` carry an `INSTANCE_VALID` `BUILD_ASSERT`
that pins the register address to instance 0 (`WDG0_BASE`, `lpTMR0_BASE`, …).
This is intentional: **the silicon has exactly one of these peripherals.** The
base is still taken from devicetree for layout consistency; the assert just
fails the build loudly if a second instance is ever added without revisiting the
driver. It is a guard, not an oversight — don't "fix" it into fake multi-instance
support without real multi-instance hardware.

---

## 5. Cross-MCU (MD1 vs MC0) portability

Drivers are expected to compile against **both** YTM32B1MD1 and YTM32B1MC0.
Where the silicon diverges, guard with `#if defined(CPU_YTM32B1MD1)`:
- `SCU->PLL_CTRL` and `SCU_PLL_CTRL_*` are **MD1-only** (MC0 has no SCU PLL).
- `eTMR_OTRIG_INITTEN_MASK` is **MD1-only**.

**Known debt:** only `pwm_ytm32_etmr.c` has been systematically audited for MD1/MC0
divergence. Other drivers currently compile for MD1 only because they happen not
to touch divergent registers — a full MC0 build sweep is pending. If you build
for MC0 and hit a missing symbol, this is why.

---

## 6. Silicon errata / quirk index

Quirks live next to the code that works around them; this table is the index so
they are findable. Update it when you add a workaround.

| ID | Where | What |
|---|---|---|
| ADC E600001 (vendor `ADC_ERRATA_E0002`) | `adc/adc_ytm32.c` top-of-file | First channel of an idle-then-triggered sequence reads inaccurately. Mitigated by HAL `ADC_Enable()` IPC-reset, gated by the `ADC_ERRATA_E0002` macro; the driver `#error`s if a HAL bump drops the macro. |
| ADC MD1 ADSTART stale gate | `adc/adc_ytm32.c` `adc_ytm32_arm_hw_trigger()` | HW-trigger accept gate not reliably armed after enable; cleared via `ADSTOP`→`ADSTART` with a bare counter loop (`k_busy_wait` disturbs the timing). |
| DMA E406002 | `dma/ytm32_dma_hal.c` | `maxChannelForChLink` config gated by `FEATURE_DMA_ERRATA_E406002`. |
| SPI E403002 | `spi/ytm32_spi_hal.c` `ytm32_spi_hal_sync_txcfg()` | A `TXCFG` write needs about three SPI functional-clock cycles followed by a read of another register before `TXCFG` is read again. The adapter waits in the system cycle-counter domain and reads `STS` after successful vendor init/configure calls. |

---

## 7. Do not touch — upstream / vendor code is pristine

- **Vendor HAL SDK** (`workspace/modules/hal/ytmicro/…`) is an imported vendor
  drop. Keep it bit-for-bit pristine. Do compatibility/shim work in *our*
  wrapper layer, never by editing imported files. (`ai/check_zephyr_pristine.sh`
  guards this.)
- **Prefer existing factory implementations.** Before writing a register
  sequence, check the factory `platform/drivers/src/<periph>/` for an existing
  one — reuse it instead of reinventing.
- Imported upstream Zephyr files stay pristine for the same reason; local
  compatibility goes in wrappers.
