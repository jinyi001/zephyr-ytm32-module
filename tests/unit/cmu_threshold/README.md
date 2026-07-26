# YTM32 CMU threshold calculation test

This is a host-only Tier 0 test for the frequency-to-CMU-register conversion.
Run it from the `zephyr-ytm32` repository with:

```sh
cc -std=c11 -Wall -Wextra -Werror -Wconversion -Wshadow -pedantic \
  -I soc/ytmicro/ytm32/ytm32b1m \
  tests/unit/cmu_threshold/test_cmu_threshold.c \
  -o /tmp/ytm32-cmu-threshold-test && /tmp/ytm32-cmu-threshold-test
```

The calculation follows the local SDK and HAL facts:

- `YTM32B1MD1/.../clock_YTM32B1Mx.c` computes the default CMU window as
  `channel_hz * 128 * 5 / (4 * reference_hz)` and
  `channel_hz * 128 * 3 / (4 * reference_hz)`.
- `clock_YTM32B1Mx.h` defines 128 reference cycles and `CMU_REF_SIRC_CLOCK`,
  and its channel fields are 16-bit compare values.
- `YTM32B1MD1_features.h` defines FIRC=96 MHz and SIRC=12 MHz in the normal
  build; the Zephyr MD1 clock binding defines the same FIRC/FXOSC values.
- The SDK PLL frequency is `(reference_hz / refdiv) * fbdiv / 2`, with the
  reference division performed first, matching `CLOCK_DRV_GetPllFreq()`.

The implementation uses Hz inputs, preserves the SDK's MHz truncation and
integer-division order, and uses 64-bit intermediates.  The MC0 case in the SDK defines
SIRC=2 MHz; the current MC0 Zephyr binding still says 12 MHz, so the production
CMU path follows the SDK definition actually consumed by the HAL.  This test
does not modify that binding.
