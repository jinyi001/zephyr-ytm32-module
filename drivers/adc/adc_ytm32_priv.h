/*
 * Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Private shared interface for the YTM32 ADC driver.
 *
 * The driver is split across two translation units:
 *   - adc_ytm32.c     Phase 1: interrupt-driven, software-triggered ADC API.
 *   - adc_ytm32_dma.c Phase 2: hardware-triggered DMA continuous sampling.
 *
 * This header is NOT a public interface — the public Phase 2 surface is
 * <zephyr/drivers/adc/adc_ytm32.h>.  It only carries what the two TUs must
 * agree on: the per-instance config, and the small slice of mutable state
 * (struct adc_ytm32_shared) that both touch.
 *
 * Note what is deliberately NOT here: the full driver data struct.  That struct
 * embeds a `struct adc_context` by value, which would force every includer to
 * pull in adc_context.h and satisfy its "define the sampling callbacks"
 * contract.  Phase 2 does not drive the adc_context state machine, so it must
 * not see that type.  The core (adc_ytm32.c) owns the full layout and exposes
 * the shared slice through adc_ytm32_shared().
 */

#ifndef ZEPHYR_DRIVERS_ADC_ADC_YTM32_PRIV_H_
#define ZEPHYR_DRIVERS_ADC_ADC_YTM32_PRIV_H_

#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/adc/adc_ytm32.h>

#include "device_registers.h"
#include "adc_driver.h"

/* Maximum external channel count across all YTM32 variants */
#define YTM32_ADC_MAX_CHANS 38U

/* SMP=2 gives t_sample=125 ns at 32 MHz and 1 us at 4 MHz. */
#define YTM32_ADC_DEFAULT_SAMPLE_TIME 2U

/* Sentinel meaning "no hardware trigger configured" */
#define YTM32_ADC_NO_HW_TRIG UINT32_MAX

/* ──────────────────────── per-instance config ──────────────────────── */

struct adc_ytm32_config {
	uint32_t                         base;
	uint8_t                          instance;
	const struct device             *clock_dev;
	clock_control_subsys_t           clock_subsys;
	uint32_t                         adc_clk_div; /* CFG1.PRS — ADC internal divider */
	uint32_t                         adc_start_time; /* CFG1.STCNT — ADC startup ticks */
	uint8_t                          sequence_order[ADC_CHSEL_COUNT];
	uint8_t                          sequence_order_count;
	const struct pinctrl_dev_config *pincfg;
	void (*irq_config_func)(void);
	IRQn_Type                         irq;
	/* Phase 2: DMA and hardware trigger (optional) */
	const struct device             *dma_dev;     /* NULL if DMA not wired */
	uint8_t                          dma_channel;
	uint8_t                          dma_slot;    /* DMAMUX request source */
	uint32_t                         hw_trig_src; /* TMU trigger source enum value */
	const struct device             *tmu_dev;    /* NULL if no TMU phandle */
	uint32_t                         cim_trig_sel; /* CIM ADC0_TRIG_SEL field value */
};

/* ──────────────────── state shared by both TUs ──────────────────── */

/*
 * The mutable conversion state that both the Phase 1 core and the Phase 2 DMA
 * extension read/write.  Kept separate from the adc_context-bearing driver data
 * so the DMA TU stays free of adc_context.h (see file header).  Embedded by
 * value inside the full driver data struct in adc_ytm32.c.
 */
struct adc_ytm32_shared {
	uint8_t channel_count;
	uint8_t sample_time[YTM32_ADC_MAX_CHANS];
	uint32_t adc_clock_hz;
	/* Phase 2 */
	bool    dma_active;
	int     dma_error;
	struct adc_ytm32_dma_config dma_cfg; /* copy of user config */
};

/* Returns the shared slice of this instance's driver data.
 * Implemented in adc_ytm32.c, which owns the full data layout.
 */
struct adc_ytm32_shared *adc_ytm32_shared(const struct device *dev);

/* ──────────────────────── shared helpers (adc_ytm32.c) ──────────────────── */

adc_resolution_t adc_ytm32_bits_to_resolution(uint8_t bits);

uint8_t adc_ytm32_channels_to_sequence(uint32_t channels_mask,
					adc_inputchannel_t *chsel,
					uint8_t *sample_times,
					uint8_t *max_smp_out);

int adc_ytm32_sequence_from_config(const struct adc_ytm32_config *config,
					   uint32_t channels_mask,
					   adc_inputchannel_t *chsel,
					   const uint8_t *sample_times,
					   uint8_t *channel_count_out,
					   uint8_t *max_smp_out);

/* ─────────────── Phase 2 half called from the Phase 1 ISR ───────────────── */

/* Defined in adc_ytm32_dma.c.  The shared NVIC line is software-pended by the
 * DMA top-half and handled here from regular ISR context (see adc_ytm32_isr).
 */
void adc_ytm32_dma_notify_isr(const struct device *dev);

#endif /* ZEPHYR_DRIVERS_ADC_ADC_YTM32_PRIV_H_ */
