/*
 * Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * YTM32 SPI HAL wrapper — the ONLY translation unit that includes vendor
 * SPI headers. All vendor types (spi_callback_t, spi_state_t, …) are
 * confined here and never leak to the Zephyr driver layer.
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/time_units.h>

/*
 * The vendor HAL defines spi_callback_t with a different signature than
 * Zephyr's spi_callback_t. Rename the vendor typedef for the duration of
 * these includes, then restore the identifier.
 */
#define spi_callback_t hal_spi_callback_t
#include "spi_master_driver.h"
#undef spi_callback_t
#include "spi_hw_access.h"

#include "ytm32_spi_hal.h"

/* ──────────────────────── module-level state ──────────────────────── */

struct spi_inst_ctx {
	spi_state_t     hal_state;
	ytm32_spi_cb_t  cb;
	void           *cb_data;
	uint32_t         txcfg_sync_cycles;
	bool            use_dma;
	uint8_t         dma_rx_ch;
	uint8_t         dma_tx_ch;
};

static struct spi_inst_ctx s_spi[SPI_INSTANCE_COUNT];

/* ──────────────────────── internal helpers ──────────────────────── */

static int status_to_errno(status_t s)
{
	switch (s) {
	case STATUS_SUCCESS: return 0;
	case STATUS_BUSY:    return -EBUSY;
	case STATUS_TIMEOUT: return -ETIMEDOUT;
	default:             return -EIO;
	}
}

/*
 * YTM32B1M erratum E403002: after TXCFG is written, wait for about three SPI
 * functional-clock cycles and read a different SPI register before TXCFG is
 * read again.  The vendor configure functions end by writing TXCFG, while a
 * subsequent transfer can perform TXCFG read-modify-write operations.
 *
 * Keep the workaround in this adapter instead of modifying the imported HAL.
 * SPI_GetStatusFlag() performs the required volatile read from STS.
 */
static void ytm32_spi_hal_sync_txcfg(uint8_t instance)
{
	uint32_t start_cycles = k_cycle_get_32();

	while ((uint32_t)(k_cycle_get_32() - start_cycles) <
	       s_spi[instance].txcfg_sync_cycles) {
		/* The cycle counter bounds this wait independently of compiler timing. */
	}

	(void)SPI_GetStatusFlag(g_spiBase[instance], SPI_MODULE_BUSY);
}

/*
 * Bridge from vendor hal_spi_callback_t to Zephyr ytm32_spi_cb_t.
 * callbackParam carries the instance index so we can look up s_spi[].
 */
static void spi_bridge_cb(void *driver_state, spi_event_t event, void *user_data)
{
	uint8_t instance = (uint8_t)(uintptr_t)user_data;

	(void)driver_state;

	if (event != SPI_EVENT_END_TRANSFER) {
		return;
	}

	/* Query the HAL for the transfer outcome before invoking the user cb. */
	status_t hal_status = SPI_DRV_MasterGetTransferStatus(instance, NULL);
	int status = (hal_status == STATUS_SUCCESS) ? 0 : -EIO;

	if (s_spi[instance].cb) {
		s_spi[instance].cb(s_spi[instance].cb_data, status);
	}
}

/*
 * Build a spi_master_config_t from per-instance and per-configure parameters.
 * Called both from ytm32_spi_hal_init (with default frequency) and from
 * ytm32_spi_hal_configure (with user-requested frequency).
 */
static void fill_hal_config(uint8_t instance,
			    uint32_t freq,
			    uint8_t cpol, uint8_t cpha, uint8_t word_size,
			    uint8_t cs_idx, bool cs_active_hi, bool gpio_cs,
			    spi_master_config_t *out)
{
	SPI_DRV_MasterGetDefaultConfig(out);

	out->bitsPerSec      = freq;
	out->whichPcs        = (spi_which_pcs_t)(cs_idx < 4U ? cs_idx : 0U);
	out->pcsPolarity     = cs_active_hi ? SPI_ACTIVE_HIGH : SPI_ACTIVE_LOW;
	/*
	 * GPIO CS: the application controls the CS line manually.
	 * Keep hardware PCS continuous so the PCS pin does not toggle between
	 * frames and interfere with the software-driven GPIO line.
	 */
	out->isPcsContinuous = gpio_cs;
	out->bitcount        = word_size;
	out->clkPolarity     = cpol ? SPI_SCK_ACTIVE_LOW : SPI_SCK_ACTIVE_HIGH;
	out->clkPhase        = cpha ? SPI_CLOCK_PHASE_2ND_EDGE
				    : SPI_CLOCK_PHASE_1ST_EDGE;
	out->lsbFirst        = false;
	out->width           = SPI_SINGLE_BIT_XFER;

	if (s_spi[instance].use_dma) {
		out->transferType  = SPI_USING_DMA;
		out->rxDMAChannel  = s_spi[instance].dma_rx_ch;
		out->txDMAChannel  = s_spi[instance].dma_tx_ch;
	} else {
		out->transferType  = SPI_USING_INTERRUPTS;
	}

	/* Bridge callback: pass instance index as callbackParam. */
	out->callback      = (hal_spi_callback_t)spi_bridge_cb;
	out->callbackParam = (void *)(uintptr_t)instance;
}

/* ──────────────────────── public API ──────────────────────── */

int ytm32_spi_hal_init(uint8_t instance, uint32_t clock_rate,
		       bool use_dma, uint8_t dma_rx_ch, uint8_t dma_tx_ch,
		       ytm32_spi_cb_t cb, void *user_data)
{
	struct spi_inst_ctx *ctx = &s_spi[instance];
	spi_master_config_t hal_cfg;

	ctx->cb        = cb;
	ctx->cb_data   = user_data;
	ctx->txcfg_sync_cycles = ytm32_spi_hal_txcfg_sync_cycles(
		sys_clock_hw_cycles_per_sec(), clock_rate);
	ctx->use_dma   = use_dma;
	ctx->dma_rx_ch = dma_rx_ch;
	ctx->dma_tx_ch = dma_tx_ch;

	/* Default baud rate: half the functional clock, capped at 1 MHz. */
	uint32_t default_freq = clock_rate / 2U;
	if (default_freq > 1000000U) {
		default_freq = 1000000U;
	}

	fill_hal_config(instance, default_freq,
			0, 0, 8U,   /* cpol=0, cpha=0, word_size=8 */
			0, false,   /* cs_idx=0, cs_active_low */
			false,      /* gpio_cs=false */
			&hal_cfg);

	status_t s = SPI_DRV_MasterInit(instance, &ctx->hal_state, &hal_cfg);

	if (s == STATUS_SUCCESS) {
		ytm32_spi_hal_sync_txcfg(instance);
	}

	return status_to_errno(s);
}

int ytm32_spi_hal_configure(uint8_t instance, uint32_t freq,
			    uint8_t cpol, uint8_t cpha, uint8_t word_size,
			    uint8_t cs_idx, bool cs_active_hi, bool gpio_cs)
{
	spi_master_config_t hal_cfg;
	uint32_t calc_baud;

	fill_hal_config(instance, freq, cpol, cpha, word_size, cs_idx,
			cs_active_hi, gpio_cs, &hal_cfg);

	status_t s = SPI_DRV_MasterConfigureBus(instance, &hal_cfg, &calc_baud);

	if (s == STATUS_SUCCESS) {
		ytm32_spi_hal_sync_txcfg(instance);
	}

	return status_to_errno(s);
}

int ytm32_spi_hal_transfer(uint8_t instance,
			   const uint8_t *tx, uint8_t *rx, uint16_t len)
{
	return status_to_errno(SPI_DRV_MasterTransfer(instance, tx, rx, len));
}

void ytm32_spi_hal_abort(uint8_t instance)
{
	(void)SPI_DRV_MasterAbortTransfer(instance);
}

void ytm32_spi_hal_irq(uint8_t instance)
{
	SPI_DRV_MasterIRQHandler(instance);
}
