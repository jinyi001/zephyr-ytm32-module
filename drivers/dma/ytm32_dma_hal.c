/*
 * Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * YTM32 DMA HAL wrapper — the ONLY translation unit that includes vendor
 * DMA headers. All vendor types (dma_callback_t, dma_state_t, …) are
 * confined here and never leak to the Zephyr driver layer.
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

/*
 * The vendor HAL defines dma_callback_t with a different signature than
 * Zephyr's dma_callback_t. Rename the vendor typedef for the duration of
 * these includes, then restore the identifier.
 */
#define dma_callback_t hal_dma_callback_t
#include "dma_driver.h"
#include "dma_irq.h"
#undef dma_callback_t

#include "ytm32_dma_hal.h"

/* ──────────────────────── module-level state ──────────────────────── */

static dma_state_t    s_dma_state;
static dma_chn_state_t s_chn[FEATURE_DMA_VIRTUAL_CHANNELS];

/* Per-channel Zephyr callback bookkeeping. */
struct chn_ctx {
	ytm32_dma_cb_t cb;
	void          *user_data;
};
static struct chn_ctx s_ctx[FEATURE_DMA_VIRTUAL_CHANNELS];

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
 * Bridge from vendor hal_dma_callback_t to Zephyr ytm32_dma_cb_t.
 * callbackParam carries the channel index so we can look up s_ctx[].
 */
static void dma_bridge_cb(void *param, dma_chn_status_t hal_status)
{
	uint8_t ch = (uint8_t)(uintptr_t)param;
	int status = (hal_status == DMA_CHN_NORMAL) ? 0 : -EIO;

	if (ch < FEATURE_DMA_VIRTUAL_CHANNELS && s_ctx[ch].cb) {
		s_ctx[ch].cb(s_ctx[ch].user_data, status);
	}
}

/* ──────────────────────── public API ──────────────────────── */

int ytm32_dma_hal_init(void)
{
	const dma_user_config_t cfg = {
		.haltOnError = true,
#if defined(FEATURE_DMA_ERRATA_E406002) && (FEATURE_DMA_ERRATA_E406002 == 1)
		.maxChannelForChLink = false,
#endif
	};

	status_t s = DMA_DRV_Init(&s_dma_state, &cfg, NULL, NULL, 0U);
	return status_to_errno(s);
}

int ytm32_dma_hal_channel_init(uint8_t ch, uint8_t trigsrc)
{
	if (ch >= FEATURE_DMA_VIRTUAL_CHANNELS) {
		return -EINVAL;
	}

	/*
	 * No Zephyr-side callback is registered here: callers that use this entry
	 * point (e.g. the vendor SPI master driver) install their own vendor DMA
	 * callback for every transfer.  Keep s_ctx[ch] cleared so that a stray
	 * dma_bridge_cb, should one ever fire for this channel, is a no-op.
	 */
	s_ctx[ch].cb        = NULL;
	s_ctx[ch].user_data = NULL;

	const dma_channel_config_t chan_cfg = {
		.virtChnConfig = ch,
		.source        = (dma_request_source_t)trigsrc,
		.callback      = NULL,
		.callbackParam = NULL,
	};

	return status_to_errno(DMA_DRV_ChannelInit(&s_chn[ch], &chan_cfg));
}

int ytm32_dma_hal_channel_config(uint8_t ch, uint8_t trigsrc,
				 uintptr_t src, uintptr_t dst,
				 uint8_t dir, uint8_t width, uint32_t length,
				 ytm32_dma_cb_t cb, void *user_data)
{
	dma_transfer_type_t type;
	dma_transfer_size_t xfer_size;
	status_t status;

	switch (dir) {
	case YTM32_DMA_DIR_PERIPH2MEM: type = DMA_TRANSFER_PERIPH2MEM; break;
	case YTM32_DMA_DIR_MEM2PERIPH: type = DMA_TRANSFER_MEM2PERIPH;  break;
	case YTM32_DMA_DIR_MEM2MEM:    type = DMA_TRANSFER_MEM2MEM;     break;
	default: return -ENOTSUP;
	}

	switch (width) {
	case 1: xfer_size = DMA_TRANSFER_SIZE_1B; break;
	case 2: xfer_size = DMA_TRANSFER_SIZE_2B; break;
	case 4: xfer_size = DMA_TRANSFER_SIZE_4B; break;
	default: return -ENOTSUP;
	}

	s_ctx[ch].cb        = cb;
	s_ctx[ch].user_data = user_data;

	const dma_channel_config_t chan_cfg = {
		.virtChnConfig  = ch,
		.source         = (dma_request_source_t)trigsrc,
		.callback       = (hal_dma_callback_t)dma_bridge_cb,
		/* Pass channel index so the bridge can look up s_ctx[]. */
		.callbackParam  = (void *)(uintptr_t)ch,
	};

	status = DMA_DRV_ChannelInit(&s_chn[ch], &chan_cfg);
	if (status != STATUS_SUCCESS) {
		return status_to_errno(status);
	}

	status = DMA_DRV_ConfigSingleBlockTransfer(ch, type,
						   (uint32_t)src, (uint32_t)dst,
						   xfer_size, length);
	if (status != STATUS_SUCCESS) {
		DMA_DRV_ReleaseChannel(ch);
		return status_to_errno(status);
	}

	return 0;
}

static int config_loop_transfer(uint8_t ch, uint8_t trigsrc,
					uintptr_t src, uintptr_t dst,
					uint8_t width, uint32_t elements_per_req,
					uint32_t request_count,
					ytm32_dma_cb_t cb, void *user_data,
					bool ram_reload)
{
	dma_transfer_size_t xfer_size;
	status_t status;

	switch (width) {
	case 1: xfer_size = DMA_TRANSFER_SIZE_1B; break;
	case 2: xfer_size = DMA_TRANSFER_SIZE_2B; break;
	case 4: xfer_size = DMA_TRANSFER_SIZE_4B; break;
	default: return -ENOTSUP;
	}

	s_ctx[ch].cb        = cb;
	s_ctx[ch].user_data = user_data;

	const dma_channel_config_t chan_cfg = {
		.virtChnConfig = ch,
		.source        = (dma_request_source_t)trigsrc,
		.callback      = (hal_dma_callback_t)dma_bridge_cb,
		.callbackParam = (void *)(uintptr_t)ch,
	};

	status = DMA_DRV_ChannelInit(&s_chn[ch], &chan_cfg);
	if (status != STATUS_SUCCESS) {
		return status_to_errno(status);
	}

	uint32_t bytes_per_req = elements_per_req * (uint32_t)width;
	uint32_t total_bytes = bytes_per_req * request_count;
	static dma_loop_transfer_config_t loop_cfg[FEATURE_DMA_VIRTUAL_CHANNELS];
	static uint8_t reload_cts_storage[FEATURE_DMA_VIRTUAL_CHANNELS][sizeof(dma_software_cts_t) + 31U];

	loop_cfg[ch] = (dma_loop_transfer_config_t){
		.triggerLoopIterationCount  = request_count,
		.srcOffsetEnable            = false,
		.dstOffsetEnable            = false,
		.triggerLoopOffset          = 0,
		.transferLoopChnLinkEnable  = false,
		.transferLoopChnLinkNumber  = 0U,
		.triggerLoopChnLinkEnable   = false,
		.triggerLoopChnLinkNumber   = 0U,
	};

	dma_software_cts_t *reload_cts =
		(dma_software_cts_t *)SCTS_ADDR(reload_cts_storage[ch]);

	dma_transfer_config_t xfer = {
		.srcAddr               = (uint32_t)src,
		.destAddr              = (uint32_t)dst,
		.srcTransferSize       = xfer_size,
		.destTransferSize      = xfer_size,
		.srcOffset             = 0,
		.destOffset            = (int16_t)width,
		.srcLastAddrAdjust     = 0,
		.destLastAddrAdjust    = -(int32_t)total_bytes,
		.srcModulo             = DMA_MODULO_OFF,
		.destModulo            = DMA_MODULO_OFF,
		.transferLoopByteCount = bytes_per_req,
		.ramReloadEnable       = ram_reload,
		.ramReloadNextDescAddr = ram_reload ? (uint32_t)reload_cts : 0U,
		.interruptEnable       = true,
		.loopTransferConfig    = &loop_cfg[ch],
	};

	if (ram_reload) {
		DMA_DRV_PushConfigToSCTS(&xfer, reload_cts);
	}

	status = DMA_DRV_ConfigLoopTransfer(ch, &xfer);
	if (status != STATUS_SUCCESS) {
		DMA_DRV_ReleaseChannel(ch);
		return status_to_errno(status);
	}

	return 0;
}

int ytm32_dma_hal_channel_config_loop(uint8_t ch, uint8_t trigsrc,
				      uintptr_t src, uintptr_t dst,
				      uint8_t width, uint32_t elements_per_req,
				      uint32_t request_count,
				      ytm32_dma_cb_t cb, void *user_data)
{
	return config_loop_transfer(ch, trigsrc, src, dst, width, elements_per_req,
				    request_count, cb, user_data, false);
}

int ytm32_dma_hal_channel_config_loop_reload(uint8_t ch, uint8_t trigsrc,
					     uintptr_t src, uintptr_t dst,
					     uint8_t width, uint32_t elements_per_req,
					     uint32_t request_count,
					     ytm32_dma_cb_t cb, void *user_data)
{
	return config_loop_transfer(ch, trigsrc, src, dst, width, elements_per_req,
				    request_count, cb, user_data, true);
}

void ytm32_dma_hal_channel_release(uint8_t ch)
{
	DMA_DRV_ReleaseChannel(ch);
	s_ctx[ch].cb        = NULL;
	s_ctx[ch].user_data = NULL;
}

int ytm32_dma_hal_start(uint8_t ch)
{
	return status_to_errno(DMA_DRV_StartChannel(ch));
}

int ytm32_dma_hal_stop(uint8_t ch)
{
	return status_to_errno(DMA_DRV_StopChannel(ch));
}

uint32_t ytm32_dma_hal_remaining(uint8_t ch)
{
	return DMA_DRV_GetRemainingTriggerIterationsCount(ch);
}

bool ytm32_dma_hal_error(uint8_t ch)
{
	return DMA_DRV_GetChannelStatus(ch) == DMA_CHN_ERROR;
}

void ytm32_dma_hal_irq(uint8_t ch)
{
	DMA_DRV_IRQHandler(ch);
}

void ytm32_dma_hal_error_irq(void)
{
#ifdef FEATURE_DMA_HAS_ERROR_IRQ
	DMA_DRV_ErrorIRQHandler(0U);
#endif
}
