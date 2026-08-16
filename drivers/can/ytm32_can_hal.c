/*
 * Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * YTM32 FlexCAN HAL wrapper.
 * This is the ONLY translation unit that includes vendor FlexCAN headers.
 * All vendor types are confined here and never leak to the Zephyr driver.
 */

/*
 * Rename vendor can_callback_t to prevent collision with Zephyr's typedef.
 */
#define can_callback_t hal_can_callback_t
#include "flexcan_driver.h"
#include "flexcan_hw_access.h"
#include "flexcan_irq.h"
#undef can_callback_t

#include "YTM32B1MD1.h"

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "ytm32_can_hal.h"

/* ──────────────────────── module state ──────────────────────── */

struct can_inst_ctx {
	flexcan_state_t     state;
	ytm32_can_evt_cb_t  cb;
	void               *cb_data;
	uint8_t             rx_base;
	uint8_t             tx_base;
	uint8_t             rx_count;
	uint8_t             tx_count;
	bool                fd_enabled;
	/* HAL-owned RX receive buffers. */
	flexcan_msgbuff_t   rx_bufs[YTM32_CAN_HAL_MAX_RX_MB];
	/* Decoded frames exposed via ytm32_can_hal_rx_frame(). */
	struct ytm32_can_frame rx_frames[YTM32_CAN_HAL_MAX_RX_MB];
};

static struct can_inst_ctx s_can[CAN_INSTANCE_COUNT];

/* Base-pointer array indexed by instance. */
static CAN_Type *const s_bases[CAN_INSTANCE_COUNT] = CAN_BASE_PTRS;

/* ──────────────────────── helpers ──────────────────────── */

static int sdk_to_errno(status_t s)
{
	switch (s) {
	case STATUS_SUCCESS:                     return 0;
	case STATUS_BUSY:                        return -EBUSY;
	case STATUS_TIMEOUT:                     return -ETIMEDOUT;
	case STATUS_CAN_BUFF_OUT_OF_RANGE:       return -EINVAL;
	case STATUS_CAN_NO_TRANSFER_IN_PROGRESS: return -EALREADY;
	default:                                 return -EIO;
	}
}

/* Convert raw CS field to Classic CAN DLC byte count (0..8). */
static uint8_t cs_to_dlc_bytes(uint32_t cs)
{
	uint8_t dlc = (uint8_t)((cs >> 16U) & 0x0FU);

	return (dlc <= 8U) ? dlc : 8U;
}

static void decode_rx_buf(const flexcan_msgbuff_t *src,
			  struct ytm32_can_frame *dst,
			  bool fd_enabled)
{
	/* CS bit 22 = IDE, bit 20 = RTR, bit 21 = EDL (FD format) */
	bool ide = (src->cs & (1UL << 22)) != 0U;
	bool rtr = (src->cs & (1UL << 20)) != 0U;
	bool edl = fd_enabled && ((src->cs & (1UL << 21)) != 0U);
	bool brs = fd_enabled && ((src->cs & (1UL << 30)) != 0U);

	dst->ide = ide;
	dst->rtr = rtr;
	dst->fd  = edl;
	dst->brs = brs;
	dst->id  = src->msgId & (ide ? 0x1FFFFFFFU : 0x7FFU);

	if (edl) {
		/* FD frame: dataLen field from flexcan_msgbuff_t holds byte count */
		dst->dlc = (src->dataLen <= YTM32_CAN_HAL_FD_DLEN)
			   ? src->dataLen : (uint8_t)YTM32_CAN_HAL_FD_DLEN;
	} else {
		dst->dlc = cs_to_dlc_bytes(src->cs);
	}
	memcpy(dst->data, src->data,
	       (dst->dlc <= sizeof(dst->data)) ? dst->dlc : sizeof(dst->data));
}

/* ──────────────────────── SDK event callbacks ──────────────────────── */

static void can_sdk_event_cb(uint8_t instance, flexcan_event_type_t event,
			     uint32_t mb_idx, flexcan_state_t *state)
{
	(void)state;
	struct can_inst_ctx *ctx = &s_can[instance];

	if (!ctx->cb) {
		return;
	}

	if (event == FLEXCAN_EVENT_TX_COMPLETE) {
		if (mb_idx >= ctx->tx_base &&
		    mb_idx < (uint32_t)(ctx->tx_base + ctx->tx_count)) {
			uint8_t slot = (uint8_t)(mb_idx - ctx->tx_base);

			ctx->cb(ctx->cb_data, YTM32_CAN_EVENT_TX_DONE, slot, 0);
		}
		return;
	}

	if (event == FLEXCAN_EVENT_RX_COMPLETE) {
		if (mb_idx >= ctx->rx_base &&
		    mb_idx < (uint32_t)(ctx->rx_base + ctx->rx_count)) {
			uint8_t slot = (uint8_t)(mb_idx - ctx->rx_base);

			decode_rx_buf(&ctx->rx_bufs[slot],
				      &ctx->rx_frames[slot],
				      ctx->fd_enabled);
			ctx->cb(ctx->cb_data, YTM32_CAN_EVENT_RX_DONE, slot, 0);
		}
		return;
	}

	/* RX FIFO events — not used in MB mode, ignore. */
#if FEATURE_CAN_HAS_WAKE_UP_IRQ
	if (event == FLEXCAN_EVENT_SELF_WAKEUP) {
		ctx->cb(ctx->cb_data, YTM32_CAN_EVENT_WAKEUP, 0xFFU, 0);
	}
#endif
}

static void can_sdk_error_cb(uint8_t instance,
			     flexcan_error_event_type_t event,
			     flexcan_state_t *state)
{
	(void)state;
	struct can_inst_ctx *ctx = &s_can[instance];

	if (!ctx->cb) {
		return;
	}

	switch (event) {
	case FLEXCAN_BUS_OFF_ENTER_EVENT:
		ctx->cb(ctx->cb_data, YTM32_CAN_EVENT_BUS_OFF, 0xFFU, -EIO);
		break;
	case FLEXCAN_BUS_OFF_DONE_EVENT:
		ctx->cb(ctx->cb_data, YTM32_CAN_EVENT_BUS_OFF_DONE, 0xFFU, 0);
		break;
	default:
		ctx->cb(ctx->cb_data, YTM32_CAN_EVENT_ERROR, 0xFFU, -EIO);
		break;
	}
}

/* ──────────────────────── public API ──────────────────────── */

int ytm32_can_hal_init(uint8_t instance, uint32_t clock_hz,
		       uint8_t max_mb, uint8_t rx_count, uint8_t tx_count,
		       const struct ytm32_can_timing *timing,
		       bool fd_enable,
		       ytm32_can_evt_cb_t cb, void *user_data)
{
	if (instance >= CAN_INSTANCE_COUNT ||
	    (uint32_t)(rx_count + tx_count) > max_mb ||
	    rx_count > YTM32_CAN_HAL_MAX_RX_MB ||
	    tx_count > YTM32_CAN_HAL_MAX_TX_MB) {
		return -EINVAL;
	}

	struct can_inst_ctx *ctx = &s_can[instance];

	ctx->cb         = cb;
	ctx->cb_data    = user_data;
	ctx->rx_count   = rx_count;
	ctx->tx_count   = tx_count;
	ctx->rx_base    = 0U;
	ctx->tx_base    = rx_count;
	ctx->fd_enabled = fd_enable;

	(void)clock_hz; /* SDK reads clock from CLOCK_SYS_GetFreq internally */

	flexcan_user_config_t cfg;
	(void)FLEXCAN_DRV_GetDefaultConfig(&cfg);

	cfg.max_num_mb        = max_mb;
	cfg.is_rx_fifo_needed = false;
	cfg.flexcanMode       = FLEXCAN_FREEZE_MODE;
	cfg.transfer_type     = FLEXCAN_RXFIFO_USING_INTERRUPTS;

	/* The vendor SDK timing fields are register values: physical - 1. */
	cfg.bitrate.propSeg    = timing->prop_seg - 1U;
	cfg.bitrate.phaseSeg1  = timing->phase_seg1 - 1U;
	cfg.bitrate.phaseSeg2  = timing->phase_seg2 - 1U;
	cfg.bitrate.preDivider = timing->prescaler - 1U;
	cfg.bitrate.rJumpwidth = timing->sjw - 1U;

#if FEATURE_CAN_HAS_PE_CLKSRC_SELECT
	/*
	 * The CAN timing passed by the Zephyr driver is calculated from the
	 * functional clock returned for the CAN peripheral.  On MD1 the vendor
	 * FlexCAN default path uses the oscillator as the protocol-engine clock;
	 * selecting the peripheral clock here makes the register timing and the
	 * physical bit rate disagree when the peripheral clock tree differs from
	 * FXOSC.  Keep the PE source aligned with the timing calculation and the
	 * board's 16 MHz oscillator.
	 */
	cfg.pe_clock = FLEXCAN_CLK_SOURCE_OSC;
#endif

#if FEATURE_CAN_HAS_FD
	cfg.fd_enable = fd_enable;
	cfg.payload   = fd_enable ? FLEXCAN_PAYLOAD_SIZE_64
				  : FLEXCAN_PAYLOAD_SIZE_8;
	/* FD data-phase bitrate defaults to same as arbitration. */
	cfg.bitrate_cbt = cfg.bitrate;
#endif

	status_t s = FLEXCAN_DRV_Init(instance, &ctx->state, &cfg);

	if (s != STATUS_SUCCESS) {
		return sdk_to_errno(s);
	}

	FLEXCAN_DRV_SetRxMaskType(instance, FLEXCAN_RX_MASK_INDIVIDUAL);

	/* Enable auto bus-off recovery by default (BOFFREC=0 means auto). */
	s_bases[instance]->CTRL1 &= ~CAN_CTRL1_BOFFREC_MASK;

	FLEXCAN_DRV_InstallEventCallback(instance, can_sdk_event_cb, NULL);
	FLEXCAN_DRV_InstallErrorCallback(instance, can_sdk_error_cb, NULL);

	return 0;
}

int ytm32_can_hal_set_timing(uint8_t instance,
			     const struct ytm32_can_timing *timing)
{
	/* The vendor SDK fields are register encodings: physical value - 1. */
	flexcan_time_segment_t t = {
		.propSeg    = timing->prop_seg - 1U,
		.phaseSeg1  = timing->phase_seg1 - 1U,
		.phaseSeg2  = timing->phase_seg2 - 1U,
		.preDivider = timing->prescaler - 1U,
		.rJumpwidth = timing->sjw - 1U,
	};

	FLEXCAN_DRV_SetBitrate(instance, &t);
	return 0;
}

int ytm32_can_hal_set_timing_fd(uint8_t instance,
				const struct ytm32_can_timing *timing_data)
{
#if FEATURE_CAN_HAS_FD
	flexcan_time_segment_t t = {
		.propSeg    = timing_data->prop_seg,
		.phaseSeg1  = timing_data->phase_seg1,
		.phaseSeg2  = timing_data->phase_seg2,
		.preDivider = timing_data->prescaler - 1U,
		.rJumpwidth = timing_data->sjw - 1U,
	};

	FLEXCAN_DRV_SetBitrateCbt(instance, &t);
	return 0;
#else
	(void)instance;
	(void)timing_data;
	return -ENOTSUP;
#endif
}

int ytm32_can_hal_set_mode(uint8_t instance, ytm32_can_mode_t mode)
{
	flexcan_operation_modes_t sdk_mode;

	switch (mode) {
	case YTM32_CAN_MODE_NORMAL:
		sdk_mode = FLEXCAN_NORMAL_MODE;
		break;
	case YTM32_CAN_MODE_LOOPBACK:
		sdk_mode = FLEXCAN_LOOPBACK_MODE;
		break;
	case YTM32_CAN_MODE_LISTENONLY:
		sdk_mode = FLEXCAN_LISTEN_ONLY_MODE;
		break;
	case YTM32_CAN_MODE_FREEZE:
		sdk_mode = FLEXCAN_FREEZE_MODE;
		break;
	default:
		return -EINVAL;
	}

	CAN_Type *base = s_bases[instance];

	FLEXCAN_SetOperationMode(base, sdk_mode);
	/* SetOperationMode(NORMAL/LISTEN/LOOPBACK) does not leave freeze. */
	if (mode != YTM32_CAN_MODE_FREEZE) {
		FLEXCAN_ExitFreezeMode(base);
	}
	return 0;
}

int ytm32_can_hal_config_rx(uint8_t instance, uint8_t rx_slot,
			    uint32_t id, uint32_t mask,
			    bool ide, bool rtr, bool fd)
{
	struct can_inst_ctx *ctx = &s_can[instance];

	if (rx_slot >= ctx->rx_count) {
		return -EINVAL;
	}

	uint8_t mb = ctx->rx_base + rx_slot;
	flexcan_msgbuff_id_type_t id_type =
		ide ? FLEXCAN_MSG_ID_EXT : FLEXCAN_MSG_ID_STD;

	status_t s = FLEXCAN_DRV_SetRxIndividualMask(instance, id_type,
						     mb, mask);
	if (s != STATUS_SUCCESS) {
		return sdk_to_errno(s);
	}

	flexcan_data_info_t info = {
		.msg_id_type = id_type,
		.data_length = 8U,
		.is_remote   = rtr,
#if FEATURE_CAN_HAS_FD
		.fd_enable   = fd && ctx->fd_enabled,
		.enable_brs  = false,
		.fd_padding  = 0U,
#endif
	};
	(void)fd;

	s = FLEXCAN_DRV_ConfigRxMb(instance, mb, &info, id);
	return sdk_to_errno(s);
}

int ytm32_can_hal_rx_arm(uint8_t instance, uint8_t rx_slot)
{
	struct can_inst_ctx *ctx = &s_can[instance];

	if (rx_slot >= ctx->rx_count) {
		return -EINVAL;
	}

	uint8_t mb = ctx->rx_base + rx_slot;
	status_t s = FLEXCAN_DRV_Receive(instance, mb,
					 &ctx->rx_bufs[rx_slot]);

	return sdk_to_errno(s);
}

int ytm32_can_hal_send(uint8_t instance, uint8_t tx_slot,
		       const struct ytm32_can_frame *frame)
{
	struct can_inst_ctx *ctx = &s_can[instance];

	if (tx_slot >= ctx->tx_count) {
		return -EINVAL;
	}

	uint8_t mb = ctx->tx_base + tx_slot;
	bool use_fd = frame->fd && ctx->fd_enabled;

	flexcan_data_info_t info = {
		.msg_id_type = frame->ide ? FLEXCAN_MSG_ID_EXT
					  : FLEXCAN_MSG_ID_STD,
		.data_length = frame->dlc,
		.is_remote   = frame->rtr,
#if FEATURE_CAN_HAS_FD
		.fd_enable   = use_fd,
		.enable_brs  = use_fd && frame->brs,
		.fd_padding  = 0U,
#endif
	};

	status_t s = FLEXCAN_DRV_ConfigTxMb(instance, mb, &info, frame->id);
	if (s != STATUS_SUCCESS) {
		return sdk_to_errno(s);
	}

	s = FLEXCAN_DRV_Send(instance, mb, &info, frame->id, frame->data);
	return sdk_to_errno(s);
}

int ytm32_can_hal_abort_tx(uint8_t instance, uint8_t tx_slot)
{
	struct can_inst_ctx *ctx = &s_can[instance];

	if (tx_slot >= ctx->tx_count) {
		return -EINVAL;
	}

	status_t s = FLEXCAN_DRV_AbortTransfer(instance,
					       ctx->tx_base + tx_slot);

	return sdk_to_errno(s);
}

const struct ytm32_can_frame *ytm32_can_hal_rx_frame(uint8_t instance,
						     uint8_t rx_slot)
{
	if (rx_slot >= s_can[instance].rx_count) {
		return NULL;
	}

	return &s_can[instance].rx_frames[rx_slot];
}

uint32_t ytm32_can_hal_get_error_status(uint8_t instance)
{
	return FLEXCAN_DRV_GetErrorStatus(instance);
}

void ytm32_can_hal_get_err_cnt(uint8_t instance,
			       uint8_t *tx_err, uint8_t *rx_err)
{
	uint32_t ecr = s_bases[instance]->ECR;

	*tx_err = (uint8_t)(ecr & CAN_ECR_TXERRCNT_MASK);
	*rx_err = (uint8_t)((ecr & CAN_ECR_RXERRCNT_MASK) >>
			    CAN_ECR_RXERRCNT_SHIFT);
}

void ytm32_can_hal_recover(uint8_t instance)
{
	CAN_Type *base = s_bases[instance];

	/* Bus-off recovery: set BOFFREC then clear it — triggers hardware
	 * auto-recovery countdown (128 × 11 recessive bits). */
	FLEXCAN_EnterFreezeMode(base);
	base->CTRL1 |=  CAN_CTRL1_BOFFREC_MASK;
	base->CTRL1 &= ~CAN_CTRL1_BOFFREC_MASK;
	FLEXCAN_ExitFreezeMode(base);
}

void ytm32_can_hal_deinit(uint8_t instance)
{
	(void)FLEXCAN_DRV_Deinit(instance);
}

/*
 * IRQ group → SDK handler dispatch.
 * Group encoding matches interrupt-names in dtsi:
 *   0=ored  1=error  2=wakeup  3=mb_0_15  4=mb_16_31  5=mb_32_47  6=mb_48_63
 */
void ytm32_can_hal_irq(uint8_t instance, uint8_t group)
{
	switch (group) {
	case 0:
		FLEXCAN_ORed_IRQHandler(instance);
		break;
	case 1:
		FLEXCAN_Error_IRQHandler(instance);
		break;
	case 2:
#if FEATURE_CAN_HAS_WAKE_UP_IRQ
		FLEXCAN_WakeUpHandler(instance);
#endif
		break;
	case 3:
		FLEXCAN_IRQHandler(instance, 0U, 15U);
		break;
	case 4:
		FLEXCAN_IRQHandler(instance, 16U, 31U);
		break;
	case 5:
		FLEXCAN_IRQHandler(instance, 32U, 47U);
		break;
	case 6:
		FLEXCAN_IRQHandler(instance, 48U, 63U);
		break;
	default:
		break;
	}
}
