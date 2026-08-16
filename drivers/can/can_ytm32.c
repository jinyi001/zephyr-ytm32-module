/*
 * Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Zephyr CAN driver for YTMicro YTM32 FlexCAN.
 * Supports Classic CAN and CAN FD (when CONFIG_CAN_FD_MODE is selected).
 */

#define DT_DRV_COMPAT ytmicro_ytm32_flexcan

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/can/transceiver.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "ytm32_can_hal.h"

LOG_MODULE_REGISTER(can_ytm32, CONFIG_CAN_LOG_LEVEL);

/* ──────────────────────── config / data structs ──────────────────────── */

struct can_ytm32_rx_filter {
	bool              in_use;
	struct can_filter filter;    /* original user filter, kept for reference */
	can_rx_callback_t cb;
	void             *cb_data;
};

struct can_ytm32_tx_slot {
	bool              in_use;
	can_tx_callback_t cb;
	void             *cb_data;
};

struct can_ytm32_config {
	/* MUST be first: Zephyr CAN common config. */
	struct can_driver_config common;
	uint32_t                 base;
	uint8_t                  instance;
	uint8_t                  max_mb;
	uint8_t                  rx_mb_count;
	uint8_t                  tx_mb_count;
	bool                     supports_fd;
	const struct device     *clock_dev;
	clock_control_subsys_t   clock_subsys;
	const struct pinctrl_dev_config *pincfg;
	void (*irq_config_func)(void);
};

struct can_ytm32_data {
	/* MUST be first: Zephyr CAN common data (mode, started, callbacks). */
	struct can_driver_data    common;
	struct k_mutex            lock;
	struct can_ytm32_tx_slot  tx_slots[YTM32_CAN_HAL_MAX_TX_MB];
	struct k_sem              tx_sem;
	struct can_ytm32_rx_filter filters[CONFIG_CAN_MAX_FILTER];
	struct ytm32_can_timing   timing;
	struct ytm32_can_timing   timing_data; /* FD data phase */
	bool                      timing_valid;
	bool                      timing_data_valid;
};

/* ──────────────────────── instance helper ──────────────────────── */

#define YTM32_CAN_BASE_STRIDE 0x4000U
#define YTM32_CAN_BASE0       0x40030000U
#define YTM32_CAN_BASE_TO_INSTANCE(base) \
	((uint8_t)(((base) - YTM32_CAN_BASE0) / YTM32_CAN_BASE_STRIDE))

/* ──────────────────────── HAL event callback ──────────────────────── */

static void can_ytm32_hal_cb(void *user_data, ytm32_can_event_t event,
			     uint8_t slot, int status)
{
	const struct device   *dev  = user_data;
	const struct can_ytm32_config *cfg  = dev->config;
	struct can_ytm32_data *data = dev->data;

	switch (event) {
	case YTM32_CAN_EVENT_TX_DONE: {
		if (slot >= cfg->tx_mb_count) {
			break;
		}
		struct can_ytm32_tx_slot *ts = &data->tx_slots[slot];
		can_tx_callback_t cb  = ts->cb;
		void *cb_data         = ts->cb_data;

		ts->in_use = false;
		k_sem_give(&data->tx_sem);

		if (cb) {
			cb(dev, status, cb_data);
		}
		break;
	}

	case YTM32_CAN_EVENT_RX_DONE: {
		if (slot >= cfg->rx_mb_count ||
		    slot >= CONFIG_CAN_MAX_FILTER) {
			/*
			 * Re-arm even if we have no filter for this slot,
			 * so the MB doesn't stay idle.
			 */
			(void)ytm32_can_hal_rx_arm(cfg->instance, slot);
			break;
		}

		struct can_ytm32_rx_filter *f = &data->filters[slot];

		if (!f->in_use) {
			(void)ytm32_can_hal_rx_arm(cfg->instance, slot);
			break;
		}

		const struct ytm32_can_frame *hf =
			ytm32_can_hal_rx_frame(cfg->instance, slot);

		if (!hf) {
			(void)ytm32_can_hal_rx_arm(cfg->instance, slot);
			break;
		}

		/* Translate to Zephyr frame. */
		struct can_frame zf = {0};

		zf.id  = hf->id;
		zf.dlc = hf->dlc;
		if (hf->ide) {
			zf.flags |= CAN_FRAME_IDE;
		}
		if (hf->rtr) {
			zf.flags |= CAN_FRAME_RTR;
		}
#ifdef CONFIG_CAN_FD_MODE
		if (hf->fd) {
			zf.flags |= CAN_FRAME_FDF;
		}
		if (hf->brs) {
			zf.flags |= CAN_FRAME_BRS;
		}
#endif
		uint8_t copy_len = MIN(hf->dlc, (uint8_t)sizeof(zf.data));

		memcpy(zf.data, hf->data, copy_len);

		/* Re-arm before dispatching to avoid missing next frame. */
		(void)ytm32_can_hal_rx_arm(cfg->instance, slot);

		if (f->cb) {
			f->cb(dev, &zf, f->cb_data);
		}
		break;
	}

	case YTM32_CAN_EVENT_BUS_OFF:
		if (data->common.state_change_cb) {
			struct can_bus_err_cnt cnt = {0};

			ytm32_can_hal_get_err_cnt(cfg->instance,
						  &cnt.tx_err_cnt,
						  &cnt.rx_err_cnt);
			data->common.state_change_cb(
				dev, CAN_STATE_BUS_OFF, cnt,
				data->common.state_change_cb_user_data);
		}
		break;

	case YTM32_CAN_EVENT_BUS_OFF_DONE:
		if (data->common.state_change_cb) {
			struct can_bus_err_cnt cnt = {0};

			data->common.state_change_cb(
				dev, CAN_STATE_ERROR_ACTIVE, cnt,
				data->common.state_change_cb_user_data);
		}
		break;

	default:
		break;
	}
}

/* ──────────────────────── API implementations ──────────────────────── */

static int can_ytm32_get_capabilities(const struct device *dev,
				      can_mode_t *cap)
{
	const struct can_ytm32_config *cfg = dev->config;

	*cap = CAN_MODE_NORMAL | CAN_MODE_LOOPBACK | CAN_MODE_LISTENONLY;

#ifdef CONFIG_CAN_FD_MODE
	if (cfg->supports_fd) {
		*cap |= CAN_MODE_FD;
	}
#else
	ARG_UNUSED(cfg);
#endif

	return 0;
}

static int can_ytm32_start(const struct device *dev)
{
	const struct can_ytm32_config *cfg = dev->config;
	struct can_ytm32_data         *data = dev->data;
	int ret = 0;

	k_mutex_lock(&data->lock, K_FOREVER);

	if (data->common.started) {
		k_mutex_unlock(&data->lock);
		return -EALREADY;
	}

	/* Bring the TCAN3413 out of standby for this controller start. */
	if (cfg->common.phy != NULL) {
		ret = can_transceiver_enable(cfg->common.phy, data->common.mode);
		if (ret != 0) {
			LOG_ERR("CAN%u: transceiver enable failed: %d",
				cfg->instance, ret);
			goto out;
		}
	}

	/* Apply pending timing before leaving freeze. */
	if (data->timing_valid) {
		ret = ytm32_can_hal_set_timing(cfg->instance, &data->timing);
		if (ret < 0) {
			if (cfg->common.phy != NULL) {
				(void)can_transceiver_disable(cfg->common.phy);
			}
			goto out;
		}
	}

#ifdef CONFIG_CAN_FD_MODE
	if (data->timing_data_valid) {
		ret = ytm32_can_hal_set_timing_fd(cfg->instance,
						  &data->timing_data);
		if (ret < 0) {
			goto out;
		}
	}
#endif

	ytm32_can_mode_t hmode;

	switch (data->common.mode) {
	case CAN_MODE_NORMAL:
		hmode = YTM32_CAN_MODE_NORMAL;
		break;
	case CAN_MODE_LOOPBACK:
		hmode = YTM32_CAN_MODE_LOOPBACK;
		break;
	case CAN_MODE_LISTENONLY:
		hmode = YTM32_CAN_MODE_LISTENONLY;
		break;
#ifdef CONFIG_CAN_FD_MODE
	case (CAN_MODE_FD):
	case (CAN_MODE_FD | CAN_MODE_LOOPBACK):
		hmode = (data->common.mode & CAN_MODE_LOOPBACK)
			? YTM32_CAN_MODE_LOOPBACK : YTM32_CAN_MODE_NORMAL;
		break;
#endif
	default:
		ret = -ENOTSUP;
		goto out;
	}

	ret = ytm32_can_hal_set_mode(cfg->instance, hmode);
	if (ret == 0) {
		data->common.started = true;

		/* Re-arm all active RX filters after leaving freeze. */
		for (int i = 0;
		     i < MIN(CONFIG_CAN_MAX_FILTER, (int)cfg->rx_mb_count);
		     i++) {
			if (data->filters[i].in_use) {
				(void)ytm32_can_hal_rx_arm(cfg->instance,
							   (uint8_t)i);
			}
		}
	} else if (cfg->common.phy != NULL) {
		(void)can_transceiver_disable(cfg->common.phy);
	}

out:
	k_mutex_unlock(&data->lock);
	return ret;
}

static int can_ytm32_stop(const struct device *dev)
{
	const struct can_ytm32_config *cfg = dev->config;
	struct can_ytm32_data         *data = dev->data;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);

	if (!data->common.started) {
		k_mutex_unlock(&data->lock);
		return -EALREADY;
	}

	ret = ytm32_can_hal_set_mode(cfg->instance, YTM32_CAN_MODE_FREEZE);
	if (ret == 0) {
		data->common.started = false;

		if (cfg->common.phy != NULL) {
			ret = can_transceiver_disable(cfg->common.phy);
		}

		/* Abort pending TX and release semaphore slots. */
		for (int i = 0; i < cfg->tx_mb_count; i++) {
			if (data->tx_slots[i].in_use) {
				can_tx_callback_t cb  = data->tx_slots[i].cb;
				void *cb_data         = data->tx_slots[i].cb_data;

				(void)ytm32_can_hal_abort_tx(cfg->instance, i);
				data->tx_slots[i].in_use = false;
				k_sem_give(&data->tx_sem);

				if (cb) {
					cb(dev, -ENETDOWN, cb_data);
				}
			}
		}
	}

	k_mutex_unlock(&data->lock);
	return ret;
}

static int can_ytm32_set_mode(const struct device *dev, can_mode_t mode)
{
	const struct can_ytm32_config *cfg = dev->config;
	struct can_ytm32_data         *data = dev->data;

	if (data->common.started) {
		return -EBUSY;
	}

	can_mode_t supported = CAN_MODE_NORMAL | CAN_MODE_LOOPBACK |
			       CAN_MODE_LISTENONLY;

#ifdef CONFIG_CAN_FD_MODE
	if (cfg->supports_fd) {
		supported |= CAN_MODE_FD;
	}
#else
	ARG_UNUSED(cfg);
#endif

	if (mode & ~supported) {
		return -ENOTSUP;
	}

	data->common.mode = mode;
	return 0;
}

static int can_ytm32_set_timing(const struct device *dev,
				const struct can_timing *timing)
{
	const struct can_ytm32_config *cfg = dev->config;
	struct can_ytm32_data         *data = dev->data;
	int ret = 0;

	k_mutex_lock(&data->lock, K_FOREVER);

	data->timing = (struct ytm32_can_timing){
		.prop_seg   = timing->prop_seg,
		.phase_seg1 = timing->phase_seg1,
		.phase_seg2 = timing->phase_seg2,
		.prescaler  = timing->prescaler,
		.sjw        = timing->sjw,
	};
	data->timing_valid = true;

	if (data->common.started) {
		ret = ytm32_can_hal_set_timing(cfg->instance, &data->timing);
	}

	k_mutex_unlock(&data->lock);
	return ret;
}

#ifdef CONFIG_CAN_FD_MODE
static int can_ytm32_set_timing_data(const struct device *dev,
				     const struct can_timing *timing_data)
{
	const struct can_ytm32_config *cfg = dev->config;
	struct can_ytm32_data         *data = dev->data;
	int ret = 0;

	if (!cfg->supports_fd) {
		return -ENOTSUP;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	data->timing_data = (struct ytm32_can_timing){
		.prop_seg   = timing_data->prop_seg,
		.phase_seg1 = timing_data->phase_seg1,
		.phase_seg2 = timing_data->phase_seg2,
		.prescaler  = timing_data->prescaler,
		.sjw        = timing_data->sjw,
	};
	data->timing_data_valid = true;

	if (data->common.started) {
		ret = ytm32_can_hal_set_timing_fd(cfg->instance,
						  &data->timing_data);
	}

	k_mutex_unlock(&data->lock);
	return ret;
}
#endif /* CONFIG_CAN_FD_MODE */

static int can_ytm32_send(const struct device *dev,
			  const struct can_frame *frame,
			  k_timeout_t timeout,
			  can_tx_callback_t callback,
			  void *user_data)
{
	const struct can_ytm32_config *cfg = dev->config;
	struct can_ytm32_data         *data = dev->data;
	int ret;

	if (!data->common.started) {
		return -ENETDOWN;
	}
	if (data->common.mode == CAN_MODE_LISTENONLY) {
		return -ENOTSUP;
	}

#ifdef CONFIG_CAN_FD_MODE
	bool is_fd = (frame->flags & CAN_FRAME_FDF) != 0U;

	if (is_fd && !(data->common.mode & CAN_MODE_FD)) {
		return -ENOTSUP;
	}
	if (is_fd && frame->dlc > CANFD_MAX_DLC) {
		return -EINVAL;
	}
	if (!is_fd && frame->dlc > CAN_MAX_DLC) {
		return -EINVAL;
	}
#else
	if (frame->dlc > CAN_MAX_DLC) {
		return -EINVAL;
	}
#endif

	ret = k_sem_take(&data->tx_sem, timeout);
	if (ret < 0) {
		return -EAGAIN;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	int slot = -1;

	for (int i = 0; i < cfg->tx_mb_count; i++) {
		if (!data->tx_slots[i].in_use) {
			slot = i;
			break;
		}
	}
	if (slot < 0) {
		k_mutex_unlock(&data->lock);
		k_sem_give(&data->tx_sem);
		return -EIO;
	}

	data->tx_slots[slot].in_use  = true;
	data->tx_slots[slot].cb      = callback;
	data->tx_slots[slot].cb_data = user_data;

	struct ytm32_can_frame hf = {
		.id  = frame->id,
		.dlc = frame->dlc,
		.ide = (frame->flags & CAN_FRAME_IDE) != 0U,
		.rtr = (frame->flags & CAN_FRAME_RTR) != 0U,
#ifdef CONFIG_CAN_FD_MODE
		.fd  = (frame->flags & CAN_FRAME_FDF) != 0U,
		.brs = (frame->flags & CAN_FRAME_BRS) != 0U,
#else
		.fd  = false,
		.brs = false,
#endif
	};

	uint8_t dlen = can_dlc_to_bytes(frame->dlc);

	memcpy(hf.data, frame->data,
	       MIN(dlen, (uint8_t)sizeof(hf.data)));

	ret = ytm32_can_hal_send(cfg->instance, (uint8_t)slot, &hf);
	if (ret < 0) {
		data->tx_slots[slot].in_use = false;
		k_sem_give(&data->tx_sem);
	}

	k_mutex_unlock(&data->lock);
	return ret;
}

static int can_ytm32_add_rx_filter(const struct device *dev,
				   can_rx_callback_t callback,
				   void *user_data,
				   const struct can_filter *filter)
{
	const struct can_ytm32_config *cfg = dev->config;
	struct can_ytm32_data         *data = dev->data;
	int max = MIN(CONFIG_CAN_MAX_FILTER, (int)cfg->rx_mb_count);
	int filter_id = -ENOSPC;

	k_mutex_lock(&data->lock, K_FOREVER);

	for (int i = 0; i < max; i++) {
		if (!data->filters[i].in_use) {
			filter_id = i;
			break;
		}
	}
	if (filter_id < 0) {
		k_mutex_unlock(&data->lock);
		return -ENOSPC;
	}

	bool ide = (filter->flags & CAN_FILTER_IDE) != 0U;
#ifdef CONFIG_CAN_FD_MODE
	bool fd  = (data->common.mode & CAN_MODE_FD) != 0U;
#else
	bool fd  = false;
#endif

	int ret = ytm32_can_hal_config_rx(cfg->instance, (uint8_t)filter_id,
					  filter->id, filter->mask,
					  ide, false, fd);
	if (ret < 0) {
		k_mutex_unlock(&data->lock);
		return ret;
	}

	if (data->common.started) {
		ret = ytm32_can_hal_rx_arm(cfg->instance, (uint8_t)filter_id);
		if (ret < 0) {
			k_mutex_unlock(&data->lock);
			return ret;
		}
	}

	data->filters[filter_id].in_use  = true;
	data->filters[filter_id].filter  = *filter;
	data->filters[filter_id].cb      = callback;
	data->filters[filter_id].cb_data = user_data;

	k_mutex_unlock(&data->lock);
	return filter_id;
}

static void can_ytm32_remove_rx_filter(const struct device *dev,
				       int filter_id)
{
	struct can_ytm32_data *data = dev->data;

	if (filter_id < 0 || filter_id >= CONFIG_CAN_MAX_FILTER) {
		return;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	data->filters[filter_id].in_use = false;
	data->filters[filter_id].cb     = NULL;
	k_mutex_unlock(&data->lock);
}

#ifdef CONFIG_CAN_MANUAL_RECOVERY_MODE
static int can_ytm32_recover(const struct device *dev, k_timeout_t timeout)
{
	const struct can_ytm32_config *cfg = dev->config;
	struct can_ytm32_data         *data = dev->data;
	enum can_state state;

	ARG_UNUSED(timeout); /* hardware-timed recovery; timeout not applicable */

	(void)can_ytm32_get_state(dev, &state, NULL);
	if (state != CAN_STATE_BUS_OFF) {
		return -EALREADY;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	ytm32_can_hal_recover(cfg->instance);
	k_mutex_unlock(&data->lock);
	return 0;
}
#endif /* CONFIG_CAN_MANUAL_RECOVERY_MODE */

static int can_ytm32_get_state(const struct device *dev,
			       enum can_state *state,
			       struct can_bus_err_cnt *err_cnt)
{
	const struct can_ytm32_config *cfg = dev->config;
	struct can_ytm32_data         *data = dev->data;

	if (!data->common.started) {
		if (state) {
			*state = CAN_STATE_STOPPED;
		}
		if (err_cnt) {
			err_cnt->tx_err_cnt = 0U;
			err_cnt->rx_err_cnt = 0U;
		}
		return 0;
	}

	uint8_t txe = 0U, rxe = 0U;

	ytm32_can_hal_get_err_cnt(cfg->instance, &txe, &rxe);

	if (err_cnt) {
		err_cnt->tx_err_cnt = txe;
		err_cnt->rx_err_cnt = rxe;
	}

	if (state) {
		uint32_t esr = ytm32_can_hal_get_error_status(cfg->instance);

		/*
		 * CAN_ESR1_FLTCONF [5:4]:
		 *   00 = error active
		 *   01 = error warning  (TXE or RXE >= 96)
		 *   10 = error passive
		 *   11 = bus-off
		 * CAN_ESR1_BOFFINT [2] latches bus-off entry.
		 */
		uint8_t fltconf = (uint8_t)((esr >> 4U) & 0x03U);

		switch (fltconf) {
		case 0x03U:
			*state = CAN_STATE_BUS_OFF;
			break;
		case 0x02U:
			*state = CAN_STATE_ERROR_PASSIVE;
			break;
		case 0x01U:
			*state = CAN_STATE_ERROR_WARNING;
			break;
		default:
			*state = CAN_STATE_ERROR_ACTIVE;
			break;
		}
	}

	return 0;
}

static void can_ytm32_set_state_change_callback(
	const struct device *dev,
	can_state_change_callback_t callback,
	void *user_data)
{
	struct can_ytm32_data *data = dev->data;

	data->common.state_change_cb           = callback;
	data->common.state_change_cb_user_data = user_data;
}

static int can_ytm32_get_core_clock(const struct device *dev, uint32_t *rate)
{
	const struct can_ytm32_config *cfg = dev->config;

	return clock_control_get_rate(cfg->clock_dev, cfg->clock_subsys, rate);
}

static int can_ytm32_get_max_filters(const struct device *dev, bool ide)
{
	const struct can_ytm32_config *cfg = dev->config;

	ARG_UNUSED(ide);
	return MIN(CONFIG_CAN_MAX_FILTER, (int)cfg->rx_mb_count);
}

/* ──────────────────────── init ──────────────────────── */

static int can_ytm32_init(const struct device *dev)
{
	const struct can_ytm32_config *cfg = dev->config;
	struct can_ytm32_data         *data = dev->data;
	uint32_t clock_hz = 0U;
	int ret;

	k_mutex_init(&data->lock);
	k_sem_init(&data->tx_sem, cfg->tx_mb_count, cfg->tx_mb_count);

	if (!device_is_ready(cfg->clock_dev)) {
		LOG_ERR("CAN%u: clock device not ready", cfg->instance);
		return -ENODEV;
	}

	ret = clock_control_on(cfg->clock_dev, cfg->clock_subsys);
	if (ret < 0) {
		LOG_ERR("CAN%u: clock enable failed: %d", cfg->instance, ret);
		return ret;
	}

	ret = clock_control_get_rate(cfg->clock_dev, cfg->clock_subsys,
				     &clock_hz);
	if (ret < 0 || clock_hz == 0U) {
		LOG_ERR("CAN%u: clock rate query failed: %d", cfg->instance, ret);
		return (ret < 0) ? ret : -EIO;
	}

	if (cfg->pincfg) {
		ret = pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_DEFAULT);
		if (ret < 0) {
			LOG_ERR("CAN%u: pinctrl failed: %d", cfg->instance, ret);
			return ret;
		}
	}

	/* Calculate initial Classic CAN arbitration timing. */
	struct can_timing timing = {0};
	int terr = can_calc_timing(dev, &timing,
				   cfg->common.bitrate,
				   cfg->common.sample_point);

	if (terr == -EINVAL) {
		LOG_ERR("CAN%u: no timing for bitrate=%u sp=%u",
			cfg->instance, cfg->common.bitrate,
			cfg->common.sample_point);
		return -EIO;
	}
	LOG_DBG("CAN%u arb: bitrate=%u presc=%u prop=%u ph1=%u ph2=%u sjw=%u err=%d",
		cfg->instance, cfg->common.bitrate,
		timing.prescaler, timing.prop_seg,
		timing.phase_seg1, timing.phase_seg2, timing.sjw, terr);

	data->timing = (struct ytm32_can_timing){
		.prop_seg   = timing.prop_seg,
		.phase_seg1 = timing.phase_seg1,
		.phase_seg2 = timing.phase_seg2,
		.prescaler  = timing.prescaler,
		.sjw        = timing.sjw,
	};
	data->timing_valid = true;

#ifdef CONFIG_CAN_FD_MODE
	/* Calculate FD data phase timing if FD is supported. */
	if (cfg->supports_fd) {
		struct can_timing tdata = {0};
		int tderr = can_calc_timing_data(dev, &tdata,
						 cfg->common.bitrate_data,
						 cfg->common.sample_point_data);

		if (tderr != -EINVAL) {
			LOG_DBG("CAN%u data: bitrate=%u presc=%u prop=%u "
				"ph1=%u ph2=%u sjw=%u err=%d",
				cfg->instance, cfg->common.bitrate_data,
				tdata.prescaler, tdata.prop_seg,
				tdata.phase_seg1, tdata.phase_seg2,
				tdata.sjw, tderr);
			data->timing_data = (struct ytm32_can_timing){
				.prop_seg   = tdata.prop_seg,
				.phase_seg1 = tdata.phase_seg1,
				.phase_seg2 = tdata.phase_seg2,
				.prescaler  = tdata.prescaler,
				.sjw        = tdata.sjw,
			};
			data->timing_data_valid = true;
		}
	}
#endif

	ret = ytm32_can_hal_init(cfg->instance, clock_hz,
				 cfg->max_mb,
				 cfg->rx_mb_count, cfg->tx_mb_count,
				 &data->timing,
				 cfg->supports_fd,
				 can_ytm32_hal_cb, (void *)dev);
	if (ret < 0) {
		LOG_ERR("CAN%u: HAL init failed: %d", cfg->instance, ret);
		return ret;
	}

#ifdef CONFIG_CAN_FD_MODE
	if (cfg->supports_fd && data->timing_data_valid) {
		ret = ytm32_can_hal_set_timing_fd(cfg->instance,
						  &data->timing_data);
		if (ret < 0) {
			LOG_ERR("CAN%u: FD data phase timing failed: %d",
				cfg->instance, ret);
			return ret;
		}
	}
#endif

	cfg->irq_config_func();

	LOG_INF("CAN%u ready, clock=%u Hz", cfg->instance, clock_hz);
	return 0;
}

/* ──────────────────────── driver API ──────────────────────── */

static DEVICE_API(can, can_ytm32_driver_api) = {
	.get_capabilities          = can_ytm32_get_capabilities,
	.start                     = can_ytm32_start,
	.stop                      = can_ytm32_stop,
	.set_mode                  = can_ytm32_set_mode,
	.set_timing                = can_ytm32_set_timing,
	.send                      = can_ytm32_send,
	.add_rx_filter             = can_ytm32_add_rx_filter,
	.remove_rx_filter          = can_ytm32_remove_rx_filter,
	.get_state                 = can_ytm32_get_state,
	.set_state_change_callback = can_ytm32_set_state_change_callback,
	.get_core_clock            = can_ytm32_get_core_clock,
	.get_max_filters           = can_ytm32_get_max_filters,
#ifdef CONFIG_CAN_MANUAL_RECOVERY_MODE
	.recover                   = can_ytm32_recover,
#endif
#ifdef CONFIG_CAN_FD_MODE
	.set_timing_data           = can_ytm32_set_timing_data,
#endif
	/*
	 * Classic CAN CTRL1 timing limits (physical values, not register values).
	 *   presc: 1..256, prop: 1..8, ph1: 1..8, ph2: 2..8, sjw: 1..4
	 */
	.timing_min = {
		.sjw        = 1U,
		.prop_seg   = 1U,
		.phase_seg1 = 1U,
		.phase_seg2 = 2U,
		.prescaler  = 1U,
	},
	.timing_max = {
		.sjw        = 4U,
		.prop_seg   = 8U,
		.phase_seg1 = 8U,
		.phase_seg2 = 8U,
		.prescaler  = 256U,
	},
#ifdef CONFIG_CAN_FD_MODE
	/*
	 * CAN FD data phase timing limits (CBT/FDCBT registers).
	 *   presc: 1..1024, prop: 0..31, ph1: 1..8, ph2: 2..8, sjw: 1..8
	 * prop_seg=0 is valid for FD data phase.
	 */
	.timing_data_min = {
		.sjw        = 1U,
		.prop_seg   = 0U,
		.phase_seg1 = 1U,
		.phase_seg2 = 2U,
		.prescaler  = 1U,
	},
	.timing_data_max = {
		.sjw        = 8U,
		.prop_seg   = 31U,
		.phase_seg1 = 8U,
		.phase_seg2 = 8U,
		.prescaler  = 1024U,
	},
#endif
};

/* ──────────────────────── IRQ wiring helpers ──────────────────────── */

#define YTM32_CAN_ISR(inst, grp)						\
	static void can_ytm32_isr_##inst##_##grp(const struct device *dev)	\
	{									\
		ytm32_can_hal_irq(						\
			((const struct can_ytm32_config *)dev->config)->instance, \
			grp);							\
	}

#define YTM32_CAN_CONNECT(node, inst, grp)					\
	do {									\
		IRQ_CONNECT(DT_IRQ_BY_IDX(node, grp, irq),			\
			    DT_IRQ_BY_IDX(node, grp, priority),			\
			    can_ytm32_isr_##inst##_##grp,			\
			    DEVICE_DT_GET(node), 0);				\
		irq_enable(DT_IRQ_BY_IDX(node, grp, irq));			\
	} while (0)

/* 5-IRQ variant: can1, can2 */
#define YTM32_CAN_IRQ5(node, inst)						\
	YTM32_CAN_ISR(inst, 0)							\
	YTM32_CAN_ISR(inst, 1)							\
	YTM32_CAN_ISR(inst, 2)							\
	YTM32_CAN_ISR(inst, 3)							\
	YTM32_CAN_ISR(inst, 4)							\
	static void can_ytm32_irq_config_##inst(void)				\
	{									\
		YTM32_CAN_CONNECT(node, inst, 0);				\
		YTM32_CAN_CONNECT(node, inst, 1);				\
		YTM32_CAN_CONNECT(node, inst, 2);				\
		YTM32_CAN_CONNECT(node, inst, 3);				\
		YTM32_CAN_CONNECT(node, inst, 4);				\
	}

/* 7-IRQ variant: can0 (MB_32_47 and MB_48_63 only on instance 0) */
#define YTM32_CAN_IRQ7(node, inst)						\
	YTM32_CAN_ISR(inst, 0)							\
	YTM32_CAN_ISR(inst, 1)							\
	YTM32_CAN_ISR(inst, 2)							\
	YTM32_CAN_ISR(inst, 3)							\
	YTM32_CAN_ISR(inst, 4)							\
	YTM32_CAN_ISR(inst, 5)							\
	YTM32_CAN_ISR(inst, 6)							\
	static void can_ytm32_irq_config_##inst(void)				\
	{									\
		YTM32_CAN_CONNECT(node, inst, 0);				\
		YTM32_CAN_CONNECT(node, inst, 1);				\
		YTM32_CAN_CONNECT(node, inst, 2);				\
		YTM32_CAN_CONNECT(node, inst, 3);				\
		YTM32_CAN_CONNECT(node, inst, 4);				\
		YTM32_CAN_CONNECT(node, inst, 5);				\
		YTM32_CAN_CONNECT(node, inst, 6);				\
	}

/* ──────────────────────── per-instance instantiation ──────────────────── */

#define YTM32_CAN_INIT(inst)							\
	PINCTRL_DT_INST_DEFINE(inst);						\
										\
	COND_CODE_1(DT_INST_PROP_HAS_IDX(inst, interrupts, 6),			\
		    (YTM32_CAN_IRQ7(DT_DRV_INST(inst), inst)),			\
		    (YTM32_CAN_IRQ5(DT_DRV_INST(inst), inst)))			\
										\
	static const struct can_ytm32_config can_ytm32_cfg_##inst = {		\
		.common = CAN_DT_DRIVER_CONFIG_INST_GET(inst, 0, 8000000),	\
		.base          = DT_INST_REG_ADDR(inst),			\
		.instance      = YTM32_CAN_BASE_TO_INSTANCE(DT_INST_REG_ADDR(inst)),	\
		.max_mb        = DT_INST_PROP(inst, max_mb),			\
		.rx_mb_count   = DT_INST_PROP(inst, rx_mb_count),		\
		.tx_mb_count   = DT_INST_PROP(inst, tx_mb_count),		\
		.supports_fd   = DT_INST_PROP(inst, supports_fd),		\
		.clock_dev     = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(inst)),	\
		.clock_subsys  = (clock_control_subsys_t)			\
				 DT_INST_CLOCKS_CELL(inst, id),			\
		.pincfg        = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),		\
		.irq_config_func = can_ytm32_irq_config_##inst,			\
	};									\
										\
	static struct can_ytm32_data can_ytm32_data_##inst;			\
										\
	CAN_DEVICE_DT_INST_DEFINE(inst, can_ytm32_init, NULL,			\
				  &can_ytm32_data_##inst,			\
				  &can_ytm32_cfg_##inst,			\
				  POST_KERNEL, CONFIG_CAN_INIT_PRIORITY,	\
				  &can_ytm32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(YTM32_CAN_INIT)
