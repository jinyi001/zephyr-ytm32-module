/*
 * Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * YTM32 FlexCAN HAL wrapper interface.
 *
 * Only standard C types are exposed here — no vendor or Zephyr headers are
 * included, preventing typedef collisions between the two SDKs.
 */

#ifndef YTM32_CAN_HAL_H
#define YTM32_CAN_HAL_H

#include <stdbool.h>
#include <stdint.h>

/* Maximum RX/TX MB slots tracked per instance by this wrapper. */
#define YTM32_CAN_HAL_MAX_RX_MB  16U
#define YTM32_CAN_HAL_MAX_TX_MB  8U

/* Maximum CAN FD data payload in bytes. */
#define YTM32_CAN_HAL_FD_DLEN    64U

typedef enum {
	YTM32_CAN_EVENT_TX_DONE,
	YTM32_CAN_EVENT_RX_DONE,
	YTM32_CAN_EVENT_ERROR,       /* bit/stuff/crc/form/ack error */
	YTM32_CAN_EVENT_BUS_OFF,
	YTM32_CAN_EVENT_BUS_OFF_DONE, /* bus-off recovery complete */
	YTM32_CAN_EVENT_WAKEUP,
} ytm32_can_event_t;

/*
 * Event callback invoked from ISR context.
 *   user_data : value provided to ytm32_can_hal_init
 *   event     : event type
 *   mb_idx    : RX/TX slot index (0-based within reservation); 0xFF for bus events
 *   status    : 0 on success, negative errno on error
 */
typedef void (*ytm32_can_evt_cb_t)(void *user_data,
				   ytm32_can_event_t event,
				   uint8_t mb_idx,
				   int status);

/* Classic CAN / CAN FD arbitration phase timing. */
struct ytm32_can_timing {
	uint32_t prop_seg;    /* propagation segment (time quanta, physical) */
	uint32_t phase_seg1;  /* phase segment 1 (time quanta, physical) */
	uint32_t phase_seg2;  /* phase segment 2 (time quanta, physical) */
	uint32_t prescaler;   /* clock prescaler (physical >= 1) */
	uint32_t sjw;         /* synchronisation jump width (time quanta, physical) */
};

/* CAN frame (classic and FD). */
struct ytm32_can_frame {
	uint32_t id;
	uint8_t  data[YTM32_CAN_HAL_FD_DLEN];
	uint8_t  dlc;
	bool     ide;
	bool     rtr;
	bool     fd;   /* CAN FD format */
	bool     brs;  /* bit-rate switch */
};

typedef enum {
	YTM32_CAN_MODE_NORMAL,
	YTM32_CAN_MODE_LOOPBACK,
	YTM32_CAN_MODE_LISTENONLY,
	YTM32_CAN_MODE_FREEZE,
} ytm32_can_mode_t;

/*
 * Initialise a FlexCAN instance.
 *   instance   : 0-2 on YTM32B1MD1
 *   clock_hz   : functional clock in Hz
 *   max_mb     : total MBs to configure
 *   rx_count   : first rx_count MBs reserved for RX (0..rx_count-1)
 *   tx_count   : next tx_count MBs reserved for TX (rx_count..rx_count+tx_count-1)
 *   timing     : initial Classic CAN / FD arbitration phase timing
 *   fd_enable  : enable CAN FD
 *   cb         : event callback (called from ISR)
 *   user_data  : forwarded to cb
 */
int ytm32_can_hal_init(uint8_t instance, uint32_t clock_hz,
		       uint8_t max_mb, uint8_t rx_count, uint8_t tx_count,
		       const struct ytm32_can_timing *timing,
		       bool fd_enable,
		       ytm32_can_evt_cb_t cb, void *user_data);

/* Set Classic CAN / FD arbitration phase timing (controller must be in freeze). */
int ytm32_can_hal_set_timing(uint8_t instance,
			     const struct ytm32_can_timing *timing);

/* Set CAN FD data phase timing (controller must be in freeze). */
int ytm32_can_hal_set_timing_fd(uint8_t instance,
				const struct ytm32_can_timing *timing_data);

/* Change operating mode. */
int ytm32_can_hal_set_mode(uint8_t instance, ytm32_can_mode_t mode);

/*
 * Configure one RX MB slot.
 *   rx_slot : 0 .. rx_count-1
 *   id / mask / ide / rtr / fd : filter parameters
 */
int ytm32_can_hal_config_rx(uint8_t instance, uint8_t rx_slot,
			    uint32_t id, uint32_t mask,
			    bool ide, bool rtr, bool fd);

/* Arm an RX MB to receive the next matching frame. */
int ytm32_can_hal_rx_arm(uint8_t instance, uint8_t rx_slot);

/* Send a frame via a TX MB slot (0 .. tx_count-1). */
int ytm32_can_hal_send(uint8_t instance, uint8_t tx_slot,
		       const struct ytm32_can_frame *frame);

/* Abort an in-progress TX. */
int ytm32_can_hal_abort_tx(uint8_t instance, uint8_t tx_slot);

/*
 * Return pointer to the decoded frame received in rx_slot.
 * Valid only after YTM32_CAN_EVENT_RX_DONE; owned by the wrapper.
 */
const struct ytm32_can_frame *ytm32_can_hal_rx_frame(uint8_t instance,
						     uint8_t rx_slot);

/* Raw CAN_ESR1 register value. */
uint32_t ytm32_can_hal_get_error_status(uint8_t instance);

/* TX/RX error counters from CAN_ECR. */
void ytm32_can_hal_get_err_cnt(uint8_t instance,
			       uint8_t *tx_err, uint8_t *rx_err);

/*
 * Manually initiate bus-off recovery (clear BOFFREC bit, then set it back
 * to let the hardware auto-recover).  No-op if not in bus-off state.
 */
void ytm32_can_hal_recover(uint8_t instance);

/* Deinitialise (enter disabled mode, clear SDK state pointer). */
void ytm32_can_hal_deinit(uint8_t instance);

/*
 * Per-IRQ dispatcher — call from Zephyr ISRs.
 * group encoding matches interrupt-names order in the DTS:
 *   0=ored  1=error  2=wakeup  3=mb_0_15  4=mb_16_31  5=mb_32_47  6=mb_48_63
 */
void ytm32_can_hal_irq(uint8_t instance, uint8_t group);

#endif /* YTM32_CAN_HAL_H */
