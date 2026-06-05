/*
 * Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ytmicro_ytm32_gpio

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>

#include "device_registers.h"

LOG_MODULE_REGISTER(gpio_ytm32, CONFIG_GPIO_LOG_LEVEL);

/*
 * YTM32 GPIO register layout (GPIO_Type):
 *   0x00 PDOR - Port Data Output Register
 *   0x04 PSOR - Port Set Output Register
 *   0x08 PCOR - Port Clear Output Register
 *   0x0C PTOR - Port Toggle Output Register
 *   0x10 PDIR - Port Data Input Register
 *   0x14 POER - Port Output Enable Register (1=output)
 *   0x18 PIER - Port Input Enable Register  (1=enabled)
 *   0x1C PIFR - Port Interrupt Flag Register (W1C)
 *   0x80 PCR[32] - Per-pin Control Register (IRQC in bits [3:0])
 *
 * Pin muxing is handled by the pinctrl driver via PCTRL peripheral.
 * This GPIO driver only manages direction, data, and interrupts.
 */

/*
 * Register layout (GPIO_Type, PCTRL_Type) and the PCR field masks/shifts come
 * from the vendor HAL device header selected by device_registers.h.  Only the
 * IRQC field value encodings and the GPIO mux-value below are not part of that
 * header (they are field values, not register-layout macros).
 */

/* GPIO PCR IRQC field value encodings (RM §6.2.1.9) */
#define YTM32_GPIO_IRQC_DISABLED       0x0U   /* Interrupt disabled              */
#define YTM32_GPIO_IRQC_INT_LOGIC_ZERO 0x8U   /* Interrupt when logic 0          */
#define YTM32_GPIO_IRQC_INT_RISING     0x9U   /* Interrupt on rising-edge        */
#define YTM32_GPIO_IRQC_INT_FALLING    0xAU   /* Interrupt on falling-edge       */
#define YTM32_GPIO_IRQC_INT_EITHER     0xBU   /* Interrupt on either edge        */
#define YTM32_GPIO_IRQC_INT_LOGIC_ONE  0xCU   /* Interrupt when logic 1          */

/* PCTRL PCR MUX field value: alternative 1 = GPIO function (RM §7.3.1.1) */
#define YTM32_GPIO_PCTRL_MUX_AS_GPIO    1U

struct gpio_ytm32_config {
	/* gpio_driver_config needs to be first */
	struct gpio_driver_config common;
	uintptr_t base;       /* GPIO register base */
	uintptr_t pctrl_base; /* PCTRL register base (pin mux) */
	const struct device *clk_dev;
	uint32_t clk_id;
	void (*irq_config_func)(const struct device *dev);
};

struct gpio_ytm32_data {
	/* gpio_driver_data needs to be first */
	struct gpio_driver_data common;
	sys_slist_t callbacks;
	uint8_t irqc_cache[32];
};

static inline GPIO_Type *gpio_regs(const struct gpio_ytm32_config *cfg)
{
	return (GPIO_Type *)cfg->base;
}

static inline PCTRL_Type *pctrl_regs(const struct gpio_ytm32_config *cfg)
{
	return (PCTRL_Type *)cfg->pctrl_base;
}

static int gpio_ytm32_configure(const struct device *dev,
				gpio_pin_t pin, gpio_flags_t flags)
{
	const struct gpio_ytm32_config *cfg = dev->config;
	GPIO_Type *gpio = gpio_regs(cfg);
	PCTRL_Type *pctrl = pctrl_regs(cfg);

	/* Reject unsupported flags */
	if ((flags & GPIO_SINGLE_ENDED) != 0) {
		/* YTM32B1MC0 does not support open-drain */
		return -ENOTSUP;
	}

	if (flags == GPIO_DISCONNECTED) {
		/* Disconnect: disable both output and input */
		gpio->POER &= ~BIT(pin);
		gpio->PIER &= ~BIT(pin);
		return 0;
	}

	/* Set pin mux to GPIO mode (mux=1) via PCTRL->PCR[pin] */
	{
		uint32_t pcr = pctrl->PCR[pin];

		pcr &= ~PCTRL_PCR_MUX_MASK;
		pcr |= (YTM32_GPIO_PCTRL_MUX_AS_GPIO << PCTRL_PCR_MUX_SHIFT) & PCTRL_PCR_MUX_MASK;
		pctrl->PCR[pin] = pcr;
	}

	/* Configure pull-up / pull-down via PCTRL PCR[pin] */
	{
		uint32_t pcr = pctrl->PCR[pin];

		pcr &= ~(PCTRL_PCR_PE_MASK | PCTRL_PCR_PS_MASK);

		if ((flags & GPIO_PULL_UP) != 0) {
			pcr |= PCTRL_PCR_PE_MASK | PCTRL_PCR_PS_MASK;
		} else if ((flags & GPIO_PULL_DOWN) != 0) {
			pcr |= PCTRL_PCR_PE_MASK;
		}

		pctrl->PCR[pin] = pcr;
	}

	if ((flags & GPIO_OUTPUT) != 0) {
		/* Set initial value before enabling output */
		if ((flags & GPIO_OUTPUT_INIT_HIGH) != 0) {
			gpio->PSOR = BIT(pin);
		} else if ((flags & GPIO_OUTPUT_INIT_LOW) != 0) {
			gpio->PCOR = BIT(pin);
		}

		/* Enable output */
		gpio->POER |= BIT(pin);

		/* If also input (bidirectional), enable input as well */
		if ((flags & GPIO_INPUT) != 0) {
			gpio->PIER |= BIT(pin);
		} else {
			gpio->PIER &= ~BIT(pin);
		}
	} else if ((flags & GPIO_INPUT) != 0) {
		/* Input only: disable output, enable input */
		gpio->POER &= ~BIT(pin);
		gpio->PIER |= BIT(pin);
	}

	return 0;
}

static int gpio_ytm32_port_get_raw(const struct device *dev,
				   gpio_port_value_t *value)
{
	const struct gpio_ytm32_config *cfg = dev->config;

	*value = gpio_regs(cfg)->PDIR;
	return 0;
}

static int gpio_ytm32_port_set_masked_raw(const struct device *dev,
					  gpio_port_pins_t mask,
					  gpio_port_value_t value)
{
	const struct gpio_ytm32_config *cfg = dev->config;
	GPIO_Type *gpio = gpio_regs(cfg);

	gpio->PSOR = mask & value;
	gpio->PCOR = mask & ~value;

	return 0;
}

static int gpio_ytm32_port_set_bits_raw(const struct device *dev,
					gpio_port_pins_t pins)
{
	const struct gpio_ytm32_config *cfg = dev->config;

	gpio_regs(cfg)->PSOR = pins;
	return 0;
}

static int gpio_ytm32_port_clear_bits_raw(const struct device *dev,
					  gpio_port_pins_t pins)
{
	const struct gpio_ytm32_config *cfg = dev->config;

	gpio_regs(cfg)->PCOR = pins;
	return 0;
}

static int gpio_ytm32_port_toggle_bits(const struct device *dev,
				       gpio_port_pins_t pins)
{
	const struct gpio_ytm32_config *cfg = dev->config;

	gpio_regs(cfg)->PTOR = pins;
	return 0;
}

static int gpio_ytm32_pin_interrupt_configure(const struct device *dev,
					      gpio_pin_t pin,
					      enum gpio_int_mode mode,
					      enum gpio_int_trig trig)
{
	const struct gpio_ytm32_config *cfg = dev->config;
	struct gpio_ytm32_data *data = dev->data;
	GPIO_Type *gpio = gpio_regs(cfg);
	uint32_t pcr;
	uint32_t irqc = YTM32_GPIO_IRQC_DISABLED;
	bool clear_pending = true;
#ifdef CONFIG_GPIO_ENABLE_DISABLE_INTERRUPT
	bool fire_pending = false;
	gpio_port_pins_t pending = 0U;
#endif

	trig &= ~GPIO_INT_WAKEUP;

#ifdef CONFIG_GPIO_ENABLE_DISABLE_INTERRUPT
	if (mode == GPIO_INT_MODE_DISABLE_ONLY) {
		clear_pending = false;
	} else if (mode == GPIO_INT_MODE_ENABLE_ONLY) {
		irqc = data->irqc_cache[pin];
		if (irqc == YTM32_GPIO_IRQC_DISABLED) {
			return 0;
		}
		clear_pending = false;
		fire_pending = true;
	} else
#endif
	{
		switch (mode) {
		case GPIO_INT_MODE_DISABLED:
			data->irqc_cache[pin] = YTM32_GPIO_IRQC_DISABLED;
			break;
		case GPIO_INT_MODE_LEVEL:
			if (trig == GPIO_INT_TRIG_LOW) {
				irqc = YTM32_GPIO_IRQC_INT_LOGIC_ZERO;
			} else if (trig == GPIO_INT_TRIG_HIGH) {
				irqc = YTM32_GPIO_IRQC_INT_LOGIC_ONE;
			} else {
				/* Both-level not supported */
				return -ENOTSUP;
			}
			data->irqc_cache[pin] = irqc;
			break;
		case GPIO_INT_MODE_EDGE:
			if (trig == GPIO_INT_TRIG_LOW) {
				irqc = YTM32_GPIO_IRQC_INT_FALLING;
			} else if (trig == GPIO_INT_TRIG_HIGH) {
				irqc = YTM32_GPIO_IRQC_INT_RISING;
			} else {
				irqc = YTM32_GPIO_IRQC_INT_EITHER;
			}
			data->irqc_cache[pin] = irqc;
			break;
		default:
			return -ENOTSUP;
		}
	}

	/* Write IRQC to GPIO PCR[pin] */
	pcr = gpio->PCR[pin];
	pcr &= ~GPIO_PCR_IRQC_MASK;
	pcr |= (irqc << GPIO_PCR_IRQC_SHIFT) & GPIO_PCR_IRQC_MASK;
	gpio->PCR[pin] = pcr;

	if (clear_pending) {
		gpio->PIFR = BIT(pin);
	}

#ifdef CONFIG_GPIO_ENABLE_DISABLE_INTERRUPT
	if (fire_pending) {
		pending = gpio->PIFR & BIT(pin);
		if (pending != 0U) {
			gpio->PIFR = pending;
			gpio_fire_callbacks(&data->callbacks, dev, pending);
		}
	}
#endif

	return 0;
}

static uint32_t gpio_ytm32_get_pending_int(const struct device *dev)
{
	const struct gpio_ytm32_config *cfg = dev->config;

	return gpio_regs(cfg)->PIFR;
}

static int gpio_ytm32_manage_callback(const struct device *dev,
				      struct gpio_callback *callback,
				      bool set)
{
	struct gpio_ytm32_data *data = dev->data;

	return gpio_manage_callback(&data->callbacks, callback, set);
}

#ifdef CONFIG_GPIO_GET_DIRECTION
static int gpio_ytm32_port_get_direction(const struct device *dev,
					 gpio_port_pins_t map,
					 gpio_port_pins_t *inputs,
					 gpio_port_pins_t *outputs)
{
	const struct gpio_ytm32_config *cfg = dev->config;
	GPIO_Type *gpio = gpio_regs(cfg);

	if (inputs != NULL) {
		*inputs = gpio->PIER & map;
	}
	if (outputs != NULL) {
		*outputs = gpio->POER & map;
	}

	return 0;
}
#endif

static void gpio_ytm32_isr(const struct device *dev)
{
	const struct gpio_ytm32_config *cfg = dev->config;
	struct gpio_ytm32_data *data = dev->data;
	GPIO_Type *gpio = gpio_regs(cfg);
	uint32_t int_status;

	int_status = gpio->PIFR;

	/* Clear all pending flags (W1C) */
	gpio->PIFR = int_status;

	gpio_fire_callbacks(&data->callbacks, dev, int_status);
}

static DEVICE_API(gpio, gpio_ytm32_api) = {
	.pin_configure = gpio_ytm32_configure,
	.port_get_raw = gpio_ytm32_port_get_raw,
	.port_set_masked_raw = gpio_ytm32_port_set_masked_raw,
	.port_set_bits_raw = gpio_ytm32_port_set_bits_raw,
	.port_clear_bits_raw = gpio_ytm32_port_clear_bits_raw,
	.port_toggle_bits = gpio_ytm32_port_toggle_bits,
	.pin_interrupt_configure = gpio_ytm32_pin_interrupt_configure,
	.manage_callback = gpio_ytm32_manage_callback,
	.get_pending_int = gpio_ytm32_get_pending_int,
#ifdef CONFIG_GPIO_GET_DIRECTION
	.port_get_direction = gpio_ytm32_port_get_direction,
#endif
};

static int gpio_ytm32_init(const struct device *dev)
{
	const struct gpio_ytm32_config *cfg = dev->config;
	uint32_t clock_rate;
	int ret;

	/* Enable GPIO peripheral clock */
	if (cfg->clk_dev != NULL) {
		if (!device_is_ready(cfg->clk_dev)) {
			LOG_ERR("Clock device not ready");
			return -ENODEV;
		}
		ret = clock_control_on(cfg->clk_dev,
				       (clock_control_subsys_t)(uintptr_t)cfg->clk_id);
		if (ret < 0) {
			return ret;
		}

		ret = clock_control_get_rate(cfg->clk_dev,
				       (clock_control_subsys_t)(uintptr_t)cfg->clk_id,
				       &clock_rate);
		if (ret < 0) {
			return ret;
		}

		LOG_INF("%s functional clock: %u Hz", dev->name, clock_rate);
	}

	cfg->irq_config_func(dev);

	return 0;
}

#define GPIO_YTM32_IRQ_CONFIG(n)						\
	static void gpio_ytm32_irq_config_##n(const struct device *dev)		\
	{									\
		IRQ_CONNECT(DT_INST_IRQN(n),					\
			    DT_INST_IRQ(n, priority),				\
			    gpio_ytm32_isr,					\
			    DEVICE_DT_INST_GET(n),				\
			    0);							\
		irq_enable(DT_INST_IRQN(n));					\
	}

#define GPIO_YTM32_INIT(n)							\
	GPIO_YTM32_IRQ_CONFIG(n)						\
										\
	static const struct gpio_ytm32_config gpio_ytm32_config_##n = {		\
		.common = {							\
			.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(n),	\
		},								\
		.base = DT_INST_REG_ADDR_BY_IDX(n, 0),				\
		.pctrl_base = DT_INST_REG_ADDR_BY_IDX(n, 1),			\
		.clk_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),		\
		.clk_id = DT_INST_CLOCKS_CELL(n, id),				\
		.irq_config_func = gpio_ytm32_irq_config_##n,			\
	};									\
										\
	static struct gpio_ytm32_data gpio_ytm32_data_##n;			\
										\
	DEVICE_DT_INST_DEFINE(n, gpio_ytm32_init, NULL,				\
			      &gpio_ytm32_data_##n,				\
			      &gpio_ytm32_config_##n,				\
			      PRE_KERNEL_1,					\
			      CONFIG_GPIO_INIT_PRIORITY,			\
			      &gpio_ytm32_api);

DT_INST_FOREACH_STATUS_OKAY(GPIO_YTM32_INIT)
