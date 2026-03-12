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

/* ---------------------------------------------------------------------------
 * GPIO register offsets (RM §6.2.1, Table 6.1)
 * ---------------------------------------------------------------------------*/
#define GPIO_PDOR_OFFSET 0x00  /* Port Data Output Register        (RW)  */
#define GPIO_PSOR_OFFSET 0x04  /* Port Set Output Register         (WO)  */
#define GPIO_PCOR_OFFSET 0x08  /* Port Clear Output Register       (WO)  */
#define GPIO_PTOR_OFFSET 0x0C  /* Port Toggle Output Register      (WO)  */
#define GPIO_PDIR_OFFSET 0x10  /* Port Data Input Register         (RO)  */
#define GPIO_POER_OFFSET 0x14  /* Port Output Enable Register      (RW)  */
#define GPIO_PIER_OFFSET 0x18  /* Port Input Enable Register       (RW)  */
#define GPIO_PIFR_OFFSET 0x1C  /* Port Interrupt Status Flag Reg   (W1C) */
#define GPIO_PCR_OFFSET  0x80  /* Per-pin Control Register base    (RW)  */

/* Per-pin register stride: PCR[n] is at GPIO_PCR_OFFSET + n * PCR_REG_STRIDE,
 * likewise PCTRL PCR[n] is at pctrl_base + n * PCR_REG_STRIDE.             */
#define PCR_REG_STRIDE   4U

/* ---------------------------------------------------------------------------
 * GPIO PCR register fields (RM §6.2.1.9, Table 6.10)
 * ---------------------------------------------------------------------------*/
#define GPIO_PCR_INVE_BIT   4U     /* Invert Enable (bit 4)           */
#define GPIO_PCR_IRQC_MASK  0xFU   /* Interrupt Configuration [3:0]   */
#define GPIO_PCR_IRQC_SHIFT 0U

/* IRQC field encodings (RM §6.2.1.9) */
#define IRQC_DISABLED       0x0U   /* Interrupt disabled              */
#define IRQC_INT_LOGIC_ZERO 0x8U   /* Interrupt when logic 0          */
#define IRQC_INT_RISING     0x9U   /* Interrupt on rising-edge        */
#define IRQC_INT_FALLING    0xAU   /* Interrupt on falling-edge       */
#define IRQC_INT_EITHER     0xBU   /* Interrupt on either edge        */
#define IRQC_INT_LOGIC_ONE  0xCU   /* Interrupt when logic 1          */

/* ---------------------------------------------------------------------------
 * PCTRL PCR register fields — pin mux control (RM §7.3.1.1, Table 7.3)
 * ---------------------------------------------------------------------------*/
#define PCTRL_PCR_PE_BIT     1U      /* Pull Enable                      */
#define PCTRL_PCR_PS_BIT     0U      /* Pull Select (0=down, 1=up)       */
#define PCTRL_PCR_MUX_MASK   0x700U  /* Port Function Selection [10:8] */
#define PCTRL_PCR_MUX_SHIFT  8U
#define PCTRL_MUX_AS_GPIO    1U      /* Alternative 1 = GPIO function  */

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
};

static inline uint32_t gpio_reg_read(uintptr_t base, uint32_t offset)
{
	return sys_read32(base + offset);
}

static inline void gpio_reg_write(uintptr_t base, uint32_t offset, uint32_t val)
{
	sys_write32(val, base + offset);
}

static int gpio_ytm32_configure(const struct device *dev,
				gpio_pin_t pin, gpio_flags_t flags)
{
	const struct gpio_ytm32_config *cfg = dev->config;
	uintptr_t base = cfg->base;
	uint32_t poer;
	uint32_t pier;

	/* Reject unsupported flags */
	if ((flags & GPIO_SINGLE_ENDED) != 0) {
		/* YTM32B1MC0 does not support open-drain */
		return -ENOTSUP;
	}

	if (flags == GPIO_DISCONNECTED) {
		/* Disconnect: disable both output and input */
		poer = gpio_reg_read(base, GPIO_POER_OFFSET);
		poer &= ~BIT(pin);
		gpio_reg_write(base, GPIO_POER_OFFSET, poer);

		pier = gpio_reg_read(base, GPIO_PIER_OFFSET);
		pier &= ~BIT(pin);
		gpio_reg_write(base, GPIO_PIER_OFFSET, pier);
		return 0;
	}

	/* Set pin mux to GPIO mode (mux=1) via PCTRL->PCR[pin] */
	{
		uintptr_t pctrl_base = cfg->pctrl_base;
		uint32_t pcr = sys_read32(pctrl_base + pin * PCR_REG_STRIDE);
		pcr &= ~PCTRL_PCR_MUX_MASK;
		pcr |= (PCTRL_MUX_AS_GPIO << PCTRL_PCR_MUX_SHIFT) & PCTRL_PCR_MUX_MASK;
		sys_write32(pcr, pctrl_base + pin * PCR_REG_STRIDE);
	}

	/* Configure pull-up / pull-down via PCTRL PCR[pin] */
	{
		uintptr_t pctrl_base = cfg->pctrl_base;
		uint32_t pcr = sys_read32(pctrl_base + pin * PCR_REG_STRIDE);

		pcr &= ~(BIT(PCTRL_PCR_PE_BIT) | BIT(PCTRL_PCR_PS_BIT));

		if ((flags & GPIO_PULL_UP) != 0) {
			pcr |= BIT(PCTRL_PCR_PE_BIT) | BIT(PCTRL_PCR_PS_BIT);
		} else if ((flags & GPIO_PULL_DOWN) != 0) {
			pcr |= BIT(PCTRL_PCR_PE_BIT);
		}

		sys_write32(pcr, pctrl_base + pin * PCR_REG_STRIDE);
	}

	if ((flags & GPIO_OUTPUT) != 0) {
		/* Set initial value before enabling output */
		if ((flags & GPIO_OUTPUT_INIT_HIGH) != 0) {
			gpio_reg_write(base, GPIO_PSOR_OFFSET, BIT(pin));
		} else if ((flags & GPIO_OUTPUT_INIT_LOW) != 0) {
			gpio_reg_write(base, GPIO_PCOR_OFFSET, BIT(pin));
		}

		/* Enable output */
		poer = gpio_reg_read(base, GPIO_POER_OFFSET);
		poer |= BIT(pin);
		gpio_reg_write(base, GPIO_POER_OFFSET, poer);

		/* If also input (bidirectional), enable input as well */
		pier = gpio_reg_read(base, GPIO_PIER_OFFSET);
		if ((flags & GPIO_INPUT) != 0) {
			pier |= BIT(pin);
		}
		gpio_reg_write(base, GPIO_PIER_OFFSET, pier);
	} else if ((flags & GPIO_INPUT) != 0) {
		/* Input only: disable output, enable input */
		poer = gpio_reg_read(base, GPIO_POER_OFFSET);
		poer &= ~BIT(pin);
		gpio_reg_write(base, GPIO_POER_OFFSET, poer);

		pier = gpio_reg_read(base, GPIO_PIER_OFFSET);
		pier |= BIT(pin);
		gpio_reg_write(base, GPIO_PIER_OFFSET, pier);
	}

	return 0;
}

static int gpio_ytm32_port_get_raw(const struct device *dev,
				   gpio_port_value_t *value)
{
	const struct gpio_ytm32_config *cfg = dev->config;

	*value = gpio_reg_read(cfg->base, GPIO_PDIR_OFFSET);
	return 0;
}

static int gpio_ytm32_port_set_masked_raw(const struct device *dev,
					  gpio_port_pins_t mask,
					  gpio_port_value_t value)
{
	const struct gpio_ytm32_config *cfg = dev->config;
	uintptr_t base = cfg->base;

	gpio_reg_write(base, GPIO_PSOR_OFFSET, mask & value);
	gpio_reg_write(base, GPIO_PCOR_OFFSET, mask & ~value);

	return 0;
}

static int gpio_ytm32_port_set_bits_raw(const struct device *dev,
					gpio_port_pins_t pins)
{
	const struct gpio_ytm32_config *cfg = dev->config;

	gpio_reg_write(cfg->base, GPIO_PSOR_OFFSET, pins);
	return 0;
}

static int gpio_ytm32_port_clear_bits_raw(const struct device *dev,
					  gpio_port_pins_t pins)
{
	const struct gpio_ytm32_config *cfg = dev->config;

	gpio_reg_write(cfg->base, GPIO_PCOR_OFFSET, pins);
	return 0;
}

static int gpio_ytm32_port_toggle_bits(const struct device *dev,
				       gpio_port_pins_t pins)
{
	const struct gpio_ytm32_config *cfg = dev->config;

	gpio_reg_write(cfg->base, GPIO_PTOR_OFFSET, pins);
	return 0;
}

static int gpio_ytm32_pin_interrupt_configure(const struct device *dev,
					      gpio_pin_t pin,
					      enum gpio_int_mode mode,
					      enum gpio_int_trig trig)
{
	const struct gpio_ytm32_config *cfg = dev->config;
	uintptr_t base = cfg->base;
	uint32_t pcr;
	uint32_t irqc;

	if (mode == GPIO_INT_MODE_DISABLED) {
		irqc = IRQC_DISABLED;
	} else if (mode == GPIO_INT_MODE_LEVEL) {
		if (trig == GPIO_INT_TRIG_LOW) {
			irqc = IRQC_INT_LOGIC_ZERO;
		} else if (trig == GPIO_INT_TRIG_HIGH) {
			irqc = IRQC_INT_LOGIC_ONE;
		} else {
			/* Both-level not supported */
			return -ENOTSUP;
		}
	} else {
		/* Edge mode */
		if (trig == GPIO_INT_TRIG_LOW) {
			irqc = IRQC_INT_FALLING;
		} else if (trig == GPIO_INT_TRIG_HIGH) {
			irqc = IRQC_INT_RISING;
		} else {
			irqc = IRQC_INT_EITHER;
		}
	}

	/* Write IRQC to GPIO PCR[pin] */
	pcr = sys_read32(base + GPIO_PCR_OFFSET + pin * PCR_REG_STRIDE);
	pcr &= ~GPIO_PCR_IRQC_MASK;
	pcr |= (irqc << GPIO_PCR_IRQC_SHIFT) & GPIO_PCR_IRQC_MASK;
	sys_write32(pcr, base + GPIO_PCR_OFFSET + pin * PCR_REG_STRIDE);

	/* Clear any pending flag */
	gpio_reg_write(base, GPIO_PIFR_OFFSET, BIT(pin));

	return 0;
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
	uintptr_t base = cfg->base;

	if (inputs != NULL) {
		*inputs = gpio_reg_read(base, GPIO_PIER_OFFSET) & map;
	}
	if (outputs != NULL) {
		*outputs = gpio_reg_read(base, GPIO_POER_OFFSET) & map;
	}

	return 0;
}
#endif

static void gpio_ytm32_isr(const struct device *dev)
{
	const struct gpio_ytm32_config *cfg = dev->config;
	struct gpio_ytm32_data *data = dev->data;
	uintptr_t base = cfg->base;
	uint32_t int_status;

	int_status = gpio_reg_read(base, GPIO_PIFR_OFFSET);

	/* Clear all pending flags (W1C) */
	gpio_reg_write(base, GPIO_PIFR_OFFSET, int_status);

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
