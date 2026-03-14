/*
 * Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define LOOP_OUT_NODE DT_ALIAS(loopback_out)
#define LOOP_IN_NODE  DT_ALIAS(loopback_in)

static const struct gpio_dt_spec loop_out = GPIO_DT_SPEC_GET_OR(LOOP_OUT_NODE, gpios, {0});
static const struct gpio_dt_spec loop_in = GPIO_DT_SPEC_GET_OR(LOOP_IN_NODE, gpios, {0});

static struct gpio_callback input_cb;
static struct k_sem edge_sem;
static volatile uint32_t edge_count;

static void loopback_isr(const struct device *dev, struct gpio_callback *cb,
				uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	edge_count++;
	k_sem_give(&edge_sem);
}

static int expect_level(int expected_level, const char *tag)
{
	int value = gpio_pin_get_dt(&loop_in);

	if (value < 0) {
		printk("FAIL: %s read error %d\n", tag, value);
		return value;
	}

	if (value != expected_level) {
		printk("FAIL: %s expected level %d but got %d\n",
		       tag, expected_level, value);
		return -EIO;
	}

	printk("OK: %s level=%d\n", tag, value);
	return 0;
}

static int clear_edge_sem(void)
{
	while (k_sem_take(&edge_sem, K_NO_WAIT) == 0) {
	}

	return 0;
}

static int drive_and_verify(int level, const char *tag)
{
	int ret;

	clear_edge_sem();
	ret = gpio_pin_set_dt(&loop_out, level);
	if (ret < 0) {
		printk("FAIL: %s output write error %d\n", tag, ret);
		return ret;
	}

	if (k_sem_take(&edge_sem, K_MSEC(100)) != 0) {
		printk("FAIL: %s no interrupt observed\n", tag);
		return -ETIMEDOUT;
	}

	k_msleep(2);
	return expect_level(level, tag);
}

int main(void)
{
	int ret;

	if ((loop_out.port == NULL) || (loop_in.port == NULL)) {
		printk("FAIL: missing loopback overlay for this board\n");
		return 0;
	}

	if (!gpio_is_ready_dt(&loop_out) || !gpio_is_ready_dt(&loop_in)) {
		printk("FAIL: loopback GPIO device is not ready\n");
		return 0;
	}

	k_sem_init(&edge_sem, 0, 8);

	printk("YTM32 gpio_loopback\n");
	printk("Wire PTD2 to PTD3 before running this test.\n");

	ret = gpio_pin_configure_dt(&loop_out, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		printk("FAIL: output pin configure error %d\n", ret);
		return 0;
	}

	ret = gpio_pin_configure_dt(&loop_in, GPIO_INPUT);
	if (ret < 0) {
		printk("FAIL: input pin configure error %d\n", ret);
		return 0;
	}

	gpio_init_callback(&input_cb, loopback_isr, BIT(loop_in.pin));
	ret = gpio_add_callback(loop_in.port, &input_cb);
	if (ret < 0) {
		printk("FAIL: gpio_add_callback error %d\n", ret);
		return 0;
	}

	ret = gpio_pin_interrupt_configure_dt(&loop_in, GPIO_INT_EDGE_BOTH);
	if (ret < 0) {
		printk("FAIL: interrupt configure error %d\n", ret);
		return 0;
	}

	k_msleep(5);
	ret = expect_level(0, "initial low");
	if (ret < 0) {
		return 0;
	}

	ret = drive_and_verify(1, "drive high");
	if (ret < 0) {
		return 0;
	}

	ret = drive_and_verify(0, "drive low");
	if (ret < 0) {
		return 0;
	}

	ret = drive_and_verify(1, "drive high again");
	if (ret < 0) {
		return 0;
	}

	ret = drive_and_verify(0, "drive low again");
	if (ret < 0) {
		return 0;
	}

	printk("PASS: GPIO loopback succeeded, observed %u edges\n", edge_count);

	while (1) {
		k_sleep(K_FOREVER);
	}

	return 0;
}
