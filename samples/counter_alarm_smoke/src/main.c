/*
 * Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define COUNTER_NODE DT_ALIAS(counter)
#define ALARM_CHANNEL 0U
#define ALARM_DELAY_US 500000U
#define ALARM_COUNT_TARGET 3U

#if !DT_NODE_HAS_STATUS(COUNTER_NODE, okay)
#error "Missing counter alias"
#endif

static const struct device *const counter_dev = DEVICE_DT_GET(COUNTER_NODE);
static struct k_sem alarm_sem;
static volatile uint32_t alarm_hits;

static void alarm_callback(const struct device *dev, uint8_t chan_id,
			   uint32_t ticks, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(chan_id);
	ARG_UNUSED(user_data);

	alarm_hits++;
	printk("Alarm %u fired at tick %u\n", alarm_hits, ticks);
	k_sem_give(&alarm_sem);
}

static int schedule_alarm(void)
{
	struct counter_alarm_cfg alarm_cfg = {
		.callback = alarm_callback,
		.user_data = NULL,
		.flags = 0U,
	};

	alarm_cfg.ticks = counter_us_to_ticks(counter_dev, ALARM_DELAY_US);
	if (alarm_cfg.ticks == 0U) {
		alarm_cfg.ticks = 1U;
	}

	return counter_set_channel_alarm(counter_dev, ALARM_CHANNEL, &alarm_cfg);
}

int main(void)
{
	int ret;
	uint32_t freq;

	if (!device_is_ready(counter_dev)) {
		printk("FAIL: counter device is not ready\n");
		return 0;
	}

	k_sem_init(&alarm_sem, 0, ALARM_COUNT_TARGET);
	freq = counter_get_frequency(counter_dev);

	printk("YTM32 counter_alarm_smoke\n");
	printk("Device: %s, freq=%u Hz, top=%u\n",
	       counter_dev->name, freq, counter_get_top_value(counter_dev));

	ret = counter_start(counter_dev);
	if ((ret < 0) && (ret != -EALREADY)) {
		printk("FAIL: counter_start error %d\n", ret);
		return 0;
	}

	for (uint32_t i = 0U; i < ALARM_COUNT_TARGET; i++) {
		ret = schedule_alarm();
		if (ret < 0) {
			printk("FAIL: counter_set_channel_alarm error %d\n", ret);
			return 0;
		}

		if (k_sem_take(&alarm_sem, K_SECONDS(2)) != 0) {
			printk("FAIL: timeout waiting for alarm %u\n", i + 1U);
			return 0;
		}
	}

	ret = counter_stop(counter_dev);
	if (ret < 0) {
		printk("FAIL: counter_stop error %d\n", ret);
		return 0;
	}

	printk("PASS: counter alarm fired %u times\n", alarm_hits);

	while (1) {
		k_sleep(K_FOREVER);
	}

	return 0;
}
