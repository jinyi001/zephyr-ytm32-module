/*
 * Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Zephyr flash driver for the YTMicro YTM32 EFM, implemented as a thin wrapper
 * over the vendor FLASH_DRV_* API.
 *
 * MPU note: on YTM32B1MD1 the EFM has no hardware write-enable bit, so the
 * vendor program/erase path temporarily disables the whole Cortex-M MPU
 * (inside its own __disable_irq() critical section) to let the CPU write the
 * flash-mapped destination words, then restores MPU_CTRL = ENABLE|PRIVDEFENA
 * (the same value Zephyr uses). It never reloads MPU regions, so Zephyr's
 * static/dynamic regions survive. Because that window is IRQ-masked, do NOT
 * call flash write/erase from real-time control paths (e.g. the FOC loop).
 * See BRINGUP_RESOURCE_OWNERSHIP.md (C-MPU-EFMINIT-01).
 */

#define DT_DRV_COMPAT ytmicro_ytm32_flash_controller

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "flash_driver.h" /* vendor SDK: FLASH_DRV_*, status_t, STATUS_SUCCESS */

LOG_MODULE_REGISTER(flash_ytm32, CONFIG_FLASH_LOG_LEVEL);

#define SOC_NV_FLASH_NODE	DT_INST(0, soc_nv_flash)

#define FLASH_YTM32_BASE	DT_REG_ADDR(SOC_NV_FLASH_NODE)
#define FLASH_YTM32_SIZE	DT_REG_SIZE(SOC_NV_FLASH_NODE)
#define FLASH_YTM32_SECTOR	DT_PROP(SOC_NV_FLASH_NODE, erase_block_size)
#define FLASH_YTM32_WRITE_BLK	DT_PROP(SOC_NV_FLASH_NODE, write_block_size)

/* The vendor driver addresses the single EFM block by instance index. */
#define FLASH_YTM32_INSTANCE	0U

struct flash_ytm32_data {
	struct k_mutex lock;
	flash_state_t state;
};

static const struct flash_parameters flash_ytm32_parameters = {
	.write_block_size = FLASH_YTM32_WRITE_BLK,
	.erase_value = 0xFF,
};

static bool flash_ytm32_range_ok(off_t offset, size_t len)
{
	return (offset >= 0) && (((uint64_t)offset + len) <= FLASH_YTM32_SIZE);
}

static int flash_ytm32_read(const struct device *dev, off_t offset,
			    void *data, size_t len)
{
	ARG_UNUSED(dev);

	if (len == 0U) {
		return 0;
	}
	if (!flash_ytm32_range_ok(offset, len)) {
		return -EINVAL;
	}

	memcpy(data, (const uint8_t *)(FLASH_YTM32_BASE + offset), len);
	return 0;
}

static int flash_ytm32_write(const struct device *dev, off_t offset,
			     const void *data, size_t len)
{
	struct flash_ytm32_data *dd = dev->data;
	status_t st;

	if (len == 0U) {
		return 0;
	}
	if (!flash_ytm32_range_ok(offset, len)) {
		return -EINVAL;
	}
	if (((offset % FLASH_YTM32_WRITE_BLK) != 0) ||
	    ((len % FLASH_YTM32_WRITE_BLK) != 0)) {
		return -EINVAL;
	}

	k_mutex_lock(&dd->lock, K_FOREVER);
	st = FLASH_DRV_Program(FLASH_YTM32_INSTANCE,
			       (uint32_t)(FLASH_YTM32_BASE + offset),
			       (uint32_t)len, data);
	k_mutex_unlock(&dd->lock);

	return (st == STATUS_SUCCESS) ? 0 : -EIO;
}

static int flash_ytm32_erase(const struct device *dev, off_t offset, size_t size)
{
	struct flash_ytm32_data *dd = dev->data;
	status_t st;

	if (size == 0U) {
		return 0;
	}
	if (!flash_ytm32_range_ok(offset, size)) {
		return -EINVAL;
	}
	if (((offset % FLASH_YTM32_SECTOR) != 0) ||
	    ((size % FLASH_YTM32_SECTOR) != 0)) {
		return -EINVAL;
	}

	k_mutex_lock(&dd->lock, K_FOREVER);
	st = FLASH_DRV_EraseSector(FLASH_YTM32_INSTANCE,
				   (uint32_t)(FLASH_YTM32_BASE + offset),
				   (uint32_t)size);
	k_mutex_unlock(&dd->lock);

	return (st == STATUS_SUCCESS) ? 0 : -EIO;
}

static const struct flash_parameters *
flash_ytm32_get_parameters(const struct device *dev)
{
	ARG_UNUSED(dev);
	return &flash_ytm32_parameters;
}

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
static const struct flash_pages_layout flash_ytm32_layout = {
	.pages_count = FLASH_YTM32_SIZE / FLASH_YTM32_SECTOR,
	.pages_size = FLASH_YTM32_SECTOR,
};

static void flash_ytm32_page_layout(const struct device *dev,
				    const struct flash_pages_layout **layout,
				    size_t *layout_size)
{
	ARG_UNUSED(dev);
	*layout = &flash_ytm32_layout;
	*layout_size = 1U;
}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

static DEVICE_API(flash, flash_ytm32_api) = {
	.read = flash_ytm32_read,
	.write = flash_ytm32_write,
	.erase = flash_ytm32_erase,
	.get_parameters = flash_ytm32_get_parameters,
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	.page_layout = flash_ytm32_page_layout,
#endif
};

static int flash_ytm32_init(const struct device *dev)
{
	struct flash_ytm32_data *dd = dev->data;
	flash_user_config_t cfg;

	k_mutex_init(&dd->lock);

	FLASH_DRV_GetDefaultConfig(&cfg);
	cfg.async = false; /* synchronous (polled) completion */

	if (FLASH_DRV_Init(FLASH_YTM32_INSTANCE, &cfg, &dd->state) != STATUS_SUCCESS) {
		LOG_ERR("FLASH_DRV_Init failed");
		return -EIO;
	}

	return 0;
}

static struct flash_ytm32_data flash_ytm32_data0;

DEVICE_DT_INST_DEFINE(0, flash_ytm32_init, NULL,
		      &flash_ytm32_data0, NULL,
		      POST_KERNEL, CONFIG_FLASH_INIT_PRIORITY,
		      &flash_ytm32_api);
