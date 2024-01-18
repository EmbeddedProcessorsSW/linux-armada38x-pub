// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2024 Marvell
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#define pr_fmt(fmt) "led-blink-rate: " fmt

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/ctype.h>
#include <linux/io.h>
#include <linux/firmware/octeontx2/mub.h>
#include <soc/marvell/octeontx/octeontx_smc.h>

#define PLAT_OCTEONTX_LED_BLINK_RATE            0xc2000d0f
#define GET_LED_BLINK_RATE			1
#define SET_LED_BLINK_RATE			2

static struct blink_rate_data {
	struct mutex lock;
	struct mub_device *mdev;
} blink_rate_data;

static ssize_t blink_rate_show(struct mub_device *mdev, char *buf)
{
	struct blink_rate_data *data = mub_get_data(mdev);
	struct arm_smccc_res res;
	int cnt = 0;

	mutex_lock(&data->lock);
	arm_smccc_smc(PLAT_OCTEONTX_LED_BLINK_RATE, GET_LED_BLINK_RATE, 0, 0, 0, 0, 0, 0, &res);
	if (res.a0 != SMCCC_RET_SUCCESS) {
		mutex_unlock(&data->lock);
		return -EINVAL;
	}

	cnt += scnprintf(buf + cnt, PAGE_SIZE, "%lu", res.a1);
	mutex_unlock(&data->lock);

	return cnt;
}

static ssize_t blink_rate_store(struct mub_device *mdev,
				const char *buf, size_t cnt)
{
	struct blink_rate_data *data = mub_get_data(mdev);
	u32 rate;
	struct arm_smccc_res res;

	if (kstrtou32(buf, 10, &rate))
		return -EINVAL;

	mutex_lock(&data->lock);
	arm_smccc_smc(PLAT_OCTEONTX_LED_BLINK_RATE, SET_LED_BLINK_RATE, rate, 0, 0, 0, 0, 0, &res);

	if (res.a0 != SMCCC_RET_SUCCESS) {
		mutex_unlock(&data->lock);
		return -EINVAL;
	}

	mutex_unlock(&data->lock);
	return cnt;
}
MUB_ATTR_RW(blink_rate, blink_rate_show, blink_rate_store);

static struct attribute *blink_rate_attrs[] = {
	MUB_TO_ATTR(blink_rate),
	NULL,
};

static const struct attribute_group blink_rate_group = {
	.attrs = blink_rate_attrs,
};

__ATTRIBUTE_GROUPS(blink_rate);

static int __init led_blink_rate_init(void)
{
	struct mub_device *mdev;

	if (octeontx_soc_check_smc() != 2)
		return -EPERM;

	mutex_init(&blink_rate_data.lock);

	mdev = mub_device_register("led-blink-rate",
				MUB_SOC_TYPE_10X,
				blink_rate_groups);
	if (IS_ERR(mdev))
		return PTR_ERR(mdev);

	blink_rate_data.mdev = mdev;
	mub_set_data(mdev, &blink_rate_data);

	return 0;
}
module_init(led_blink_rate_init);

static void __exit led_blink_rate_exit(void)
{
	mub_device_unregister(blink_rate_data.mdev);
}
module_exit(led_blink_rate_exit);

MODULE_DESCRIPTION("Marvell CN10K LED blink rate configuration sysfs interface");
MODULE_AUTHOR("Scott Rowberry <rowberry@marvell.com>");
MODULE_LICENSE("GPL");
