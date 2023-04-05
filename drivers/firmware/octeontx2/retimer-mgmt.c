// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2023 Marvell
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#define pr_fmt(fmt) "retimer-mgmt: " fmt

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/ctype.h>
#include <linux/io.h>
#include <linux/firmware/octeontx2/mub.h>

#define RETIMER_MUX_CNT 4
#define PLAT_OCTEONTX_CONFIG_RETIMER 0xc2000d0d

struct mux_cfg {
	u32 gserm:3;
	u32 gserm_updated:1;
	u32 mode:7;
	u32 mode_updated:1;
	u32 _rsrvd:20;
};

static struct rtmr_mgmt_data {
	spinlock_t lock;
	struct mub_device *mdev;
	struct mux_cfg cfg[RETIMER_MUX_CNT];
} rtmr_mgmt_data;

static inline u32 _attr_to_num(struct device_attribute *attr)
{
	struct dev_ext_attribute *eattr;

	eattr = container_of(attr, struct dev_ext_attribute, attr);
	return (u32)(unsigned long)eattr->var;
}

static ssize_t gserm_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	u32 gserm, num;
	struct mub_device *mdev = container_of(dev, struct mub_device, dev);
	struct rtmr_mgmt_data *data = mub_get_data(mdev);

	num = _attr_to_num(attr);
	if (kstrtou32(buf, 10, &gserm))
		return -EINVAL;

	spin_lock(&data->lock);
	data->cfg[num].gserm = gserm;
	data->cfg[num].gserm_updated = 1;
	spin_unlock(&data->lock);

	return count;
}

static ssize_t gserm_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u32 num, gserm;
	bool updated = false;
	struct mub_device *mdev = container_of(dev, struct mub_device, dev);
	struct rtmr_mgmt_data *data = mub_get_data(mdev);

	num = _attr_to_num(attr);

	spin_lock(&data->lock);
	if (data->cfg[num].gserm_updated) {
		gserm = data->cfg[num].gserm;
		updated = true;
	}
	spin_unlock(&data->lock);

	return (updated) ?
		scnprintf(buf, PAGE_SIZE, "%u\n", gserm) :
		scnprintf(buf, PAGE_SIZE, "not updated\n");
}

static ssize_t mode_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	u32 mode, num;
	struct mub_device *mdev = container_of(dev, struct mub_device, dev);
	struct rtmr_mgmt_data *data = mub_get_data(mdev);

	num = _attr_to_num(attr);
	if (kstrtou32(buf, 10, &mode))
		return -EINVAL;

	spin_lock(&data->lock);
	data->cfg[num].mode = mode;
	data->cfg[num].mode_updated = 1;
	spin_unlock(&data->lock);

	return count;
}

static ssize_t mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u32 num, mode;
	bool updated = false;
	struct mub_device *mdev = container_of(dev, struct mub_device, dev);
	struct rtmr_mgmt_data *data = mub_get_data(mdev);

	num = _attr_to_num(attr);

	spin_lock(&data->lock);
	if (data->cfg[num].mode_updated) {
		mode = data->cfg[num].mode;
		updated = true;
	}
	spin_unlock(&data->lock);

	return (updated) ?
		scnprintf(buf, PAGE_SIZE, "%u\n", mode) :
		scnprintf(buf, PAGE_SIZE, "not updated\n");
}

static ssize_t update_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct arm_smccc_res res;
	u32 update, x[4] = {0};
	int i;
	struct mub_device *mdev = container_of(dev, struct mub_device, dev);
	struct rtmr_mgmt_data *data = mub_get_data(mdev);

	if (kstrtou32(buf, 10, &update))
		return -EINVAL;

	if (update != 1)
		return count;

	spin_lock(&data->lock);
	for (i = 0; i < RETIMER_MUX_CNT; i++) {
		struct mux_cfg *cfg = &data->cfg[i];
		struct mux_cfg *reg = (struct mux_cfg *)&x[i];

		if (cfg->gserm_updated) {
			reg->gserm = cfg->gserm;
			reg->gserm_updated = 1;
			cfg->gserm_updated = 0;
		}

		if (cfg->mode_updated) {
			reg->mode = cfg->mode;
			reg->mode_updated = 1;
			cfg->mode_updated = 0;
		}
	}
	spin_unlock(&data->lock);

	arm_smccc_smc(PLAT_OCTEONTX_CONFIG_RETIMER, x[0], x[1],
		x[2], x[3], 0, 0, 0, &res);

	if (res.a0 != SMCCC_RET_SUCCESS) {
		pr_err("Sending Retimer config failed\n");
		return -EINVAL;
	}

	return count;
}

#define retimer_dev_attr(_name, _num)						\
	(&((struct dev_ext_attribute[]) {					\
	   {									\
		__ATTR(_name, 0644, _name##_show, _name##_store),		\
		(void *)(unsigned long)_num					\
	   }									\
	})[0].attr.attr)

#define retimer_attrs(_name, _num)						\
	static struct attribute *_name##_attrs##_num[] = {			\
		retimer_dev_attr(gserm, _num),					\
		retimer_dev_attr(mode, _num),					\
		NULL								\
	}

#define retimer_grp(_name, _num)						\
	retimer_attrs(_name, _num);						\
	static struct attribute_group _name##_group##_num = {			\
		.name = #_name #_num,						\
		.attrs = _name##_attrs##_num,					\
	}

retimer_grp(rtmr, 0);
retimer_grp(rtmr, 1);
retimer_grp(rtmr, 2);
retimer_grp(rtmr, 3);

DEVICE_ATTR_WO(update);
static struct attribute *common_attrs[] = {
	&dev_attr_update.attr,
	NULL,
};

static struct attribute_group common_grp = {
	.attrs = common_attrs,
};

static const struct attribute_group *rtmr_mgmt_groups[] = {
	&rtmr_group0,
	&rtmr_group1,
	&rtmr_group2,
	&rtmr_group3,
	&common_grp,
	NULL
};

static int __init rtmr_mgmt_init(void)
{
	int ret = 0;
	struct mub_device *mdev;

	spin_lock_init(&rtmr_mgmt_data.lock);
	mdev = mub_device_register("retimer-mgmt",
				    MUB_SOC_TYPE_10X,
				    rtmr_mgmt_groups);
	if (IS_ERR(mdev))
		return PTR_ERR(mdev);

	rtmr_mgmt_data.mdev = mdev;
	mub_set_data(mdev, &rtmr_mgmt_data);
	return ret;
}
module_init(rtmr_mgmt_init);

static void __exit rtmr_mgmt_exit(void)
{
	mub_device_unregister(rtmr_mgmt_data.mdev);
}
module_exit(rtmr_mgmt_exit);

MODULE_DESCRIPTION("Marvell CN10K Retimer management sysfs interface");
MODULE_AUTHOR("Damian Eppel <deppel@marvell.com>");
MODULE_LICENSE("GPL");
