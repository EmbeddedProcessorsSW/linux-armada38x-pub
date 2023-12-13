// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2023 Marvell
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#define pr_fmt(fmt) "portm-boot-cfg: " fmt

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/ctype.h>
#include <linux/io.h>
#include <linux/firmware/octeontx2/mub.h>
#include <soc/marvell/octeontx/octeontx_smc.h>

#define PORTM_MAX 28
#define DRV_MAGIC 0xa5a5a5a5
#define PLAT_OCTEONTX_PORTM_MODE_BOOT_CFG 0xc2000d0e

struct portm_boot_cfg_ctx {
	u64 status:2;     /* valid 0x2, other values invalid */
	u64 portm_idx:8;  /* PORTM index */
	u64 portm_mode:8; /* PORTM mode */
	u64 rsvd:46;
};

union mode_s {
	u32 u;
	struct {
		u16 cur;
		u16 req;
	} s;
};

enum subcmd_e {
	SUBCMD_INIT = 0,
	SUBCMD_READ,
	SUBCMD_STORE,
	SUBCMD_ERASE
};

static struct portm_boot_cfg_data {
	struct mutex lock;
	struct mub_device *mdev;
	union mode_s portm_modes[PORTM_MAX];
	size_t portm_count;
	int updated;
	void __iomem *shmem;
} portm_boot_cfg_data;

enum {
	MODE_DISABLED       = 0,   /* Port is disabled */
	MODE_INVALID        = 1,   /* Invalid port specified */
	MODE_INACTIVE       = 2,   /* Associated SERDES lane used by another Port */

	/* Ethernet modes */
	MODE_SGMII          = 3,
	MODE_1000BASE_X     = 4,
	MODE_SFI_1G         = 5,
	MODE_2500BASE_X     = 48,
	MODE_5000BASE_X     = 49,
	MODE_QSGMII         = 6,
	MODE_XFI            = 7,
	MODE_SFI            = 8,
	MODE_10GBASE_KR     = 9,
	MODE_25GAUI_C2C     = 10,
	MODE_25GAUI_C2M     = 11,
	MODE_25GBASE_USR    = 63,
	MODE_25GBASE_CR     = 12,
	MODE_25GBASE_KR     = 13,
	MODE_25GBASE_CR_C   = 14,
	MODE_25GBASE_KR_C   = 15,
	MODE_XLAUI          = 16,
	MODE_XLAUI_C2M      = 17,
	MODE_40GBASE_CR4    = 18,
	MODE_40GBASE_KR4    = 19,
	MODE_LAUI_2_C2C     = 20,
	MODE_LAUI_2_C2M     = 21,
	MODE_50GBASE_CR2_C  = 22,
	MODE_50GBASE_KR2_C  = 23,
	MODE_50GAUI_1_C2C   = 24,
	MODE_50GAUI_1_C2M   = 25,
	MODE_50GBASE_USR    = 26,
	MODE_50GBASE_CR     = 27,
	MODE_50GBASE_KR     = 28,
	MODE_CAUI_4_C2C     = 29,
	MODE_CAUI_4_C2M     = 30,
	MODE_100GBASE_CR4   = 31,
	MODE_100GBASE_KR4   = 32,
	MODE_100GAUI_2_C2C  = 33,
	MODE_100GAUI_2_C2M  = 34,
	MODE_100GBASE_USR2  = 35,
	MODE_100GBASE_CR2   = 36,
	MODE_100GBASE_KR2   = 37,
	MODE_802_3AP        = 38,

	/* USXGMII modes */
	MODE_2_5G_SXGMII    = 50,
	MODE_5G_SXGMII      = 51,
	MODE_10G_SXGMII     = 39,
	MODE_10G_DXGMII     = 52,
	MODE_10G_QXGMII     = 53,

	/* USGMII modes */
	MODE_Q_USGMII       = 54,
	MODE_O_USGMII       = 55,

	/* CPRI modes */
	MODE_CPRI_2_4G      = 40,
	MODE_CPRI_3_1G      = 41,
	MODE_CPRI_4_9G      = 42,
	MODE_CPRI_6_1G      = 43,
	MODE_CPRI_9_8G      = 44,
	MODE_CPRI_2_4G_TEST = 56,
	MODE_CPRI_3_1G_TEST = 57,
	MODE_CPRI_4_9G_TEST = 58,
	MODE_CPRI_6_1G_TEST = 59,
	MODE_CPRI_9_8G_TEST = 60,
	MODE_CPRI_12_3G_TEST = 61,
	MODE_CPRI_19_7G_TEST = 62,

	/* JESD204C modes */
	MODE_JESD204C_12_2G = 45,
	MODE_JESD204C_16_2G = 46,
	MODE_JESD204C_24_3G = 47,

	MODE_LAST           = 64,  /* Always has to be the largest number */
};

#define STR_MODE_ENTRY(_m) \
[MODE_ ## _m] = #_m
const char *str_modes[MODE_LAST] = {
	STR_MODE_ENTRY(DISABLED),
	STR_MODE_ENTRY(INVALID),
	STR_MODE_ENTRY(INACTIVE),

	STR_MODE_ENTRY(SGMII),
	STR_MODE_ENTRY(1000BASE_X),
	STR_MODE_ENTRY(SFI_1G),
	STR_MODE_ENTRY(2500BASE_X),
	STR_MODE_ENTRY(5000BASE_X),
	STR_MODE_ENTRY(QSGMII),
	STR_MODE_ENTRY(XFI),
	STR_MODE_ENTRY(SFI),
	STR_MODE_ENTRY(10GBASE_KR),
	STR_MODE_ENTRY(25GAUI_C2C),
	STR_MODE_ENTRY(25GAUI_C2M),
	STR_MODE_ENTRY(25GBASE_USR),
	STR_MODE_ENTRY(25GBASE_CR),
	STR_MODE_ENTRY(25GBASE_KR),
	STR_MODE_ENTRY(25GBASE_CR_C),
	STR_MODE_ENTRY(25GBASE_KR_C),
	STR_MODE_ENTRY(XLAUI),
	STR_MODE_ENTRY(XLAUI_C2M),
	STR_MODE_ENTRY(40GBASE_CR4),
	STR_MODE_ENTRY(40GBASE_KR4),
	STR_MODE_ENTRY(LAUI_2_C2C),
	STR_MODE_ENTRY(LAUI_2_C2M),
	STR_MODE_ENTRY(50GBASE_CR2_C),
	STR_MODE_ENTRY(50GBASE_KR2_C),
	STR_MODE_ENTRY(50GAUI_1_C2C),
	STR_MODE_ENTRY(50GAUI_1_C2M),
	STR_MODE_ENTRY(50GBASE_USR),
	STR_MODE_ENTRY(50GBASE_CR),
	STR_MODE_ENTRY(50GBASE_KR),
	STR_MODE_ENTRY(CAUI_4_C2C),
	STR_MODE_ENTRY(CAUI_4_C2M),
	STR_MODE_ENTRY(100GBASE_CR4),
	STR_MODE_ENTRY(100GBASE_KR4),
	STR_MODE_ENTRY(100GAUI_2_C2C),
	STR_MODE_ENTRY(100GAUI_2_C2M),
	STR_MODE_ENTRY(100GBASE_USR2),
	STR_MODE_ENTRY(100GBASE_CR2),
	STR_MODE_ENTRY(100GBASE_KR2),
	STR_MODE_ENTRY(802_3AP),
	STR_MODE_ENTRY(2_5G_SXGMII),
	STR_MODE_ENTRY(5G_SXGMII),
	STR_MODE_ENTRY(10G_SXGMII),
	STR_MODE_ENTRY(10G_DXGMII),
	STR_MODE_ENTRY(10G_QXGMII),
	STR_MODE_ENTRY(Q_USGMII),
	STR_MODE_ENTRY(O_USGMII),
	STR_MODE_ENTRY(CPRI_2_4G),
	STR_MODE_ENTRY(CPRI_3_1G),
	STR_MODE_ENTRY(CPRI_4_9G),
	STR_MODE_ENTRY(CPRI_6_1G),
	STR_MODE_ENTRY(CPRI_9_8G),
	STR_MODE_ENTRY(CPRI_2_4G_TEST),
	STR_MODE_ENTRY(CPRI_3_1G_TEST),
	STR_MODE_ENTRY(CPRI_4_9G_TEST),
	STR_MODE_ENTRY(CPRI_6_1G_TEST),
	STR_MODE_ENTRY(CPRI_9_8G_TEST),
	STR_MODE_ENTRY(CPRI_12_3G_TEST),
	STR_MODE_ENTRY(CPRI_19_7G_TEST),
	STR_MODE_ENTRY(JESD204C_12_2G),
	STR_MODE_ENTRY(JESD204C_16_2G),
	STR_MODE_ENTRY(JESD204C_24_3G),
};

static inline int str2mode(const char *str)
{
	int idx;

	for (idx = 0; idx < MODE_LAST; idx++) {
		if (!strcmp(str_modes[idx], str))
			return idx;
	}

	return -1;
}

static inline const char *mode2str(int mode)
{
	if (mode >= 0 && mode < MODE_LAST)
		return str_modes[mode];

	return NULL;
}

static inline u32 _attr_to_num(struct device_attribute *attr)
{
	struct dev_ext_attribute *eattr;

	eattr = container_of(attr, struct dev_ext_attribute, attr);
	return (u32)(unsigned long)eattr->var;
}

static ssize_t port_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	u32 num;
	int mode;
	char *s, str[32];
	size_t len;
	struct mub_device *mdev = container_of(dev, struct mub_device, dev);
	struct portm_boot_cfg_data *data = mub_get_data(mdev);

	len = count < 32 ? count : 32 - 1;
	strncpy(str, buf, len);
	str[len] = '\0';
	s = strchr(str, '\n');
	if (s)
		*s = '\0';

	num = _attr_to_num(attr);
	mode = str2mode(str);

	if (mode != -1) {
		mutex_lock(&data->lock);
		data->portm_modes[num].s.req = mode;
		data->updated = 1;
		mutex_unlock(&data->lock);

		return count;
	}

	return -1;
}

static ssize_t port_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u32 num;
	union mode_s mode;
	struct mub_device *mdev = container_of(dev, struct mub_device, dev);
	struct portm_boot_cfg_data *data = mub_get_data(mdev);
	int cnt = 0;

	num = _attr_to_num(attr);

	mutex_lock(&data->lock);
	mode = data->portm_modes[num];
	mutex_unlock(&data->lock);

	cnt += scnprintf(buf + cnt, PAGE_SIZE, "%s",
		mode.s.cur != (u16)-1 ? mode2str(mode.s.cur) : "-");

	if (mode.s.req != (u16)-1)
		cnt += scnprintf(buf + cnt, PAGE_SIZE, " -> %s\n", mode2str(mode.s.req));
	else
		cnt += scnprintf(buf + cnt, PAGE_SIZE, "\n");

	return cnt;
}

static void copy_shared2current(struct portm_boot_cfg_data *data)
{
	int idx;
	const int count = data->portm_count;
	struct portm_boot_cfg_ctx *shared = (struct portm_boot_cfg_ctx *)data->shmem;

	for (idx = 0; idx < count; idx++) {
		struct portm_boot_cfg_ctx *sh_ptr = &shared[idx];
		union mode_s *mode = &data->portm_modes[idx];

		if (sh_ptr->status == 0x2 && sh_ptr->portm_idx == idx)
			mode->s.cur = sh_ptr->portm_mode;
		else
			mode->s.cur = (u16)-1;
	}
}

static void copy_requested2shared(struct portm_boot_cfg_data *data)
{
	int idx;
	const int count = data->portm_count;
	struct portm_boot_cfg_ctx *shared = (struct portm_boot_cfg_ctx *)data->shmem;

	for (idx = 0; idx < count; idx++) {
		struct portm_boot_cfg_ctx *sh_ptr = &shared[idx];
		union mode_s *mode = &data->portm_modes[idx];

		if (mode->s.req != (u16)-1) {
			sh_ptr->status = 0x2;
			sh_ptr->portm_idx = idx;
			sh_ptr->portm_mode = mode->s.req;
			mode->s.req = (u16)-1;
		} else
			sh_ptr->status = 0x3; //set status to invalid
	}
}

static void clear_local_data(struct portm_boot_cfg_data *data)
{
	int idx;
	const int count = data->portm_count;

	for (idx = 0; idx < count; idx++) {
		union mode_s *mode = &data->portm_modes[idx];

		mode->u = (u32)-1;
	}

	data->updated = 0;
}

static ssize_t update_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct arm_smccc_res res;
	struct mub_device *mdev = container_of(dev, struct mub_device, dev);
	struct portm_boot_cfg_data *data = mub_get_data(mdev);
	int subcmd;

	mutex_lock(&data->lock);
	if (data->updated) {
		subcmd = SUBCMD_STORE;
		copy_requested2shared(data);
		data->updated = 0;
	} else
		subcmd = SUBCMD_READ;

	arm_smccc_smc(PLAT_OCTEONTX_PORTM_MODE_BOOT_CFG, subcmd, 0, 0, 0, 0, 0, 0, &res);

	if (res.a0 != SMCCC_RET_SUCCESS) {
		mutex_unlock(&data->lock);
		return -EINVAL;
	}

	copy_shared2current(data);
	mutex_unlock(&data->lock);

	return count;
}

static ssize_t erase_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct arm_smccc_res res;
	struct mub_device *mdev = container_of(dev, struct mub_device, dev);
	struct portm_boot_cfg_data *data = mub_get_data(mdev);

	mutex_lock(&data->lock);

	arm_smccc_smc(PLAT_OCTEONTX_PORTM_MODE_BOOT_CFG, SUBCMD_ERASE, 0, 0, 0, 0, 0, 0, &res);

	if (res.a0 != SMCCC_RET_SUCCESS) {
		mutex_unlock(&data->lock);
		return -EINVAL;
	}

	clear_local_data(data);
	mutex_unlock(&data->lock);
	return count;
}

umode_t port_is_visible(struct kobject *kobj, struct attribute *a, int n)
{
	struct device_attribute *dattr;
	u32 num;

	dattr = container_of(a, struct device_attribute, attr);
	num = _attr_to_num(dattr);

	if (num < portm_boot_cfg_data.portm_count)
		return a->mode;

	return 0;
}

#define port_dev_attr(_name, _num)					\
	(&((struct dev_ext_attribute[]) {				\
	   {								\
		__ATTR(_name ## _num, 0644, port_show, port_store),	\
		(void *)(unsigned long)_num				\
	   }								\
	})[0].attr.attr)

static struct attribute *port_attrs[] = {
	port_dev_attr(PORTM, 0),
	port_dev_attr(PORTM, 1),
	port_dev_attr(PORTM, 2),
	port_dev_attr(PORTM, 3),
	port_dev_attr(PORTM, 4),
	port_dev_attr(PORTM, 5),
	port_dev_attr(PORTM, 6),
	port_dev_attr(PORTM, 7),
	port_dev_attr(PORTM, 8),
	port_dev_attr(PORTM, 9),
	port_dev_attr(PORTM, 10),
	port_dev_attr(PORTM, 11),
	port_dev_attr(PORTM, 12),
	port_dev_attr(PORTM, 13),
	port_dev_attr(PORTM, 14),
	port_dev_attr(PORTM, 15),
	port_dev_attr(PORTM, 16),
	port_dev_attr(PORTM, 17),
	port_dev_attr(PORTM, 18),
	port_dev_attr(PORTM, 19),
	port_dev_attr(PORTM, 20),
	port_dev_attr(PORTM, 21),
	port_dev_attr(PORTM, 22),
	port_dev_attr(PORTM, 23),
	port_dev_attr(PORTM, 24),
	port_dev_attr(PORTM, 25),
	port_dev_attr(PORTM, 26),
	port_dev_attr(PORTM, 27),
	NULL
};

static struct attribute_group port_group = {
	.attrs = port_attrs,
	.is_visible = port_is_visible,
};

static DEVICE_ATTR_WO(update);
static DEVICE_ATTR_WO(erase);
static struct attribute *common_attrs[] = {
	&dev_attr_update.attr,
	&dev_attr_erase.attr,
	NULL,
};

static struct attribute_group common_grp = {
	.attrs = common_attrs,
};

static const struct attribute_group *portm_boot_cfg_groups[] = {
	&port_group,
	&common_grp,
	NULL
};

static int __init portm_boot_cfg_init(void)
{
	struct mub_device *mdev;
	struct arm_smccc_res res;

	if (octeontx_soc_check_smc() != 2) {
		pr_debug("Not supported\n");
		return -EPERM;
	}
	arm_smccc_smc(PLAT_OCTEONTX_PORTM_MODE_BOOT_CFG, SUBCMD_INIT, 0, 0, 0, 0, 0, 0, &res);
	if (res.a0 != SMCCC_RET_SUCCESS)
		return -ENOMEM;

	if (res.a3 != DRV_MAGIC || res.a1 == 0 || res.a2 == 0 || res.a2 > PORTM_MAX)
		return -EINVAL;

	portm_boot_cfg_data.portm_count = res.a2;
	clear_local_data(&portm_boot_cfg_data);
	portm_boot_cfg_data.shmem = ioremap_wc(res.a1,
		portm_boot_cfg_data.portm_count * sizeof(struct portm_boot_cfg_ctx));
	mutex_init(&portm_boot_cfg_data.lock);

	mdev = mub_device_register("portm-boot-cfg",
				    MUB_SOC_TYPE_10X,
				    portm_boot_cfg_groups);
	if (IS_ERR(mdev))
		return PTR_ERR(mdev);

	portm_boot_cfg_data.mdev = mdev;
	mub_set_data(mdev, &portm_boot_cfg_data);

	return 0;
}
module_init(portm_boot_cfg_init);

static void __exit portm_boot_cfg_exit(void)
{
	mub_device_unregister(portm_boot_cfg_data.mdev);
	iounmap(portm_boot_cfg_data.shmem);
}
module_exit(portm_boot_cfg_exit);

MODULE_DESCRIPTION("Marvell CN10K PORTM boot configuration sysfs interface");
MODULE_AUTHOR("Damian Eppel <deppel@marvell.com>");
MODULE_LICENSE("GPL");
