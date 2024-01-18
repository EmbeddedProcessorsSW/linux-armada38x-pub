// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2017 Cavium, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License
 * as published by the Free Software Foundation.
 */

#include <linux/types.h>
#include <linux/stat.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/delay.h>

#include "rst.h"

#define DRV_NAME "octeontx-rst"
#define DRV_VERSION "1.0"

static atomic_t rst_count = ATOMIC_INIT(0);
static DEFINE_MUTEX(octeontx_rst_devices_lock);
static LIST_HEAD(octeontx_rst_devices);

struct rstpf {
	struct pci_dev		*pdev;
	void __iomem		*reg_base;
	int			id;
	struct list_head	list;
};

/* In Cavium OcteonTX SoCs, all accesses to the device registers are
 * implicitly strongly ordered.
 * So writeq_relaxed() and readq_relaxed() are safe to use
 * with out any memory barriers.
 */

/* Register read/write APIs */
static inline void rst_reg_write(struct rstpf *rst, u64 offset, u64 val)
{
	writeq_relaxed(val, rst->reg_base + offset);
}

static inline u64 rst_reg_read(struct rstpf *rst, u64 offset)
{
	return readq_relaxed(rst->reg_base + offset);
}

static struct rstpf *rst_get(u32 id)
{
	struct rstpf *rst = NULL;
	struct rstpf *curr;

	mutex_lock(&octeontx_rst_devices_lock);
	list_for_each_entry(curr, &octeontx_rst_devices, list) {
		if (curr->id == id) {
			rst = curr;
			break;
		}
	}

	if (!rst) {
		mutex_unlock(&octeontx_rst_devices_lock);
		return NULL;
	}

	mutex_unlock(&octeontx_rst_devices_lock);
	return rst;
}

static u64 rst_get_sclk_freq(int node)
{
	u64 sclk_freq;
	struct rstpf *rst = NULL;

	rst = rst_get(node);
	if (!rst)
		return 0;

	/* Bit 38:33 is PNR_MULL */
	sclk_freq = (rst_reg_read(rst, RST_BOOT) >> 33) & 0x3f;
	sclk_freq *= PLL_REF_CLK;

	return sclk_freq;
}

static u64 rst_get_rclk_freq(int node)
{
	u64 rclk_freq;
	struct rstpf *rst = NULL;

	rst = rst_get(node);
	if (!rst)
		return 0;

	/* Bit 46:40 is C_MULL */
	rclk_freq = (rst_reg_read(rst, RST_BOOT) >> 40) & 0x3f;
	rclk_freq *= PLL_REF_CLK;

	return rclk_freq;
}

struct rst_com_s rst_com = {
	.get_sclk_freq = rst_get_sclk_freq,
	.get_rclk_freq = rst_get_rclk_freq
};
EXPORT_SYMBOL_GPL(rst_com);

static int rst_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	struct device *dev = &pdev->dev;
	struct rstpf *rst;
	int err = -ENOMEM;

	rst = devm_kzalloc(dev, sizeof(*rst), GFP_KERNEL);
	if (!rst)
		return err;

	pci_set_drvdata(pdev, rst);
	rst->pdev = pdev;

	err = pcim_enable_device(pdev);
	if (err) {
		dev_err(dev, "Failed to enable PCI device\n");
		return err;
	}

	err = pci_request_regions(pdev, DRV_NAME);
	if (err) {
		dev_err(dev, "PCI request regions failed\n");
		return err;
	}

	/* Map CFG registers */
	rst->reg_base = pcim_iomap(pdev, PCI_RST_PF_CFG_BAR, 0);
	if (!rst->reg_base) {
		dev_err(dev, "Can't map CFG space\n");
		err = -ENOMEM;
		return err;
	}

	/* set RST ID */
	rst->id = atomic_add_return(1, &rst_count);
	rst->id -= 1;

	INIT_LIST_HEAD(&rst->list);

	/* use sso_device_lock; as rst use-case scope limited till sso */
	mutex_lock(&octeontx_rst_devices_lock);
	list_add(&rst->list, &octeontx_rst_devices);
	mutex_unlock(&octeontx_rst_devices_lock);

	return 0;
}

static void rst_remove(struct pci_dev *pdev)
{
	struct device *dev = &pdev->dev;
	struct rstpf *rst = pci_get_drvdata(pdev);

	if (!rst)
		return;

	/* use sso_device_lock; as rst use-case scope limited till sso */
	mutex_lock(&octeontx_rst_devices_lock);
	list_del(&rst->list);
	mutex_unlock(&octeontx_rst_devices_lock);
	pci_release_regions(pdev);
	pci_disable_device(pdev);
	pci_set_drvdata(pdev, NULL);
	devm_kfree(dev, rst);
}

/* devices supported */
static const struct pci_device_id rst_id_table[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_CAVIUM, PCI_DEVICE_ID_OCTEONTX_RST_PF) },
	{ 0, } /* end of table */
};

static struct pci_driver rst_driver = {
	.name = DRV_NAME,
	.id_table = rst_id_table,
	.probe = rst_probe,
	.remove = rst_remove,
	.sriov_configure = NULL,
};

MODULE_AUTHOR("Santosh Shukla");
MODULE_DESCRIPTION("Cavium OCTEONTX RST Physical Function Driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRV_VERSION);
MODULE_DEVICE_TABLE(pci, rst_id_table);

static int __init rst_init_module(void)
{
	pr_info("%s, ver %s\n", DRV_NAME, DRV_VERSION);

	return pci_register_driver(&rst_driver);
}

static void __exit rst_cleanup_module(void)
{
	pci_unregister_driver(&rst_driver);
}

module_init(rst_init_module);
module_exit(rst_cleanup_module);

