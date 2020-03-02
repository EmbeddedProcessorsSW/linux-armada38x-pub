// SPDX-License-Identifier: GPL-2.0
/* Marvell Loki driver
 *
 * Copyright (C) 2018 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/pci.h>

#define PCI_DEVICE_ID_BPHY	0xA089

struct mrvl_loki {
	void __iomem *msix;
	struct pci_dev *pdev;
	struct msix_entry msix_ent;
	int intr_num;
};

static irqreturn_t mrvl_loki_handler(int irq, void *dev)
{
	return IRQ_HANDLED;
};

static inline void msix_enable_ctrl(struct pci_dev *dev)
{
	u16 control;

	pci_read_config_word(dev, dev->msix_cap + PCI_MSIX_FLAGS, &control);
	control |= PCI_MSIX_FLAGS_ENABLE;
	pci_write_config_word(dev, dev->msix_cap + PCI_MSIX_FLAGS, control);
}

static int mrvl_loki_probe(struct platform_device *pdev)
{
	struct mrvl_loki *ml;
	struct device *dev = &pdev->dev;
	struct pci_dev *bphy_pdev;
	uint64_t regval;
	int ret = 0;

	ml = devm_kzalloc(dev, sizeof(*ml), GFP_KERNEL);
	if (!ml)
		return -ENOMEM;

	platform_set_drvdata(pdev, ml);

	/*
	 * BPHY is a PCI device and the kernel resets the MSIXEN bit during
	 * enumeration. So enable it back for interrupts to be generated.
	 */
	bphy_pdev = pci_get_device(PCI_VENDOR_ID_CAVIUM, PCI_DEVICE_ID_BPHY,
				  NULL);
	if (!bphy_pdev) {
		dev_err(dev, "Couldn't find BPHY PCI device %x\n",
			PCI_DEVICE_ID_BPHY);
		ret = -ENODEV;
		goto err;
	}

	ml->pdev = bphy_pdev;
	ml->msix_ent.entry = 0;

	msix_enable_ctrl(bphy_pdev);

	/* register interrupt */
	ml->intr_num = irq_of_parse_and_map(dev->of_node, 0);

	if (request_irq(ml->intr_num, mrvl_loki_handler, 0,
			"mrvl loki handler", pdev)) {
		dev_err(dev, "failed to register irq handler\n");
		ret = -ENOMEM;
		goto err;
	}

	dev_info(dev, "Registered interrupt handler for %d\n", ml->intr_num);

	return 0;

err:
	devm_kfree(&pdev->dev, ml);
	return ret;
}

static int mrvl_loki_remove(struct platform_device *pdev)
{
	struct mrvl_loki *ml = platform_get_drvdata(pdev);

	free_irq(ml->intr_num, pdev);
	devm_kfree(&pdev->dev, ml);

	return 0;
}

static const struct of_device_id mrvl_loki_of_match[] = {
	{ .compatible = "marvell,loki", },
	{},
};
MODULE_DEVICE_TABLE(of, mrvl_loki_of_match);

static struct platform_driver mrvl_loki_driver = {
	.probe = mrvl_loki_probe,
	.remove = mrvl_loki_remove,
	.driver = {
		.name = "mrvl-loki",
		.of_match_table = of_match_ptr(mrvl_loki_of_match),
	},
};

module_platform_driver(mrvl_loki_driver);

MODULE_DESCRIPTION("Marvell Loki Driver");
MODULE_AUTHOR("Radha Mohan Chintakuntla");
MODULE_LICENSE("GPL v2");
