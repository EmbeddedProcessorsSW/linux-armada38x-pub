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

#define PSM_GPINT0_SUM_W1C	0x0ULL
#define PSM_GPINT0_SUM_W1S	0x40ULL
#define PSM_GPINT0_ENA_W1C	0x80ULL
#define PSM_GPINT0_ENA_W1S	0xC0ULL

#define CPRI_IP_AXI_INT_STATUS(a)	(0x100ULL | a << 10)
#define CPRI_IP_AXI_INT(a)		(0x108ULL | a << 10)

#define CPRI_MAX_MHAB		3
#define CONNIP_MAX_INST		5
#define CPRI_INT_MASK		0x1F

typedef int (*connip_irq_cb_t)(uint32_t instance, uint32_t pss_int);

struct mrvl_loki {
	struct pci_dev *pdev;
	struct msix_entry msix_ent;
	void __iomem *psm_gpint;
	void __iomem *cpri_axi[CPRI_MAX_MHAB];
	int intr_num;
	connip_irq_cb_t irq_cb;
};

struct mrvl_loki *g_ml;

int mrvl_loki_register_irq_cb(connip_irq_cb_t func)
{
	if (!g_ml) {
		pr_err("Error: mrvl_loki is NULL\n");
		return -ENOENT;
	}

	if (func)
		g_ml->irq_cb = func;
	else
		return -EIO;

	return 0;
}
EXPORT_SYMBOL(mrvl_loki_register_irq_cb);

void mrvl_loki_unregister_irq_cb(void)
{
	g_ml->irq_cb = NULL;
}
EXPORT_SYMBOL(mrvl_loki_unregister_irq_cb);

static irqreturn_t mrvl_loki_handler(int irq, void *dev)
{
	struct mrvl_loki *ml =
		platform_get_drvdata((struct platform_device *)dev);
	uint32_t instance, pss_int, val;
	uint8_t cpri, mac;
	int ret;

	/* clear GPINT */
	val = readq_relaxed(ml->psm_gpint + PSM_GPINT0_SUM_W1C) & CPRI_INT_MASK;
	writeq_relaxed((u64)val, ml->psm_gpint + PSM_GPINT0_SUM_W1C);

	for (instance = 0; instance < CONNIP_MAX_INST; instance++) {
		if (!(val & (1 << instance)))
			continue;
		cpri = instance / 2;
		mac = instance % 2;
		pss_int = (u32)readq_relaxed(ml->cpri_axi[cpri] +
					     CPRI_IP_AXI_INT_STATUS(mac));
		if (ml->irq_cb) {
			ret = ml->irq_cb(instance, pss_int);
			if (ret < 0)
				dev_err(dev,
					"Error %d from loki CPRI callback\n",
					ret);
		}

		/* clear AXI_INT */
		writeq_relaxed((u64)pss_int,
			       ml->cpri_axi[cpri] + CPRI_IP_AXI_INT(mac));
	}

	return IRQ_HANDLED;
}

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
	struct resource *res;
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

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ml->psm_gpint = ioremap(res->start, resource_size(res));
	if (IS_ERR(ml->psm_gpint)) {
		dev_err(dev, "error in ioremap PSM GPINT\n");
		return PTR_ERR(ml->psm_gpint);
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	ml->cpri_axi[0] = ioremap(res->start, resource_size(res));
	if (IS_ERR(ml->cpri_axi[0])) {
		dev_err(dev, "error in ioremap CPRI AXI0\n");
		return PTR_ERR(ml->cpri_axi[0]);
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	ml->cpri_axi[1] = ioremap(res->start, resource_size(res));
	if (IS_ERR(ml->cpri_axi[1])) {
		dev_err(dev, "error in ioremap CPRI AXI1\n");
		return PTR_ERR(ml->cpri_axi[1]);
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 3);
	ml->cpri_axi[2] = ioremap(res->start, resource_size(res));
	if (IS_ERR(ml->cpri_axi[2])) {
		dev_err(dev, "error in ioremap CPRI AXI2\n");
		return PTR_ERR(ml->cpri_axi[2]);
	}

	/* register interrupt */
	ml->intr_num = irq_of_parse_and_map(dev->of_node, 0);

	if (request_irq(ml->intr_num, mrvl_loki_handler, 0,
			"mrvl loki handler", pdev)) {
		dev_err(dev, "failed to register irq handler\n");
		ret = -ENOMEM;
		goto err;
	}

	g_ml = ml;
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
