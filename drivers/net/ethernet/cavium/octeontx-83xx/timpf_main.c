// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2017 Cavium, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License
 * as published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/random.h>

#include "rst.h"
#include "tim.h"

#define DRV_NAME "octeontx-tim"
#define DRV_VERSION "1.0"

/* TIM PCI Device ID (See PCC_DEV_IDL_E in HRM) */
#define PCI_DEVICE_ID_OCTEONTX_TIM_PF 0xA050
#define TIM_VF_CFG_SIZE		0x100000
#define TIM_VF_OFFSET(__x)	(0x10000000 | (TIM_VF_CFG_SIZE * (__x)))

#define PCI_TIM_PF_CFG_BAR	0
#define PCI_TIM_PF_MSIX_BAR	4
#define TIM_PF_MSIX_COUNT	2

#define TIM_DEV_PER_NODE	1
#define TIM_VFS_PER_DEV		64

#define TIM_RINGS_PER_DEV	TIM_VFS_PER_DEV
#define TIM_RING_NODE_SHIFT	6 /* 2 pow(6) */
#define TIM_RING_MASK		(TIM_RINGS_PER_DEV - 1)
#define TIM_MAX_RINGS \
	(TIM_RINGS_PER_DEV * TIM_DEV_PER_NODE * OCTTX_MAX_NODES)

/* TIM PF CSR offsets (within a single TIM device) */
#define TIM_REG_FLAGS		0x0
#define TIM_BKT_SKIP_INT	0x30
#define TIM_BKT_SKIP_INT_W1S	0x40
#define TIM_ECCERR_INT		0x60
#define TIM_ECCERR_INT_W1S	0x68
#define TIM_DBG2		0xA0
#define TIM_DBG3		0xA8
#define TIM_FR_RN_CYCLES	0xC0
#define TIM_BKT_SKIP_ENA_W1C	0x100
#define TIM_BKT_SKIP_ENA_W1S	0x108
#define TIM_ECCERR_ENA_W1C	0x110
#define TIM_ECCERR_ENA_W1S	0x118
#define TIM_ENG_ACTIVE(__x)	(0x1000 | ((__x) << 3))
#define TIM_RING_CTL0(__x)	(0x2000 | ((__x) << 3))
#define TIM_RING_CTL1(__x)	(0x2400 | ((__x) << 3))
#define TIM_RING_CTL2(__x)	(0x2800 | ((__x) << 3))
#define TIM_RING_GMCTL(__x)	(0x2A00 | ((__x) << 3))
#define TIM_BKT_SKIP_INT_STATUS(__x)	(0x2C00 | ((__x) << 3))
#define TIM_VRING_LATE(__x)	(0x2E00 | ((__x) << 3))

/* TIM device configuration and control block */
struct timpf_vf {
	struct octeontx_pf_vf domain;
	int vf_id;
	u64 start_cyc;
};

struct timpf {
	struct list_head list; /* List of TIM devices */
	struct pci_dev *pdev;
	void __iomem *reg_base;
	struct msix_entry *msix_entries;
	int id; /* Global/multinode TIM device ID (nod + TIM index).*/
	int total_vfs;
	int vfs_in_use;
#define TIM_SRIOV_ENABLED 0x1
	u32 flags;
	struct timpf_vf vf[TIM_VFS_PER_DEV];
};

/* Global list of TIM devices and rings */
static atomic_t tim_count = ATOMIC_INIT(0);
static DEFINE_MUTEX(octeontx_tim_dev_lock);
static LIST_HEAD(octeontx_tim_devices);

/* Interface to the RST device */
static struct rst_com_s *rst;

static inline void tim_reg_write(struct timpf *tim, u64 offset, u64 val)
{
	writeq_relaxed(val, tim->reg_base + offset);
}

static inline u64 tim_reg_read(struct timpf *tim, u64 offset)
{
	return readq_relaxed(tim->reg_base + offset);
}

static inline int node_from_devid(int id)
{
	return id;
}

static inline int dev_from_devid(int id)
{
	return id;
}

static inline int ringid_is_valid(unsigned int ringid)
{
	return ringid < TIM_MAX_RINGS;
}

static inline int node_from_ringid(int ringid)
{
	return ringid >> TIM_RING_NODE_SHIFT;
}

static inline int ring_from_ringid(int ringid)
{
	return ringid & TIM_RING_MASK;
}

static struct timpf *tim_dev_from_id(int id)
{
	struct timpf *tim;

	mutex_lock(&octeontx_tim_dev_lock);
	list_for_each_entry(tim, &octeontx_tim_devices, list) {
		if (tim->id == id) {
			mutex_unlock(&octeontx_tim_dev_lock);
			return tim;
		}
	}
	mutex_unlock(&octeontx_tim_dev_lock);
	return NULL;
}

static struct timpf *tim_dev_from_devid(int id, int domain_id, int devid)
{
	struct timpf *tim;

	mutex_lock(&octeontx_tim_dev_lock);
	list_for_each_entry(tim, &octeontx_tim_devices, list) {
		if (node_from_devid(tim->id) == id &&
		    dev_from_devid(tim->id) == devid) {
			mutex_unlock(&octeontx_tim_dev_lock);
			return tim;
		}
	}
	mutex_unlock(&octeontx_tim_dev_lock);
	return NULL;
}

static struct timpf *tim_dev_from_ringid(int id, int domain_id,
					 int ringid, int *ring)
{
	int i;
	struct timpf *tim;
	struct timpf_vf *vf;
	int node = node_from_ringid(ringid);

	if (id != node || !ringid_is_valid(ringid))
		return NULL;

	mutex_lock(&octeontx_tim_dev_lock);
	list_for_each_entry(tim, &octeontx_tim_devices, list) {
		if (node_from_devid(tim->id) != id)
			continue;
		for (i = 0; i < tim->total_vfs; i++) {
			vf = &tim->vf[i];
			if (vf->domain.domain_id == domain_id &&
			    vf->domain.subdomain_id == ringid) {
				mutex_unlock(&octeontx_tim_dev_lock);
				*ring = i;
				return tim;
			}
		}
	}
	mutex_unlock(&octeontx_tim_dev_lock);
	return NULL;
}

static inline u64 rdtsc(void)
{
	u64 tsc;

	asm volatile("mrs %0, cntvct_el0" : "=r" (tsc));
	return tsc;
}

/* Main MBOX message processing function.
 */
static int tim_pf_receive_message(u32 id, u16 domain_id, struct mbox_hdr *hdr,
				  union mbox_data *req, union mbox_data *resp,
				  void *mdata)
{
	struct timpf *tim;
	int rc, ring; /* TIM device local ring index */

	if (!mdata)
		return -ENOMEM;

	rc = 0;
	switch (hdr->msg) {
	case MBOX_TIM_DEV_INFO_GET: {
		struct mbox_tim_dev_info *info = mdata;
		int i;

		tim = tim_dev_from_devid(id, domain_id, hdr->vfid);
		if (!tim) {
			rc = -EINVAL;
			break;
		}
		for (i = 0; i < 4; i++)
			info->eng_active[i] = tim_reg_read(tim,
							   TIM_ENG_ACTIVE(i));
		info->tim_clk_freq = rst->get_sclk_freq(tim->id);
		resp->data = sizeof(struct mbox_tim_dev_info);
		/*make sure the writes are comitted*/
		wmb();
		break;
	}
	case MBOX_TIM_RING_INFO_GET: {
		struct mbox_tim_ring_info *info = mdata;

		tim = tim_dev_from_ringid(id, domain_id, hdr->vfid, &ring);
		if (!tim) {
			rc = -EINVAL;
			break;
		}
		info->node = id;
		info->ring_late = tim_reg_read(tim, TIM_VRING_LATE(ring));
		resp->data = sizeof(struct mbox_tim_ring_info);
		/*make sure the writes are comitted*/
		wmb();
		break;
	}
	case MBOX_TIM_RING_CONFIG_SET: {
		struct mbox_tim_ring_conf *conf = mdata;

		tim = tim_dev_from_ringid(id, domain_id, hdr->vfid, &ring);
		if (!tim) {
			rc = -EINVAL;
			break;
		}
		tim_reg_write(tim, TIM_RING_CTL2(ring), conf->ctl2);
		tim_reg_write(tim, TIM_RING_CTL0(ring), conf->ctl0);
		tim_reg_write(tim, TIM_RING_CTL1(ring), conf->ctl1);
		tim->vf[ring].start_cyc = rdtsc();
		resp->data = 0;
		break;
	}
	case MBOX_TIM_RING_START_CYC_GET: {
		u64 *ret = mdata;

		tim = tim_dev_from_ringid(id, domain_id, hdr->vfid, &ring);
		if (!tim) {
			rc = -EINVAL;
			break;
		}
		*ret = tim->vf[ring].start_cyc;
		resp->data = sizeof(uint64_t);
		/*make sure the writes are comitted*/
		wmb();
		break;
	}
	default:
		rc = -EINVAL;
		resp->data = 0;
		break;
	}
	if (rc)
		hdr->res_code = MBOX_RET_INVALID;
	else
		hdr->res_code = MBOX_RET_SUCCESS;
	return rc;
}

void identify(struct timpf_vf *vf, u16 domain_id, u16 subdomain_id)
{
	u64 offs = 0x100; /* TIM_VRING_BASE */
	u64 reg = MBOX_TIM_IDENT_CODE(domain_id, subdomain_id);

	writeq_relaxed(reg, vf->domain.reg_base + offs);
}

/* Domain control functions.
 */
static int tim_pf_destroy_domain(u32 id, u16 domain_id, struct kobject *kobj)
{
	struct timpf *tim = NULL;
	struct pci_dev *virtfn;
	struct timpf *curr;
	struct timpf_vf *vf;
	int i, vf_idx = 0;
	int ret = 0;
	u64 reg;

	mutex_lock(&octeontx_tim_dev_lock);
	list_for_each_entry(curr, &octeontx_tim_devices, list) {
		if (curr->id == id) {
			tim = curr;
			break;
		}
	}
	if (!tim) {
		ret = -ENODEV;
		goto err_unlock;
	}
	for (i = 0; i < tim->total_vfs; i++) {
		vf = &tim->vf[i];
		if (vf->domain.in_use &&
		    vf->domain.domain_id == domain_id) {
			vf->domain.in_use = false;

			virtfn = pci_get_domain_bus_and_slot(pci_domain_nr
				   (tim->pdev->bus),
				   pci_iov_virtfn_bus(tim->pdev, i),
				   pci_iov_virtfn_devfn(tim->pdev, i));
			if (virtfn && kobj)
				sysfs_remove_link(kobj, virtfn->dev.kobj.name);

			dev_info(&tim->pdev->dev,
				 "Free vf[%d] from domain:%d subdomain_id:%d\n",
				 i, vf->domain.domain_id, vf_idx);
			/* Cleanup MMU info.*/
			reg = tim_reg_read(tim, TIM_RING_GMCTL(i));
			reg &= ~0xFFFFull; /*GMID*/
			tim_reg_write(tim, TIM_RING_GMCTL(i), reg);
			identify(vf, 0x0, 0x0);
			iounmap(tim->vf[i].domain.reg_base);
			vf_idx++;
		}
	}
	tim->vfs_in_use -= vf_idx;

err_unlock:
	mutex_unlock(&octeontx_tim_dev_lock);
	return ret;
}

static u64 tim_pf_create_domain(u32 id, u16 domain_id, u32 num_vfs,
				struct octeontx_master_com_t *com, void *domain,
				struct kobject *kobj)
{
	struct timpf *tim = NULL;
	struct pci_dev *virtfn;
	struct timpf_vf *vf;
	struct timpf *curr;
	resource_size_t ba;
	u64 reg = 0, gmid;
	int i, vf_idx = 0, ret = 0;
	unsigned long tim_mask = 0;

	if (!kobj)
		return 0;
	gmid = get_gmid(domain_id);

	mutex_lock(&octeontx_tim_dev_lock);
	list_for_each_entry(curr, &octeontx_tim_devices, list) {
		if (curr->id == id) {
			tim = curr;
			break;
		}
	}
	if (!tim)
		goto err_unlock;

	for (i = 0; i < tim->total_vfs; i++) {
		vf = &tim->vf[i];
		if (vf->domain.in_use)
			continue;

		virtfn = pci_get_domain_bus_and_slot(pci_domain_nr
						     (tim->pdev->bus),
				pci_iov_virtfn_bus(tim->pdev, i),
				pci_iov_virtfn_devfn(tim->pdev, i));
		if (!virtfn)
			break;
		ret = sysfs_create_link(kobj, &virtfn->dev.kobj,
					virtfn->dev.kobj.name);
		if (ret < 0)
			goto err_unlock;

		ba = pci_resource_start(tim->pdev, PCI_TIM_PF_CFG_BAR);
		ba += TIM_VF_OFFSET(i);
		vf->domain.reg_base = ioremap_wc(ba, TIM_VF_CFG_SIZE);
		vf->domain.domain_id = domain_id;
		vf->domain.subdomain_id = vf_idx;
		vf->domain.gmid = get_gmid(domain_id);
		vf->domain.master = com;
		vf->domain.master_data = domain;
		vf->domain.in_use = true;

		reg = ((uint64_t)i + 1) << 16 /*STRM*/ | gmid; /*GMID*/
		tim_reg_write(tim, TIM_RING_GMCTL(i), reg);

		identify(vf, domain_id, vf_idx);
		set_bit(i, &tim_mask);
		vf_idx++;
		if (vf_idx == num_vfs) {
			tim->vfs_in_use += num_vfs;
			break;
		}
	}
	if (vf_idx != num_vfs)
		goto err_unlock;

	mutex_unlock(&octeontx_tim_dev_lock);
	return tim_mask;

err_unlock:
	mutex_unlock(&octeontx_tim_dev_lock);
	tim_pf_destroy_domain(id, domain_id, kobj);
	return 0;
}

static int tim_ring_reset(struct timpf *tim, int ring)
{
	u64 reg;

	/* Stop the ring and set the power-on defaults for CTL registers.*/
	reg = tim_reg_read(tim, TIM_RING_CTL1(ring));
	reg &= ~(1ull << 47); /*ENA*/
	tim_reg_write(tim, TIM_RING_CTL1(ring), reg);
	return 0;
}

static int tim_pf_reset_domain(u32 id, u16 domain_id)
{
	struct timpf *tim = NULL;
	struct timpf_vf *vf;
	int i, sdom;

	mutex_lock(&octeontx_tim_dev_lock);
	list_for_each_entry(tim, &octeontx_tim_devices, list) {
		for (i = 0; i < tim->total_vfs; i++) {
			vf = &tim->vf[i];
			sdom = vf->domain.subdomain_id;
			if (vf->domain.in_use &&
			    vf->domain.domain_id == domain_id) {
				tim_ring_reset(tim, i);
				identify(vf, domain_id, sdom);
			}
		}
	}
	mutex_unlock(&octeontx_tim_dev_lock);
	return 0;
}

static int tim_pf_get_vf_count(u32 id)
{
	struct timpf *tim;

	tim = tim_dev_from_id(id);
	if (!tim)
		return 0;
	return tim->total_vfs;
}

/* Interface with the main OCTEONTX driver.
 */
struct timpf_com_s timpf_com  = {
	.create_domain = tim_pf_create_domain,
	.destroy_domain = tim_pf_destroy_domain,
	.reset_domain = tim_pf_reset_domain,
	.receive_message = tim_pf_receive_message,
	.get_vf_count = tim_pf_get_vf_count
};
EXPORT_SYMBOL(timpf_com);

/* Driver startup initialization and shutdown functions.
 */
static int tim_init(struct timpf *tim)
{
	u64 reg;
	int i;

	/* Initialize TIM rings.*/
	reg = (1ull << 48) |  /*LOCK_EN*/
#ifdef __BIG_ENDIAN
		(1ull << 54) | /*BE*/
#endif
		(1ull << 44); /*ENA_LDWB*/
	for (i = 0; i < TIM_RINGS_PER_DEV; i++) {
		tim_reg_write(tim, TIM_RING_CTL1(i), reg);
		tim_reg_write(tim, TIM_RING_CTL0(i), 0);
		tim_reg_write(tim, TIM_RING_CTL2(i), 0);
	}
	/* Reset free running counter and enable TIM device.*/
	reg = (1ull << 2)/*RESET*/ | 0x1ull; /*ENA_TIM*/
	tim_reg_write(tim, TIM_REG_FLAGS, reg);

	/* Initialize domain resources.*/
	for (i = 0; i < TIM_VFS_PER_DEV; i++) {
		tim->vf[i].domain.in_use = 0;
		tim->vf[i].domain.master = NULL;
		tim->vf[i].domain.master_data = NULL;
	}
	return 0;
}

static irqreturn_t tim_bkt_skip_intr_handler(int irq, void *tim_irq)
{
	struct timpf *tim = (struct timpf *)tim_irq;
	u64 reg;

	reg = tim_reg_read(tim, TIM_BKT_SKIP_INT);
	dev_err(&tim->pdev->dev, "BKT_SKIP_INT: 0x%llx\n", reg);
	tim_reg_write(tim, TIM_BKT_SKIP_INT, reg);
	return IRQ_HANDLED;
}

static irqreturn_t tim_eccerr_intr_handler(int irq, void *tim_irq)
{
	struct timpf *tim = (struct timpf *)tim_irq;
	u64 reg;

	reg = tim_reg_read(tim, TIM_ECCERR_INT);
	dev_err(&tim->pdev->dev, "ECCERR_INT: 0x%llx\n", reg);
	tim_reg_write(tim, TIM_ECCERR_INT, reg);
	return IRQ_HANDLED;
}

static struct intr_hand intr[TIM_PF_MSIX_COUNT] = {
	{~0ull, "tim bkt skip", TIM_BKT_SKIP_ENA_W1C, TIM_BKT_SKIP_ENA_W1S,
		tim_bkt_skip_intr_handler},
	{~0ull, "tim eccerr", TIM_ECCERR_ENA_W1C, TIM_ECCERR_ENA_W1S,
		tim_eccerr_intr_handler}
};

static void tim_irq_free(struct timpf *tim)
{
	int i;

	/* Clear interrupts */
	for (i = 0; i < TIM_PF_MSIX_COUNT; i++) {
		tim_reg_write(tim, intr[i].coffset, intr[i].mask);
		if (tim->msix_entries[i].vector)
			free_irq(tim->msix_entries[i].vector, tim);
	}
	pci_disable_msix(tim->pdev);
}

static int tim_irq_init(struct timpf *tim)
{
	int i;
	int ret = 0;

	/* Clear interrupts */
	for (i = 0; i < TIM_PF_MSIX_COUNT; i++)
		tim_reg_write(tim, intr[i].coffset, intr[i].mask);

	tim->msix_entries = devm_kzalloc(&tim->pdev->dev, TIM_PF_MSIX_COUNT *
					 sizeof(struct msix_entry), GFP_KERNEL);
	if (!tim->msix_entries)
		return -ENOMEM;

	for (i = 0; i < TIM_PF_MSIX_COUNT; i++)
		tim->msix_entries[i].entry = i;

	ret = pci_enable_msix_exact(tim->pdev, tim->msix_entries,
				    TIM_PF_MSIX_COUNT);
	if (ret < 0) {
		dev_err(&tim->pdev->dev, "Failed to enable TIM MSIX.\n");
		return ret;
	}
	for (i = 0; i < TIM_PF_MSIX_COUNT; i++) {
		ret = request_irq(tim->msix_entries[i].vector, intr[i].handler,
				  0, intr[i].name, tim);
		if (ret)
			goto free_irq;
	}
	/* Enable interrupts */
	for (i = 0; i < TIM_PF_MSIX_COUNT; i++)
		tim_reg_write(tim, intr[i].soffset, intr[i].mask);
	return 0;

free_irq:
	for (; i < TIM_PF_MSIX_COUNT; i++)
		tim->msix_entries[i].vector = 0;
	tim_irq_free(tim);
	return ret;
}

static int tim_sriov_configure(struct pci_dev *pdev, int numvfs)
{
	struct timpf *tim = pci_get_drvdata(pdev);
	int ret = -EBUSY;
	int disable = 0;

	if (tim->vfs_in_use != 0)
		return ret;

	ret = 0;
	if (tim->flags & TIM_SRIOV_ENABLED)
		disable = 1;

	if (disable) {
		pci_disable_sriov(pdev);
		tim->flags &= ~TIM_SRIOV_ENABLED;
		tim->total_vfs = 0;
	}
	if (numvfs > 0) {
		ret = pci_enable_sriov(pdev, numvfs);
		if (ret == 0) {
			tim->flags |= TIM_SRIOV_ENABLED;
			tim->total_vfs = numvfs;
			ret = numvfs;
		}
	}

	dev_notice(&tim->pdev->dev, "VFs enabled: %d\n", ret);
	return ret;
}

static int tim_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	struct device *dev = &pdev->dev;
	struct timpf *tim;
	int err = -ENOMEM;

	tim = devm_kzalloc(dev, sizeof(*tim), GFP_KERNEL);
	if (!tim)
		return -ENOMEM;

	pci_set_drvdata(pdev, tim);
	tim->pdev = pdev;

	err = pcim_enable_device(pdev);
	if (err) {
		dev_err(dev, "Failed to enable TIM PCI device\n");
		return err;
	}
	err = pci_request_regions(pdev, DRV_NAME);
	if (err) {
		dev_err(dev, "TIM PCI request regions failed\n");
		return err;
	}
	tim->reg_base = pcim_iomap(pdev, PCI_TIM_PF_CFG_BAR, 0);
	if (!tim->reg_base) {
		dev_err(dev, "Can't map TIM CFG space\n");
		return -ENOMEM;
	}
	tim->id = atomic_add_return(1, &tim_count);
	tim->id -= 1; /* Convert to 0-based */

	err = tim_init(tim);
	if (err < 0) {
		dev_err(dev, "Failed to initialize TIM device.\n");
		return err;
	}
	err = tim_irq_init(tim);
	if (err) {
		atomic_sub_return(1, &tim_count);
		dev_err(dev, "Failed to init TIM interrupts\n");
		return err;
	}
	INIT_LIST_HEAD(&tim->list);
	mutex_lock(&octeontx_tim_dev_lock);
	list_add(&tim->list, &octeontx_tim_devices);
	mutex_unlock(&octeontx_tim_dev_lock);
	return 0;
}

static void tim_remove(struct pci_dev *pdev)
{
	struct timpf *tim = pci_get_drvdata(pdev);
	struct timpf *curr;

	if (!tim)
		return;

	mutex_lock(&octeontx_tim_dev_lock);
	list_for_each_entry(curr, &octeontx_tim_devices, list) {
		if (curr == tim) {
			list_del(&tim->list);
			break;
		}
	}
	mutex_unlock(&octeontx_tim_dev_lock);

	tim_irq_free(tim);
	tim_sriov_configure(pdev, 0);
}

static const struct pci_device_id tim_id_table[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_CAVIUM, PCI_DEVICE_ID_OCTEONTX_TIM_PF) },
	{ 0 } /* End of table */
};

static struct pci_driver tim_driver = {
	.name = DRV_NAME,
	.id_table = tim_id_table,
	.probe = tim_probe,
	.remove = tim_remove,
	.sriov_configure = tim_sriov_configure,
};

MODULE_AUTHOR("Cavium");
MODULE_DESCRIPTION("Cavium OCTEONTX TIM PF Driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRV_VERSION);
MODULE_DEVICE_TABLE(pci, tim_id_table);

static int __init tim_init_module(void)
{
	pr_info("%s, ver %s\n", DRV_NAME, DRV_VERSION);
	rst = try_then_request_module(symbol_get(rst_com), "rst");
	if (!rst)
		return -ENODEV;

	return pci_register_driver(&tim_driver);
}

static void __exit tim_cleanup_module(void)
{
	pci_unregister_driver(&tim_driver);
	symbol_put(rst_com);
}

module_init(tim_init_module);
module_exit(tim_cleanup_module);

