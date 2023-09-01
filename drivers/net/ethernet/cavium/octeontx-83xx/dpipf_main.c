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
#include <linux/delay.h>

#include "dpi.h"

#define DRV_NAME	"octeontx-dpi"
#define DRV_VERSION	"1.0"

static atomic_t dpi_count = ATOMIC_INIT(0);
static DEFINE_MUTEX(octeontx_dpi_devices_lock);
static LIST_HEAD(octeontx_dpi_devices);

static int dpi_init(struct dpipf *dpi);
static int dpi_fini(struct dpipf *dpi);
static int dpi_queue_init(struct dpipf *dpi, u16 domain_id,
			  u16 subdomain_id, int buf_size,
			  u16 aura);

static int dpi_queue_fini(struct dpipf *dpi,
			  u16 domain_id,
			  u16 vf);

static int dpi_reg_dump(struct dpipf *dpi,
			u16 domain_id,
			u16 vf);

static int dpi_queue_reset(struct dpipf *dpi,
			   u16 vf);

static int dpi_get_reg_cfg(struct dpipf *dpi,
			   u16 domain_id, u16 vf,
			   struct mbox_dpi_reg_cfg *reg_cfg);

static int mps = 128;
module_param(mps, int, 0644);
MODULE_PARM_DESC(mps, "Maximum payload size, Supported sizes are 128, 256 and 512 bytes");

static int mrrs = 128;
module_param(mrrs, int, 0644);
MODULE_PARM_DESC(mrrs, "Maximum read request size, Supported sizes are 128, 256, 512 and 1024 bytes");

/* Supported devices */
static const struct pci_device_id dpi_id_table[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_CAVIUM, PCI_DEVICE_ID_OCTEONTX_DPI_PF) },
	{ 0, }	/* end of table */
};

MODULE_AUTHOR("Cavium");
MODULE_DESCRIPTION("Cavium Thunder DPI Physical Function Driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRV_VERSION);
MODULE_DEVICE_TABLE(pci, dpi_id_table);

/* Register read/write APIs */
static void dpi_reg_write(struct dpipf *dpi, u64 offset, u64 val)
{
	writeq_relaxed(val, dpi->reg_base + offset);
}

static u64 dpi_reg_read(struct dpipf *dpi, u64 offset)
{
	return readq_relaxed(dpi->reg_base + offset);
}

static void identify(struct dpipf_vf *vf, u16 domain_id,
		     u16 subdomain_id)
{
	u64 reg = (((u64)subdomain_id << 16) | (domain_id)) << 8;

	writeq_relaxed(reg, vf->domain.reg_base + DPI_VDMA_SADDR);
}

static int dpi_pf_destroy_domain(u32 id, u16 domain_id, struct kobject *kobj)
{
	struct dpipf *dpi = NULL;
	struct dpipf *curr;
	int i, vf_idx = 0;
	struct pci_dev *virtfn;

	mutex_lock(&octeontx_dpi_devices_lock);
	list_for_each_entry(curr, &octeontx_dpi_devices, list) {
		if (curr->id == id) {
			dpi = curr;
			break;
		}
	}

	if (!dpi) {
		mutex_unlock(&octeontx_dpi_devices_lock);
		return -ENODEV;
	}

	for (i = 0; i < dpi->total_vfs; i++) {
		if (dpi->vf[i].domain.in_use &&
		    dpi->vf[i].domain.domain_id == domain_id) {
			dpi->vf[i].domain.in_use = false;
			identify(&dpi->vf[i], 0x0, 0x0);
			dpi_reg_write(dpi, DPI_DMAX_IDS(i), 0x0ULL);

			if (dpi->vf[i].domain.reg_base)
				iounmap(dpi->vf[i].domain.reg_base);

			virtfn = pci_get_domain_bus_and_slot(pci_domain_nr
					(dpi->pdev->bus),
					pci_iov_virtfn_bus(dpi->pdev, i),
					pci_iov_virtfn_devfn(dpi->pdev, i));

			if (virtfn && kobj)
				sysfs_remove_link(kobj, virtfn->dev.kobj.name);

			dev_info(&dpi->pdev->dev,
				 "Free vf[%d] from domain:%d subdomain_id:%d\n",
				 i, dpi->vf[i].domain.domain_id, vf_idx);
			vf_idx++;
		}
	}

	mutex_unlock(&octeontx_dpi_devices_lock);

	return 0;
}

static u64 dpi_pf_create_domain(u32 id, u16 domain_id, u32 num_vfs,
				void *master, void *master_data,
				struct kobject *kobj)
{
	struct dpipf *dpi = NULL;
	struct dpipf *curr;
	u64 i;
	int vf_idx = 0, ret = 0;
	resource_size_t vf_start;
	struct pci_dev *virtfn;
	unsigned long dpi_mask = 0;

	if (!kobj)
		return 0;

	mutex_lock(&octeontx_dpi_devices_lock);
	list_for_each_entry(curr, &octeontx_dpi_devices, list) {
		if (curr->id == id) {
			dpi = curr;
			break;
		}
	}

	if (!dpi)
		goto err_unlock;

	for (i = 0; i < dpi->total_vfs; i++) {
		if (dpi->vf[i].domain.in_use) {
			continue;
		} else {
			virtfn = pci_get_domain_bus_and_slot(pci_domain_nr
					(dpi->pdev->bus),
					pci_iov_virtfn_bus(dpi->pdev, i),
					pci_iov_virtfn_devfn(dpi->pdev, i));
			if (!virtfn)
				break;

			ret = sysfs_create_link(kobj, &virtfn->dev.kobj,
						virtfn->dev.kobj.name);
			if (ret < 0)
				goto err_unlock;

			dpi->vf[i].domain.domain_id = domain_id;
			dpi->vf[i].domain.subdomain_id = vf_idx;
			dpi->vf[i].domain.gmid = get_gmid(domain_id);

			dpi->vf[i].domain.in_use = true;
			dpi->vf[i].domain.master = master;
			dpi->vf[i].domain.master_data = master_data;

			vf_start = pci_resource_start(dpi->pdev,
						      PCI_DPI_PF_CFG_BAR);
			vf_start += DPI_VF_OFFSET(i);

			dpi->vf[i].domain.reg_base =
				ioremap_wc(vf_start, DPI_VF_CFG_SIZE);

			if (!dpi->vf[i].domain.reg_base)
				break;

			identify(&dpi->vf[i], domain_id, vf_idx);
			dpi_reg_write(dpi, DPI_DMAX_IDS(i),
				      DPI_DMA_IDS_INST_AURA(0) |
				      DPI_DMA_IDS_INST_STRM(vf_idx + 1) |
				      DPI_DMA_IDS_DMA_STRM(vf_idx + 1) |
				      get_gmid(domain_id));

			dev_dbg(&dpi->pdev->dev, "DOMAIN Details of DPI\n");

			dev_dbg(&dpi->pdev->dev,
				"domain creation @index: %llx for domain: %d, sub domain: %d, gmid: %d, vf_idx: %d\n",
				i, dpi->vf[i].domain.domain_id,
				dpi->vf[i].domain.subdomain_id,
				dpi->vf[i].domain.gmid, vf_idx);

			dev_dbg(&dpi->pdev->dev, "DPI_VDMA_SADDR: 0x%016llx\n",
				readq_relaxed(dpi->vf[i].domain.reg_base
				+ DPI_VDMA_SADDR));

			dev_dbg(&dpi->pdev->dev, "DPI_DMA%llx_IDS: 0x%016llx\n",
				i, dpi_reg_read(dpi, DPI_DMAX_IDS(i)));

			set_bit(i, &dpi_mask);
			vf_idx++;
			if (vf_idx == num_vfs) {
				dpi->vfs_in_use += num_vfs;
				break;
			}
		}
	}

	mutex_unlock(&octeontx_dpi_devices_lock);

	if (vf_idx != num_vfs) {
		dpi_mask = 0;
		dpi_pf_destroy_domain(id, domain_id, kobj);
	}
	return dpi_mask;

err_unlock:
	mutex_unlock(&octeontx_dpi_devices_lock);
	return 0;
}

static struct dpipf_vf *get_vf(u32 id, u16 domain_id, u16 subdomain_id,
			       struct dpipf **master)
{
	struct dpipf *dpi = NULL;
	struct dpipf *curr;
	int i;
	int vf_idx = -1;

	list_for_each_entry(curr, &octeontx_dpi_devices, list) {
		if (curr->id == id) {
			dpi = curr;
			break;
		}
	}

	if (!dpi)
		return NULL;

	for (i = 0; i < dpi->total_vfs; i++) {
		if (dpi->vf[i].domain.domain_id == domain_id &&
		    dpi->vf[i].domain.subdomain_id == subdomain_id) {
			vf_idx = i;
			if (master)
				*master = dpi;
			break;
		}
	}
	if (vf_idx >= 0)
		return &dpi->vf[vf_idx];
	else
		return NULL;
}

static int dpi_pf_receive_message(u32 id, u16 domain_id,
				  struct mbox_hdr *hdr,
				  union mbox_data *req,
				  union mbox_data *resp,
				  void *mdata)
{
	struct dpipf_vf *vf;
	struct dpipf *dpi = NULL;
	struct mbox_dpi_cfg *cfg;

	mutex_lock(&octeontx_dpi_devices_lock);

	vf = get_vf(id, domain_id, hdr->vfid, &dpi);

	if (!vf) {
		hdr->res_code = MBOX_RET_INVALID;
		mutex_unlock(&octeontx_dpi_devices_lock);
		return -ENODEV;
	}

	switch (hdr->msg) {
	case DPI_QUEUE_OPEN:
		cfg = mdata;
		dpi_queue_init(dpi, domain_id, hdr->vfid, cfg->buf_size,
			       cfg->inst_aura);
		resp->data = 0;
		break;

	case DPI_QUEUE_CLOSE:
		dpi_queue_fini(dpi, domain_id, hdr->vfid);
		resp->data = 0;
		break;

	case DPI_REG_DUMP:
		dpi_reg_dump(dpi, domain_id, hdr->vfid);
		resp->data = 0;
		break;

	case DPI_GET_REG_CFG:
		dpi_get_reg_cfg(dpi, domain_id, hdr->vfid, mdata);
		resp->data = sizeof(struct mbox_dpi_reg_cfg);
		break;

	case IDENTIFY:
		identify(vf, domain_id, hdr->vfid);
		resp->data = 0;
		break;

	default:
		hdr->res_code = MBOX_RET_INVALID;
		return -EINVAL;
	}

	hdr->res_code = MBOX_RET_SUCCESS;
	mutex_unlock(&octeontx_dpi_devices_lock);
	return 0;
}

static int dpi_pf_get_vf_count(u32 id)
{
	struct dpipf *dpi = NULL;
	struct dpipf *curr;
	int ret = 0;

	mutex_lock(&octeontx_dpi_devices_lock);
	list_for_each_entry(curr, &octeontx_dpi_devices, list) {
		if (curr->id == id) {
			dpi = curr;
			break;
		}
	}

	mutex_unlock(&octeontx_dpi_devices_lock);
	if (dpi)
		ret = dpi->total_vfs;

	return ret;
}

int dpi_reset_domain(u32 id, u16 domain_id)
{
	struct dpipf *dpi = NULL;
	struct dpipf *curr;
	int i;

	mutex_lock(&octeontx_dpi_devices_lock);
	list_for_each_entry(curr, &octeontx_dpi_devices, list) {
		if (curr->id == id) {
			dpi = curr;
			break;
		}
	}

	if (!dpi) {
		mutex_unlock(&octeontx_dpi_devices_lock);
		return -ENODEV;
	}

	for (i = 0; i < dpi->total_vfs; i++) {
		if (dpi->vf[i].domain.in_use &&
		    dpi->vf[i].domain.domain_id == domain_id) {
			dpi_queue_reset(dpi, i);
			identify(&dpi->vf[i], domain_id,
				 dpi->vf[i].domain.subdomain_id);
		}
	}

	mutex_unlock(&octeontx_dpi_devices_lock);
	return 0;
}

struct dpipf_com_s dpipf_com  = {
	.create_domain = dpi_pf_create_domain,
	.destroy_domain = dpi_pf_destroy_domain,
	.reset_domain = dpi_reset_domain,
	.receive_message = dpi_pf_receive_message,
	.get_vf_count = dpi_pf_get_vf_count
};
EXPORT_SYMBOL(dpipf_com);

static void dpi_irq_free(struct dpipf *dpi)
{
	int i = 0;

	/* Clear All Enables */
	dpi_reg_write(dpi, DPI_INT_ENA_W1C, DPI_INT_REG_NFOVR |
		      DPI_INT_REG_NDERR);
	dpi_reg_write(dpi, DPI_SBE_INT_ENA_W1C, DPI_SBE_INT_RDB_SBE);
	dpi_reg_write(dpi, DPI_DBE_INT_ENA_W1C, DPI_DBE_INT_RDB_DBE);

	for (i = 0; i < DPI_PF_MSIX_COUNT; i++) {
		if (dpi->msix_entries[i].vector)
			free_irq(dpi->msix_entries[i].vector, dpi);
	}

	pci_disable_msix(dpi->pdev);
	devm_kfree(&dpi->pdev->dev, dpi->msix_entries);

	for (i = 0; i < DPI_MAX_CC_INT; i++) {
		dpi_reg_write(dpi, DPI_REQQX_INT(i), DPI_REQQ_INT);
		dpi_reg_write(dpi, DPI_REQQX_INT_ENA_W1C(i), DPI_REQQ_INT);
	}

	for (i = 0; i < DPI_MAX_REQQ_INT; i++) {
		dpi_reg_write(dpi, DPI_DMA_CCX_INT(i), DPI_DMA_CC_INT);
		dpi_reg_write(dpi, DPI_DMA_CCX_INT_ENA_W1C(i), DPI_DMA_CC_INT);
	}
}

static irqreturn_t dpi_pf_intr_handler (int irq, void *dpi_irq)
{
	u64 reg_val = 0;
	int i = 0;
	struct dpipf *dpi = (struct dpipf *)dpi_irq;

	dev_err(&dpi->pdev->dev, "intr received: %d\n", irq);

	/* extract MSIX vector number from irq number. */
	while (irq != dpi->msix_entries[i].vector) {
		i++;
		if (i > DPI_PF_MSIX_COUNT)
			break;
	}
	if (i < DPI_DMA_REQQ_INT) {
		reg_val = dpi_reg_read(dpi, DPI_DMA_CCX_INT(i));
		dev_err(&dpi->pdev->dev, "DPI_CC%d_INT raised: 0x%016llx\n",
			i, reg_val);
		dpi_reg_write(dpi, DPI_DMA_CCX_INT(i), 0x1ULL);
	} else if (i < DPI_DMA_INT_REG) {
		reg_val = dpi_reg_read(dpi,
				       DPI_REQQX_INT(i - DPI_DMA_REQQ_INT));
		dev_err(&dpi->pdev->dev,
			"DPI_REQQ_INT raised for q:%d: 0x%016llx\n",
			(i - 0x40), reg_val);

		dpi_reg_write(dpi,
			      DPI_REQQX_INT(i - DPI_DMA_REQQ_INT), reg_val);

		if (reg_val & (0x71ULL))
			dpi_queue_reset(dpi, (i - DPI_DMA_REQQ_INT));
	} else if (i == DPI_DMA_INT_REG) {
		reg_val = dpi_reg_read(dpi, DPI_INT_REG);
		dev_err(&dpi->pdev->dev, "DPI_INT_REG raised: 0x%016llx\n",
			reg_val);
		dpi_reg_write(dpi, DPI_INT_REG, reg_val);
	} else if (i == DPI_DMA_SBE_INT) {
		reg_val = dpi_reg_read(dpi, DPI_SBE_INT);
		dev_err(&dpi->pdev->dev, "DPI_SBE_INT raised: 0x%016llx\n",
			reg_val);
		dpi_reg_write(dpi, DPI_SBE_INT, reg_val);
	} else	if (i == DPI_DMA_DBE_INT) {
		reg_val = dpi_reg_read(dpi, DPI_DBE_INT);
		dev_err(&dpi->pdev->dev, "DPI_DBE_INT raised: 0x%016llx\n",
			reg_val);
		dpi_reg_write(dpi, DPI_DBE_INT, reg_val);
	}	return IRQ_HANDLED;
}

static int dpi_irq_init(struct dpipf *dpi)
{
	int i;
	int ret = 0;

	/* Clear All Interrupts */
	dpi_reg_write(dpi, DPI_INT_REG, DPI_INT_REG_NFOVR | DPI_INT_REG_NDERR);
	dpi_reg_write(dpi, DPI_SBE_INT, DPI_SBE_INT_RDB_SBE);
	dpi_reg_write(dpi, DPI_DBE_INT, DPI_DBE_INT_RDB_DBE);

	/* Clear All Enables */
	dpi_reg_write(dpi, DPI_INT_ENA_W1C, DPI_INT_REG_NFOVR |
		      DPI_INT_REG_NDERR);
	dpi_reg_write(dpi, DPI_SBE_INT_ENA_W1C, DPI_SBE_INT_RDB_SBE);
	dpi_reg_write(dpi, DPI_DBE_INT_ENA_W1C, DPI_DBE_INT_RDB_DBE);

	for (i = 0; i < 8; i++) {
		dpi_reg_write(dpi, DPI_REQQX_INT(i), DPI_REQQ_INT);
		dpi_reg_write(dpi, DPI_REQQX_INT_ENA_W1C(i), DPI_REQQ_INT);
	}

	for (i = 0; i < 64; i++) {
		dpi_reg_write(dpi, DPI_DMA_CCX_INT(i), DPI_DMA_CC_INT);
		dpi_reg_write(dpi, DPI_DMA_CCX_INT_ENA_W1C(i), DPI_DMA_CC_INT);
	}

	dpi->msix_entries =
	devm_kzalloc(&dpi->pdev->dev,
		     DPI_PF_MSIX_COUNT * sizeof(struct msix_entry), GFP_KERNEL);

	if (!dpi->msix_entries)
		return -ENOMEM;

	for (i = 0; i < DPI_PF_MSIX_COUNT; i++)
		dpi->msix_entries[i].entry = i;

	ret = pci_enable_msix_exact(dpi->pdev, dpi->msix_entries, DPI_PF_MSIX_COUNT);
	if (ret) {
		dev_err(&dpi->pdev->dev, "Enabling msix failed\n");
		goto free_entries;
	}

	for (i = 0; i < DPI_PF_MSIX_COUNT; i++) {
		ret = request_irq(dpi->msix_entries[i].vector,
				  dpi_pf_intr_handler, 0, "dpipf", dpi);
		if (ret)
			goto free_irq;
	}
#define ENABLE_DPI_INTERRUPTS 0
#if ENABLE_DPI_INTERRUPTS
	/*Enable All Interrupts */
	dpi_reg_write(dpi, DPI_INT_ENA_W1S, DPI_INT_REG_NFOVR |
		      DPI_INT_REG_NDERR);
	dpi_reg_write(dpi, DPI_SBE_INT_ENA_W1S, DPI_SBE_INT_RDB_SBE);
	dpi_reg_write(dpi, DPI_DBE_INT_ENA_W1S, DPI_DBE_INT_RDB_DBE);

	for (i = 0; i < 8; i++)
		dpi_reg_write(dpi, DPI_REQQX_INT_ENA_W1S(i), DPI_REQQ_INT);
#endif
	return 0;
free_irq:
	for (; i >= 0; i--)
		free_irq(dpi->msix_entries[i].vector, dpi);
	pci_disable_msix(dpi->pdev);

free_entries:
	devm_kfree(&dpi->pdev->dev, dpi->msix_entries);
	return ret;
}

/* cavium-pf code starts here */
static int dpi_sriov_configure(struct pci_dev *pdev, int numvfs)
{
	struct dpipf *dpi = pci_get_drvdata(pdev);
	int ret = -EBUSY;
	int disable = 0;

	if (dpi->vfs_in_use != 0)
		return ret;

	ret = 0;
	if (dpi->flags & DPI_SRIOV_ENABLED)
		disable = 1;

	if (disable) {
		pci_disable_sriov(pdev);
		dpi->flags &= ~DPI_SRIOV_ENABLED;
		dpi->total_vfs = 0;
	}

	if (numvfs > 0) {
		ret = pci_enable_sriov(pdev, numvfs);
		if (ret == 0) {
			dpi->flags |= DPI_SRIOV_ENABLED;
			dpi->total_vfs = numvfs;
			ret = numvfs;
		}
	}
	return ret;
}

int dpi_dma_engine_get_num(void)
{
	return DPI_MAX_ENGINES;
}

/**
 * Perform global init of DPI
 *
 * @return Zero on success, negative on failure
 */
int dpi_init(struct dpipf *dpi)
{
	int engine = 0, port;
	u8 mrrs_val, mps_val;
	u64 reg = 0;

	for (engine = 0; engine < dpi_dma_engine_get_num(); engine++) {
		if (engine == 4 || engine == 5)
			reg = DPI_ENG_BUF_BLKS(8);
		else
			reg = DPI_ENG_BUF_BLKS(4);

		dpi_reg_write(dpi, DPI_ENGX_BUF(engine), reg);

		/* Here qmap for the engines are set to 0.
		 * No dpi queues are mapped to engines.
		 * When a VF is initialised corresponding bit
		 * in the qmap will be set for all engines.
		 */
		dpi_reg_write(dpi, DPI_DMA_ENGX_EN(engine), 0x0ULL);
	}

	reg = 0;
	reg =  (DPI_DMA_CONTROL_ZBWCSEN | DPI_DMA_CONTROL_PKT_EN |
		DPI_DMA_CONTROL_LDWB | DPI_DMA_CONTROL_O_MODE |
		DPI_DMA_CONTROL_DMA_ENB(0xfULL));

	dpi_reg_write(dpi, DPI_DMA_CONTROL, reg);

	dpi_reg_write(dpi, DPI_CTL, DPI_CTL_EN);

	/* Configure MPS and MRRS for DPI */
	if (mrrs < DPI_SLI_MRRS_MIN || mrrs > DPI_SLI_MRRS_MAX ||
	    !is_power_of_2(mrrs)) {
		dev_info(&dpi->pdev->dev,
			 "Invalid MRRS size:%d,Using default size(128 bytes)\n",
			 mrrs);
		mrrs = 128;
	}
	mrrs_val = fls(mrrs) - 8;

	if (mps < DPI_SLI_MPS_MIN || mps > DPI_SLI_MPS_MAX ||
	    !is_power_of_2(mps)) {
		dev_info(&dpi->pdev->dev,
			 "Invalid MPS size:%d,Using default size(128 bytes)\n",
			 mps);
		mps = 128;
	}
	mps_val = fls(mps) - 8;

	for (port = 0; port < DPI_SLI_MAX_PORTS; port++) {
		reg = dpi_reg_read(dpi, DPI_SLI_PRTX_CFG(port));
		reg &= ~(DPI_SLI_PRTX_CFG_MRRS(0x7) |
			 DPI_SLI_PRTX_CFG_MPS(0x7));
		reg |= (DPI_SLI_PRTX_CFG_MPS(mps_val) |
			DPI_SLI_PRTX_CFG_MRRS(mrrs_val));
		dpi_reg_write(dpi, DPI_SLI_PRTX_CFG(port), reg);
	}

	return 0;
}

int dpi_fini(struct dpipf *dpi)
{
	int engine = 0, port;
	u64 reg = 0ULL;

	for (port = 0; port < DPI_SLI_MAX_PORTS; port++) {
		reg = dpi_reg_read(dpi, DPI_SLI_PRTX_CFG(port));
		reg &= ~(DPI_SLI_PRTX_CFG_MRRS(0x7) |
			 DPI_SLI_PRTX_CFG_MPS(0x7));
		dpi_reg_write(dpi, DPI_SLI_PRTX_CFG(port), reg);
	}

	for (engine = 0; engine < dpi_dma_engine_get_num(); engine++) {
		dpi_reg_write(dpi, DPI_ENGX_BUF(engine), 0x0ULL);
		dpi_reg_write(dpi, DPI_DMA_ENGX_EN(engine), 0x0ULL);
	}

	dpi_reg_write(dpi, DPI_DMA_CONTROL, 0x0ULL);
	dpi_reg_write(dpi, DPI_CTL, ~DPI_CTL_EN);

	return 0;
}

int dpi_queue_init(struct dpipf *dpi, u16 domain_id,
		   u16 vf, int buf_size, u16 aura)
{
	int engine = 0;
	int queue = vf;
	u64 reg = 0ULL;

	dpi_reg_write(dpi, DPI_DMAX_IBUFF_CSIZE(queue),
		      DPI_DMA_IBUFF_CSIZE_CSIZE((u64)(buf_size / 8)));

	/* IDs are already configured while crating the domains.
	 * No need to configure here.
	 */
	for (engine = 0; engine < dpi_dma_engine_get_num(); engine++) {
		/* Dont configure the queus for PKT engines */
		if (engine >= 4)
			break;

		reg = 0;
		reg = dpi_reg_read(dpi, DPI_DMA_ENGX_EN(engine));
		reg |= DPI_DMA_ENG_EN_QEN(0x1 << queue);
		dpi_reg_write(dpi, DPI_DMA_ENGX_EN(engine), reg);
	}

	reg = dpi_reg_read(dpi, DPI_DMAX_IDS(queue));
	reg |= DPI_DMA_IDS_INST_AURA(aura);
	dpi_reg_write(dpi, DPI_DMAX_IDS(queue), reg);

	return 0;
}

int dpi_queue_fini(struct dpipf *dpi, u16 domain_id,
		   u16 vf)
{
	int engine = 0;
	int queue = vf;
	u64 reg = 0ULL;

	for (engine = 0; engine < dpi_dma_engine_get_num(); engine++) {
		/* Dont configure the queus for PKT engines */
		if (engine >= 4)
			break;

		reg = 0;
		reg = dpi_reg_read(dpi, DPI_DMA_ENGX_EN(engine));
		reg &= DPI_DMA_ENG_EN_QEN((~(1 << queue)));
		dpi_reg_write(dpi, DPI_DMA_ENGX_EN(engine), reg);
	}

	dpi_reg_write(dpi, DPI_DMAX_QRST(queue), 0x1ULL);

	return 0;
}

int dpi_queue_reset(struct dpipf *dpi, u16 vf)
{
	int engine = 0;
	u64 reg = 0ULL;
	struct dpipf_vf *dpivf = &dpi->vf[vf];
	u64 val = 0;

	/* wait for SADDR to become idle. */
	do {
		val = readq_relaxed(dpivf->domain.reg_base + DPI_VDMA_SADDR);
	} while (!(val & (0x1ULL << 63)));

	/* Disable the QEN bit in all engines for that queue/vf. */
	for (engine = 0; engine < dpi_dma_engine_get_num(); engine++) {
		/* Dont configure the queus for PKT engines.*/
		if (engine >= 4)
			break;

		reg = 0;
		reg = dpi_reg_read(dpi, DPI_DMA_ENGX_EN(engine));
		reg &= DPI_DMA_ENG_EN_QEN((~(1 << vf)));
		dpi_reg_write(dpi, DPI_DMA_ENGX_EN(engine), reg);
	}

	/* Reset the queue. */
	dpi_reg_write(dpi, DPI_DMAX_QRST(vf), 0x1ULL);

	/* Enable the QEN bit in all engines for that queue/vf. */
	for (engine = 0; engine < dpi_dma_engine_get_num(); engine++) {
		/* Dont configure the queus for PKT engines */
		if (engine >= 4)
			break;

		reg = 0;
		reg = dpi_reg_read(dpi, DPI_DMA_ENGX_EN(engine));
		reg |= DPI_DMA_ENG_EN_QEN((1 << vf));
		dpi_reg_write(dpi, DPI_DMA_ENGX_EN(engine), reg);
	}

	/* Reneable the Queue. */
	val = 0x1ULL;
	writeq_relaxed(val, dpivf->domain.reg_base + DPI_VDMA_EN);

	return 0;
}

int dpi_get_reg_cfg(struct dpipf *dpi, u16 domain_id, u16 vf,
		    struct mbox_dpi_reg_cfg *reg_cfg)
{
	reg_cfg->dpi_dma_ctl = dpi_reg_read(dpi, DPI_DMA_CONTROL);
	reg_cfg->dpi_sli_prt_cfg = dpi_reg_read(dpi, DPI_REQ_ERR_RESP_EN);
	reg_cfg->dpi_req_err_rsp_en = dpi_reg_read(dpi, DPI_SLI_PRTX_CFG(0));

	return 0;
}

int dpi_reg_dump(struct dpipf *dpi, u16 domain_id,
		 u16 vf)
{
	int i = vf;

	/* TODO: add the dump for required registers*/
	dev_info(&dpi->pdev->dev, "REG DUMP for VF: %d\n", vf);
	dev_info(&dpi->pdev->dev, "Global Registers\n");

	dev_info(&dpi->pdev->dev, "DPI_DMA_IBUFF_CSIZE: 0x%016llx\n",
		 dpi_reg_read(dpi, DPI_DMAX_IBUFF_CSIZE(i)));

	dev_info(&dpi->pdev->dev, "DPI_DMA_REQBANK0: 0x%016llx\n",
		 dpi_reg_read(dpi, DPI_DMAX_REQBANK0(i)));
	dev_info(&dpi->pdev->dev, "DPI_DMA_REQBANK1: 0x%016llx\n",
		 dpi_reg_read(dpi, DPI_DMAX_REQBANK1(i)));
	dev_info(&dpi->pdev->dev, "DPI_DMA_IDS: 0x%016llx\n",
		 dpi_reg_read(dpi, DPI_DMAX_IDS(i)));
	dev_info(&dpi->pdev->dev, "DPI_DMA_QRST: 0x%016llx\n",
		 dpi_reg_read(dpi, DPI_DMAX_QRST(i)));
	for (i = 0; i < 7; i++)
		dev_info(&dpi->pdev->dev, "DPI_DMA%d_ERR_RSP_STATUS: 0x%016llx\n",
			 i, dpi_reg_read(dpi, DPI_DMAX_ERR_RSP_STATUS(i)));

	dev_info(&dpi->pdev->dev, "DPI_CTL: 0x%016llx\n",
		 dpi_reg_read(dpi, DPI_CTL));
	dev_info(&dpi->pdev->dev, "DPI_DMA_CONTROL: 0x%016llx\n",
		 dpi_reg_read(dpi, DPI_DMA_CONTROL));
	for (i = 0; i < 6; i++)
		dev_info(&dpi->pdev->dev, "DPI_DMA_ENG%d_EN: 0x%016llx\n", i,
			 dpi_reg_read(dpi, DPI_DMA_ENGX_EN(i)));

	dev_info(&dpi->pdev->dev, "DPI_REQ_ERR_RSP: 0x%016llx\n",
		 dpi_reg_read(dpi, DPI_REQ_ERR_RSP));

	dev_info(&dpi->pdev->dev, "DPI_REQ_ERR_RSP_EN: 0x%016llx\n",
		 dpi_reg_read(dpi, DPI_REQ_ERR_RESP_EN));

	dev_info(&dpi->pdev->dev, "DPI_PKT_ERR_RSP: 0x%016llx\n",
		 dpi_reg_read(dpi, DPI_PKT_ERR_RSP));

	dev_info(&dpi->pdev->dev, "DPI_SLI_PRT_CFG: 0x%016llx\n",
		 dpi_reg_read(dpi, DPI_SLI_PRTX_CFG(0)));
	dev_info(&dpi->pdev->dev, "DPI_SLI_PRT_ERROR: 0x%016llx\n",
		 dpi_reg_read(dpi, DPI_SLI_PRTX_ERR(0)));
	dev_info(&dpi->pdev->dev, "DPI_SLI_PRT_ERR_INFO: 0x%016llx\n",
		 dpi_reg_read(dpi, DPI_SLI_PRTX_ERR_INFO(0)));

	return 0;
}

int dpi_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	struct device *dev = &pdev->dev;
	struct dpipf *dpi;
	int err;

	dpi = devm_kzalloc(dev, sizeof(*dpi), GFP_KERNEL);
	if (!dpi)
		return -ENOMEM;

	pci_set_drvdata(pdev, dpi);
	dpi->pdev = pdev;

	err = pcim_enable_device(pdev);
	if (err) {
		dev_err(dev, "Failed to enable PCI device\n");
		pci_set_drvdata(pdev, NULL);
		return err;
	}
	err = pci_request_regions(pdev, DRV_NAME);
	if (err) {
		dev_err(dev, "PCI request regions failed 0x%x\n", err);
		return err;
	}

	/* MAP PF's configuration registers */
	dpi->reg_base = pcim_iomap(pdev, PCI_DPI_PF_CFG_BAR, 0);
	if (!dpi->reg_base) {
		dev_err(dev, "Cannot map config register space, aborting\n");
		err = -ENOMEM;
		return err;
	}

	/*set DPI ID */
	dpi->id = atomic_add_return(1, &dpi_count);
	dpi->id -= 1;

	err = dpi_init(dpi);
	if (err) {
		dev_err(dev, "Failed to init DPI\n");
		atomic_sub_return(1, &dpi_count);
		return err;
	}

	/* Register interrupts */
	err = dpi_irq_init(dpi);
	if (err) {
		atomic_sub_return(1, &dpi_count);
		dev_err(dev, "failed init irqs\n");
		err = -EINVAL;
		return err;
	}

	INIT_LIST_HEAD(&dpi->list);
	mutex_lock(&octeontx_dpi_devices_lock);
	list_add(&dpi->list, &octeontx_dpi_devices);
	mutex_unlock(&octeontx_dpi_devices_lock);

	return 0;
}

static void dpi_remove(struct pci_dev *pdev)
{
	struct dpipf *dpi = pci_get_drvdata(pdev);

	dpi_fini(dpi);
	dpi_irq_free(dpi);
	dpi_sriov_configure(pdev, 0);
}

static struct pci_driver dpi_driver = {
	.name = DRV_NAME,
	.id_table = dpi_id_table,
	.probe = dpi_probe,
	.remove = dpi_remove,
	.sriov_configure = dpi_sriov_configure,
};

static int __init dpi_init_module(void)
{
	pr_info("%s, ver %s\n", DRV_NAME, DRV_VERSION);

	return pci_register_driver(&dpi_driver);
}

static void __exit dpi_cleanup_module(void)
{
	pci_unregister_driver(&dpi_driver);
}

module_init(dpi_init_module);
module_exit(dpi_cleanup_module);
