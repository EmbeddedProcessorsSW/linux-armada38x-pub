// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2016 Cavium, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License
 * as published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/delay.h>

#include "fpa.h"

#define DRV_NAME "octeontx-fpa"
#define DRV_VERSION "1.0"

static atomic_t fpa_count = ATOMIC_INIT(0);

static DEFINE_MUTEX(octeontx_fpa_devices_lock);
static LIST_HEAD(octeontx_fpa_devices);

/* In Cavium OcteonTX SoCs, all accesses to the device registers are
 * implicitly strongly ordered.
 * So writeq_relaxed() and readq_relaxed() are safe to use
 * with out any memory barriers.
 */

/* Register read/write APIs */
static void fpa_reg_write(struct fpapf *fpa, u64 offset, u64 val)
{
	writeq_relaxed(val, fpa->reg_base + offset);
}

static u64 fpa_reg_read(struct fpapf *fpa, u64 offset)
{
	return readq_relaxed(fpa->reg_base + offset);
}

static u64 fpa_vf_alloc(void *reg_base, u32 aura)
{
	u64 addr = 0;

	addr = readq_relaxed(reg_base + FPA_VF_VHAURA_OP_ALLOC(aura));

	return addr;
}

/* Caller is responsible for locks */
static struct fpapf_vf *get_vf(u32 id, u16 domain_id, u16 subdomain_id,
			       struct fpapf **master)
{
	struct fpapf *fpa = NULL;
	struct fpapf *curr;
	int i;
	int vf_idx = -1;

	list_for_each_entry(curr, &octeontx_fpa_devices, list) {
		if (curr->id == id) {
			fpa = curr;
			break;
		}
	}

	if (!fpa)
		return NULL;

	for (i = 0; i < fpa->total_vfs; i++) {
		if (fpa->vf[i].domain.domain_id == domain_id &&
		    fpa->vf[i].domain.subdomain_id == subdomain_id) {
			vf_idx = i;
			if (master)
				*master = fpa;
			break;
		}
	}
	if (vf_idx >= 0)
		return &fpa->vf[vf_idx];
	else
		return NULL;
}

static void identify(struct fpapf_vf *vf, u16 domain_id,
		     u16 subdomain_id, u32 stack_ln_ptrs)
{
	u64 reg = (((u64)subdomain_id << 16) | domain_id) << 8;

	writeq_relaxed(0x0, vf->domain.reg_base +
			FPA_VF_VHPOOL_START_ADDR(0));

	writeq_relaxed(reg, vf->domain.reg_base +
		       FPA_VF_VHAURA_CNT_THRESHOLD(0));

	reg = stack_ln_ptrs;
	writeq_relaxed(reg, vf->domain.reg_base +
			FPA_VF_VHPOOL_THRESHOLD(0));
}

static int fpa_pf_receive_message(u32 id, u16 domain_id,
				  struct mbox_hdr *hdr,
				  union mbox_data *req,
				  union mbox_data *resp,
				  void *add_data)
{
	struct fpapf_vf *vf;
	struct fpapf *fpa = NULL;
	struct mbox_fpa_cfg *cfg;
	struct mbox_fpa_lvls *lvls;
	unsigned int aura, pool;
	u64 reg;
	int i;

	mutex_lock(&octeontx_fpa_devices_lock);

	vf = get_vf(id, domain_id, hdr->vfid, &fpa);
	if (!vf) {
		hdr->res_code = MBOX_RET_INVALID;
		mutex_unlock(&octeontx_fpa_devices_lock);
		return -ENODEV;
	}

	resp->data = 0;
	hdr->res_code = MBOX_RET_SUCCESS;

	switch (hdr->msg) {
	case IDENTIFY:
		identify(vf, domain_id,	hdr->vfid,
			 fpa->stack_ln_ptrs);
		break;
	case FPA_CONFIGSET:
		cfg = add_data;

		dev_dbg(&fpa->pdev->dev, "Setup vf[%u] stack:[%llx-%llx] cfg:%llx\n",
			vf->hardware_pool, cfg->pool_stack_base,
			cfg->pool_stack_end - 1, cfg->pool_cfg);

		/* Disable pool before configuration update */
		fpa_reg_write(fpa, FPA_PF_POOLX_CFG(vf->hardware_pool), 0x0);

		/* Update pool configuration and enable if required */
		fpa_reg_write(fpa, FPA_PF_AURAX_CFG((vf->hardware_aura_set *
				FPA_AURA_SET_SIZE) + cfg->aid),
				cfg->aura_cfg);
		fpa_reg_write(fpa, FPA_PF_POOLX_STACK_BASE(vf->hardware_pool),
			      cfg->pool_stack_base);
		fpa_reg_write(fpa, FPA_PF_POOLX_STACK_ADDR(vf->hardware_pool),
			      cfg->pool_stack_base);
		fpa_reg_write(fpa, FPA_PF_POOLX_STACK_END(vf->hardware_pool),
			      cfg->pool_stack_end);
		fpa_reg_write(fpa, FPA_PF_POOLX_CFG(vf->hardware_pool),
			      cfg->pool_cfg);
		break;
	case FPA_CONFIGGET:
		cfg = add_data;
		cfg->aura_cfg = fpa_reg_read(fpa, FPA_PF_AURAX_CFG
					     ((vf->hardware_aura_set *
					       FPA_AURA_SET_SIZE) + cfg->aid));
		cfg->pool_stack_base = fpa_reg_read(fpa, FPA_PF_POOLX_STACK_BASE
						    (vf->hardware_pool));
		cfg->pool_stack_end = fpa_reg_read(fpa,	FPA_PF_POOLX_STACK_END
						   (vf->hardware_pool));
		cfg->pool_cfg = fpa_reg_read(fpa, FPA_PF_POOLX_CFG
					     (vf->hardware_pool));

		/* update data read len */
		resp->data = sizeof(struct mbox_fpa_cfg);

		break;
	case FPA_START_COUNT:
		for (i = 0; i < FPA_AURA_SET_SIZE; i++) {
			reg = fpa_reg_read(fpa, FPA_PF_AURAX_CFG
					   ((vf->hardware_aura_set *
					     FPA_AURA_SET_SIZE) + i));
			reg = reg & ~(1ULL << 9);
			fpa_reg_write(fpa, FPA_PF_AURAX_CFG
				      ((vf->hardware_aura_set *
					FPA_AURA_SET_SIZE) + i), reg);
		}
		break;
	case FPA_STOP_COUNT:
		for (i = 0; i < FPA_AURA_SET_SIZE; i++) {
			reg = fpa_reg_read(fpa, FPA_PF_AURAX_CFG
					   ((vf->hardware_aura_set *
					     FPA_AURA_SET_SIZE) + i));
			reg = reg | (1 << 9);
			fpa_reg_write(fpa, FPA_PF_AURAX_CFG
				      ((vf->hardware_aura_set *
					FPA_AURA_SET_SIZE) + i), reg);
		}
		break;

	case FPA_ATTACHAURA:
		cfg = add_data;
		aura = vf->hardware_aura_set * FPA_AURA_SET_SIZE +
			(cfg->aid % FPA_AURA_SET_SIZE);

		/* Get pool from aura
		 * Assuming gpool is vf_id
		 */
		pool = aura / FPA_AURA_SET_SIZE;
		fpa_reg_write(fpa, FPA_PF_AURAX_POOL(aura), pool);

		/* Disable red/bp lvl */
		fpa_reg_write(fpa, FPA_PF_AURAX_CNT_LEVELS(aura), 1ul << 40);
		break;

	case FPA_DETACHAURA:
		cfg = add_data;
		aura = vf->hardware_aura_set * FPA_AURA_SET_SIZE +
			(cfg->aid % FPA_AURA_SET_SIZE);

		fpa_reg_write(fpa, FPA_PF_AURAX_POOL(aura), 0);

		/* Clear red/bp lvl */
		fpa_reg_write(fpa, FPA_PF_AURAX_CNT_LEVELS(aura), 0);
		break;

	case FPA_SETAURALVL:
		lvls = add_data;
		aura = vf->hardware_aura_set * FPA_AURA_SET_SIZE +
			(lvls->gaura % FPA_AURA_SET_SIZE);

		fpa_reg_write(fpa, FPA_PF_AURAX_CNT_LEVELS(aura),
			      lvls->cnt_levels);
		fpa_reg_write(fpa, FPA_PF_AURAX_POOL_LEVELS(aura),
			      lvls->pool_levels);
		break;

	case FPA_GETAURALVL:
		lvls = add_data;
		aura = vf->hardware_aura_set * FPA_AURA_SET_SIZE +
			(lvls->gaura % FPA_AURA_SET_SIZE);

		lvls->cnt_levels = fpa_reg_read(fpa,
						FPA_PF_AURAX_CNT_LEVELS(aura));
		lvls->pool_levels =
			fpa_reg_read(fpa, FPA_PF_AURAX_POOL_LEVELS(aura));

		/* Update data read len */
		resp->data = sizeof(struct mbox_fpa_lvls);
		break;

	default:
		hdr->res_code = MBOX_RET_INVALID;
		break;
	}

	mutex_unlock(&octeontx_fpa_devices_lock);
	return 0;
}

static ssize_t pool_maxcnt_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct fpapf *curr, *fpa = NULL;
	struct pci_dev *pdev = container_of(dev, struct pci_dev, dev);
	int i, n, vfid = pdev->devfn - 1;
	u64 cnt;
	char info[512];

	list_for_each_entry(curr, &octeontx_fpa_devices, list) {
		if (curr->pdev == pdev->physfn) {
			fpa = curr;
			break;
		}
	}
	if (!fpa)
		return 0;

	for (i = 0, n = 0; i < FPA_AURA_SET_SIZE; i++) {
		cnt = readq_relaxed(fpa->vf[vfid].domain.reg_base +
				    FPA_VF_VHAURA_CNT_LIMIT(i));
		n += sprintf(&info[n], "%lld\n", cnt);
	}
	return snprintf(buf, PAGE_SIZE, "%s", info);
}

static struct device_attribute pool_maxcnt_attr = {
	.attr = {.name = "pool_maxcnt",  .mode = 0444},
	.show = pool_maxcnt_show,
	.store = NULL
};

static ssize_t pool_curcnt_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct fpapf *curr, *fpa = NULL;
	struct pci_dev *pdev = container_of(dev, struct pci_dev, dev);
	int i, n, vfid = pdev->devfn - 1;
	u64 cnt;
	char info[512];

	list_for_each_entry(curr, &octeontx_fpa_devices, list) {
		if (curr->pdev == pdev->physfn) {
			fpa = curr;
			break;
		}
	}
	if (!fpa)
		return 0;

	for (i = 0, n = 0; i < FPA_AURA_SET_SIZE; i++) {
		cnt = readq_relaxed(fpa->vf[vfid].domain.reg_base +
				    FPA_VF_VHAURA_CNT(i));
		n += sprintf(&info[n], "%lld\n", cnt);
	}
	return snprintf(buf, PAGE_SIZE, "%s", info);
}

static struct device_attribute pool_curcnt_attr = {
	.attr = {.name = "pool_curcnt",  .mode = 0444},
	.show = pool_curcnt_show,
	.store = NULL
};

static ssize_t pool_redcnt_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct fpapf *curr, *fpa = NULL;
	struct pci_dev *pdev = container_of(dev, struct pci_dev, dev);
	int i, n, vfid = pdev->devfn - 1;
	u64 reg, ena, lvl, pass, drop, aura;
	char *info;

	list_for_each_entry(curr, &octeontx_fpa_devices, list) {
		if (curr->pdev == pdev->physfn) {
			fpa = curr;
			break;
		}
	}
	if (!fpa)
		return 0;

	info = kzalloc(PAGE_SIZE, GFP_KERNEL);
	if (!info)
		return 0;

	aura = vfid * FPA_AURA_SET_SIZE;
	for (i = 0, n = 0; i < FPA_AURA_SET_SIZE; i++, aura++) {
		reg = fpa_reg_read(fpa, FPA_PF_AURAX_CNT_LEVELS(aura));
		ena = reg & (1ull << 39);
		lvl = reg & 0xFFull;
		pass = (reg >> 8) & 0xFFull;
		drop = (reg >> 16) & 0xFFull;
		n += sprintf(&info[n], "%lld %lld %lld %lld\n",
			     ena, lvl, pass, drop);
	}
	n = snprintf(buf, PAGE_SIZE, "%s", info);
	kfree(info);
	return n;
}

static struct device_attribute pool_redcnt_attr = {
	.attr = {.name = "pool_redcnt",  .mode = 0444},
	.show = pool_redcnt_show,
	.store = NULL
};

static int fpa_pf_destroy_domain(u32 id, u16 domain_id, struct kobject *kobj)
{
	struct fpapf *fpa = NULL;
	struct pci_dev *virtfn;
	struct fpapf *curr;
	int i, j, vf_idx = 0;
	u64 reg;

	mutex_lock(&octeontx_fpa_devices_lock);
	list_for_each_entry(curr, &octeontx_fpa_devices, list) {
		if (curr->id == id) {
			fpa = curr;
			break;
		}
	}

	if (!fpa) {
		mutex_unlock(&octeontx_fpa_devices_lock);
		return -ENODEV;
	}

	for (i = 0; i < fpa->total_vfs; i++) {
		if (fpa->vf[i].domain.in_use &&
		    fpa->vf[i].domain.domain_id == domain_id) {
			reg = 0x1 << 7;
			writeq_relaxed(0x0, fpa->vf[i].domain.reg_base +
					FPA_VF_VHPOOL_START_ADDR(0));
			reg = -1;
			writeq_relaxed(reg, fpa->vf[i].domain.reg_base +
					FPA_VF_VHPOOL_END_ADDR(0));

			writeq_relaxed(reg, fpa->vf[i].domain.reg_base +
				       FPA_VF_VHAURA_CNT_THRESHOLD(0));

			writeq_relaxed(reg, fpa->vf[i].domain.reg_base +
					FPA_VF_VHPOOL_THRESHOLD(0));

			for (j = 0; j < FPA_AURA_SET_SIZE; j++)
				writeq_relaxed(0x0, fpa->vf[i].domain.reg_base
					       + FPA_VF_VHAURA_CNT(j));

			iounmap(fpa->vf[i].domain.reg_base);

			virtfn = pci_get_domain_bus_and_slot(pci_domain_nr
					(fpa->pdev->bus),
					pci_iov_virtfn_bus(fpa->pdev, i),
					pci_iov_virtfn_devfn(fpa->pdev, i));
			if (virtfn && kobj) {
				sysfs_remove_file(&virtfn->dev.kobj,
						  &pool_maxcnt_attr.attr);
				sysfs_remove_file(&virtfn->dev.kobj,
						  &pool_curcnt_attr.attr);
				sysfs_remove_file(&virtfn->dev.kobj,
						  &pool_redcnt_attr.attr);
				sysfs_remove_link(kobj, virtfn->dev.kobj.name);
			}
			dev_dbg(&fpa->pdev->dev,
				"Free vf[%d] from domain:%d subdomain_id:%d\n",
				i, fpa->vf[i].domain.domain_id, vf_idx);
			memset(&fpa->vf[i], 0, sizeof(struct octeontx_pf_vf));
			reg = FPA_MAP_VALID(0) | FPA_MAP_VHAURASET(i)
				| FPA_MAP_GAURASET(0)
				| FPA_MAP_GMID(fpa->vf[i].domain.gmid);
			fpa_reg_write(fpa, FPA_PF_MAPX(i), reg);
			vf_idx++;
		}
	}

	fpa->vfs_in_use -= vf_idx;
	mutex_unlock(&octeontx_fpa_devices_lock);
	return 0;
}

/* This will be the first call before using any VF
 * each usbale VF should be assigned a domain
 * Domain_id is the way to bind different VFs and resources together.
 * id: nodid
 * domain_id: domain id to bind different resources together
 * num_vfs: number of FPA vfs requested.
 * ret: will return bit mask of VFs assigned to this domain
 * on failure: returns 0
 *
 * Created domain also does the mappings for AURASET to GARUARASET
 */
static u64 fpa_pf_create_domain(u32 id, u16 domain_id,
				u32 num_vfs, struct kobject *kobj)
{
	int i, j, ret, aura, vf_idx = 0;
	struct fpapf *fpa = NULL;
	resource_size_t vf_start;
	struct pci_dev *virtfn;
	unsigned long aura_set = 0;
	struct fpapf *curr;
	u64 reg;

	mutex_lock(&octeontx_fpa_devices_lock);
	/* this loop is unnecessary as nodid is always 0 :: ask tirumalesh? */
	list_for_each_entry(curr, &octeontx_fpa_devices, list) {
		if (curr->id == id) {
			fpa = curr;
			break;
		}
	}
	if (!fpa) {
		mutex_unlock(&octeontx_fpa_devices_lock);
		return 0;
	}
	if ((fpa->total_vfs - fpa->vfs_in_use) < num_vfs) {
		mutex_unlock(&octeontx_fpa_devices_lock);
		return 0;
	}
	for (i = 0; i < fpa->total_vfs; i++) {
		if (fpa->vf[i].domain.in_use) {
			continue;
		} else {
			if (kobj) {
				virtfn =
				pci_get_domain_bus_and_slot(pci_domain_nr
							    (fpa->pdev->bus),
					   pci_iov_virtfn_bus(fpa->pdev, i),
					   pci_iov_virtfn_devfn(fpa->pdev, i));
				if (!virtfn)
					goto err_unlock;
				ret = sysfs_create_link(kobj, &virtfn->dev.kobj,
							virtfn->dev.kobj.name);
				if (ret < 0)
					goto err_unlock;
				ret = sysfs_create_file(&virtfn->dev.kobj,
							&pool_maxcnt_attr.attr);
				if (ret < 0)
					goto err_unlock;
				ret = sysfs_create_file(&virtfn->dev.kobj,
							&pool_curcnt_attr.attr);
				if (ret < 0)
					goto err_unlock;
				ret = sysfs_create_file(&virtfn->dev.kobj,
							&pool_redcnt_attr.attr);
				if (ret < 0)
					goto err_unlock;
			}
			fpa->vf[i].domain.domain_id = domain_id;
			fpa->vf[i].domain.subdomain_id = vf_idx;
			fpa->vf[i].domain.gmid = get_gmid(domain_id);

			fpa->vf[i].hardware_pool = get_pool(i);
			fpa->vf[i].hardware_aura_set = get_aura_set(i);

			vf_start = pci_resource_start(fpa->pdev,
						      PCI_FPA_PF_CFG_BAR);
			vf_start += FPA_VF_OFFSET(i);

			fpa->vf[i].domain.reg_base =
				ioremap_wc(vf_start, FPA_VF_CFG_SIZE);

			if (!fpa->vf[i].domain.reg_base)
				goto err_unlock;

			for (j = 0; j < FPA_AURA_SET_SIZE; j++) {
				aura = (i * FPA_AURA_SET_SIZE) + j;
				fpa_reg_write(fpa, FPA_PF_AURAX_POOL(aura), i);

				/* Disable aura red/bp lvl */
				fpa_reg_write(fpa, FPA_PF_AURAX_CNT_LEVELS(aura)
						, 1ul << 40);
			}

			reg = FPA_MAP_VALID(1) | FPA_MAP_VHAURASET(i) |
				FPA_MAP_GAURASET(fpa->vf[i].domain.subdomain_id)
				| FPA_MAP_GMID(fpa->vf[i].domain.gmid);
			fpa_reg_write(fpa, FPA_PF_MAPX(i), reg);

			if (domain_id != FPA_SSO_XAQ_AURA &&
			    domain_id != FPA_PKO_DPFI_AURA)
				reg = (0xff & (i + 1)) << 24 |
				      (0xff & (i + 1)) << 16;
			else
				reg = 0;

			fpa_reg_write(fpa, FPA_PF_VFX_GMCTL(i), reg);

			dev_dbg(&fpa->pdev->dev, "Alloc vf[%u] domain:%u subdomain_id:%u\n",
				i, domain_id, vf_idx);

			fpa->vf[i].domain.in_use = true;
			set_bit(i, &aura_set);
			identify(&fpa->vf[i], domain_id, vf_idx,
				 fpa->stack_ln_ptrs);
			vf_idx++;
			if (vf_idx == num_vfs) {
				fpa->vfs_in_use += num_vfs;
				break;
			}
		}
	}
	if (vf_idx != num_vfs)
		goto err_unlock;

	mutex_unlock(&octeontx_fpa_devices_lock);
	return aura_set;

err_unlock:
	mutex_unlock(&octeontx_fpa_devices_lock);
	fpa_pf_destroy_domain(id, domain_id, kobj);
	return 0;
}

static int fpa_pf_get_vf_count(u32 id)
{
	struct fpapf *fpa = NULL;
	struct fpapf *curr;

	mutex_lock(&octeontx_fpa_devices_lock);
	list_for_each_entry(curr, &octeontx_fpa_devices, list) {
		if (curr->id == id) {
			fpa = curr;
			break;
		}
	}
	if (!fpa) {
		mutex_unlock(&octeontx_fpa_devices_lock);
		return 0;
	}
	mutex_unlock(&octeontx_fpa_devices_lock);
	return fpa->total_vfs;
}

int fpa_reset_domain(u32 id, u16 domain_id)
{
	struct fpapf *fpa = NULL;
	struct fpapf *curr;
	int i, j, aura;
	u64 reg = 0;
	u64 addr;
	u64 avail;

	mutex_lock(&octeontx_fpa_devices_lock);
	list_for_each_entry(curr, &octeontx_fpa_devices, list) {
		if (curr->id == id) {
			fpa = curr;
			break;
		}
	}
	if (!fpa) {
		mutex_unlock(&octeontx_fpa_devices_lock);
		return 0;
	}

	for (i = 0; i < fpa->total_vfs; i++) {
		if (fpa->vf[i].domain.in_use &&
		    fpa->vf[i].domain.domain_id == domain_id) {
			avail = readq_relaxed(fpa->vf[i].domain.reg_base +
					FPA_VF_VHPOOL_AVAILABLE(0));
			if (avail <= 0)
				goto empty;

			/* wait until input queue is empty,
			 * this can take upto 100ms.
			 * this is a very bad way of doing it
			 * after saying this, there is no other
			 * way of doing a proper reset.
			 */
			while (1) {
				reg = fpa_reg_read(fpa, FPA_PF_STATUS);
				if (!(reg & 0x1))
					break;
			}

			for (j = 0; j < FPA_AURA_SET_SIZE; j++) {
				addr = fpa_vf_alloc(fpa->vf[i].domain.reg_base,
						    j);
				while (addr) {
					addr =
					fpa_vf_alloc(fpa->vf[i].domain.reg_base,
						     j);
				}
			}

empty:
			dev_dbg(&fpa->pdev->dev, "Reset vf[%u] domain:%u subdomain_id:%u\n",
				i, domain_id, fpa->vf[i].domain.subdomain_id);

			if (domain_id != FPA_SSO_XAQ_AURA &&
			    domain_id != FPA_PKO_DPFI_AURA)
				reg = (0xff & (i + 1)) << 24 |
				      (0xff & (i + 1)) << 16;
			fpa_reg_write(fpa, FPA_PF_VFX_GMCTL(i), reg);
			fpa_reg_write(fpa, FPA_PF_POOLX_CFG(i), 0x0);
			fpa_reg_write(fpa, FPA_PF_POOLX_STACK_BASE(i), 0x0);
			fpa_reg_write(fpa, FPA_PF_POOLX_STACK_END(i), 0x0);
			fpa_reg_write(fpa, FPA_PF_POOLX_STACK_ADDR(i), 0x0);
			fpa_reg_write(fpa, FPA_PF_POOLX_OP_PC(i), 0x0);
			fpa_reg_write(fpa, FPA_PF_POOLX_FPF_MARKS(i),
				      (0x80 << 16));

			for (j = 0; j < FPA_AURA_SET_SIZE; j++) {
				aura = (i * FPA_AURA_SET_SIZE) + j;
				fpa_reg_write(fpa,
					      FPA_PF_AURAX_POOL(aura), i);
				fpa_reg_write(fpa,
					      FPA_PF_AURAX_CFG(aura), 0x0);
				fpa_reg_write(fpa,
					      FPA_PF_AURAX_POOL_LEVELS(aura),
					      0x0);
				/* Disable aura red/bp lvl */
				fpa_reg_write(fpa,
					      FPA_PF_AURAX_CNT_LEVELS(aura),
					      1ul << 40);
			}
			writeq_relaxed(0xffffffffffffffffULL,
				       fpa->vf[i].domain.reg_base +
				       FPA_VF_VHPOOL_END_ADDR(0));
			identify(&fpa->vf[i], domain_id,
				 fpa->vf[i].domain.subdomain_id,
				 fpa->stack_ln_ptrs);
		}
	}

	mutex_unlock(&octeontx_fpa_devices_lock);
	return 0;
}

struct fpapf_com_s fpapf_com = {
	.create_domain = fpa_pf_create_domain,
	.destroy_domain = fpa_pf_destroy_domain,
	.reset_domain = fpa_reset_domain,
	.receive_message = fpa_pf_receive_message,
	.get_vf_count = fpa_pf_get_vf_count,
};
EXPORT_SYMBOL(fpapf_com);

static irqreturn_t fpa_pf_ecc_intr_handler (int irq, void *fpa_irq)
{
	struct fpapf *fpa = (struct fpapf *)fpa_irq;
	u64 ecc_int = fpa_reg_read(fpa, FPA_PF_ECC_INT);

	dev_err(&fpa->pdev->dev, "ECC errors recievd: %llx\n", ecc_int);
	/* clear ECC irq status */
	fpa_reg_write(fpa, FPA_PF_ECC_INT, ecc_int);

	return IRQ_HANDLED;
}

static irqreturn_t fpa_pf_gen_intr_handler (int irq, void *fpa_irq)
{
	struct fpapf *fpa = (struct fpapf *)fpa_irq;
	u64 gen_int = fpa_reg_read(fpa, FPA_PF_GEN_INT);
	u64 inp_ctl;
	u64 unmap_info;
	u32 gmid;
	u32 gaura;

	if (gen_int & FPA_GEN_INT_GMID0_MASK)
		dev_err(&fpa->pdev->dev,
			"allocate or free a buffer using GMID0\n");

	if (gen_int & FPA_GEN_INT_GMID_UNMAP_MASK) {
		unmap_info = fpa_reg_read(fpa, FPA_PF_UNMAP_INFO);

		gmid = (unmap_info & FPA_UNMAP_INFO_GMID_MASK) >>
			FPA_UNMAP_INFO_GMID_SHIFT;
		gaura = (unmap_info & FPA_UNMAP_INFO_GAURA_MASK) >>
			FPA_UNMAP_INFO_GAURA_SHIFT;
		dev_err(&fpa->pdev->dev,
			"GMID: 0x%x GAURA: 0x%x failed due to no map exist\n",
			gmid, gaura);
	}
	/* If both UNMAP_MASK and MULTI_MASK present
	 * at same time, the GMID and GAURA reported might not be accurate.
	 * The same is true if multiple instances of same error occurred.
	 * As thumb rule dont belive in GMID and GAURA reported.
	 */
	if (gen_int & FPA_GEN_INT_GMID_MULTI_MASK) {
		unmap_info = fpa_reg_read(fpa, FPA_PF_UNMAP_INFO);

		gmid = (unmap_info & FPA_UNMAP_INFO_GMID_MASK) >>
			FPA_UNMAP_INFO_GMID_SHIFT;
		gaura = (unmap_info & FPA_UNMAP_INFO_GAURA_MASK) >>
			FPA_UNMAP_INFO_GAURA_SHIFT;
		dev_err(&fpa->pdev->dev,
			"GMID: 0x%x GAURA: 0x%x has multiple maps\n", gmid,
			gaura);
	}

	inp_ctl = fpa_reg_read(fpa, FPA_PF_INP_CTL);

	if (gen_int & FPA_GEN_INT_FREE_DIS_MASK)
		dev_err(&fpa->pdev->dev,
			"Free request is dropped inp_ctl: %llx\n", inp_ctl);

	if (gen_int & FPA_GEN_INT_ALLOC_DIS_MASK)
		dev_err(&fpa->pdev->dev,
			"Alloc request is dropped inp_ctl: %llx\n", inp_ctl);

	fpa_reg_write(fpa, FPA_PF_GEN_INT, gen_int);
	return IRQ_HANDLED;
}

static int fpa_irq_init(struct fpapf *fpa)
{
	u64 ecc_int = ((0ULL | FPA_ECC_RAM_SBE_MASK) << FPA_ECC_RAM_SBE_SHIFT) |
		((0ULL | FPA_ECC_RAM_DBE_MASK) << FPA_ECC_RAM_DBE_SHIFT);
	u64 gen_int = 0x1f;
	int i, ret;

	/*clear ECC irq status */
	fpa_reg_write(fpa, FPA_PF_ECC_INT_ENA_W1C, ecc_int);
	/*clear GEN irq status */
	fpa_reg_write(fpa, FPA_PF_GEN_INT_ENA_W1C, gen_int);

	ret = FPA_PF_MSIX_COUNT;
	if (ret < 0) {
		dev_err(&fpa->pdev->dev, "Failed to get MSIX table size\n");
		return ret;
	}

	fpa->msix_entries = devm_kzalloc(&fpa->pdev->dev,
			ret * sizeof(struct msix_entry), GFP_KERNEL);
	if (!fpa->msix_entries)
		return -ENOMEM;
	for (i = 0; i < ret; i++)
		fpa->msix_entries[i].entry = i;

	ret = pci_enable_msix_exact(fpa->pdev, fpa->msix_entries, ret);
	if (ret < 0) {
		dev_err(&fpa->pdev->dev, "Enabling msix failed\n");
		return ret;
	}

	/* register ECC intr handler */
	ret = request_irq(fpa->msix_entries[0].vector, fpa_pf_ecc_intr_handler,
			  0, "fpapf ecc", fpa);
	if (ret)
		return ret;

	/* register GEN intr handler */
	ret = request_irq(fpa->msix_entries[1].vector, fpa_pf_gen_intr_handler,
			  0, "fpapf gen", fpa);
	if (ret)
		goto free_ecc_irq;

	/* it's time to enable interrupts*/
	fpa_reg_write(fpa, FPA_PF_ECC_INT_ENA_W1S, ecc_int);
	fpa_reg_write(fpa, FPA_PF_GEN_INT_ENA_W1S, gen_int);

	return 0;

free_ecc_irq:
	free_irq(fpa->msix_entries[0].vector, fpa);

	return ret;
}

static void fpa_irq_free(struct fpapf *fpa)
{
	u64 ecc_int = ((0ULL | FPA_ECC_RAM_SBE_MASK) << FPA_ECC_RAM_SBE_SHIFT) |
		((0ULL | FPA_ECC_RAM_DBE_MASK) << FPA_ECC_RAM_DBE_SHIFT);
	u64 gen_int = 0x1f;

	/*clear ECC irq status */
	fpa_reg_write(fpa, FPA_PF_ECC_INT_ENA_W1C, ecc_int);
	/*clear GEN irq status */
	fpa_reg_write(fpa, FPA_PF_GEN_INT_ENA_W1C, gen_int);

	free_irq(fpa->msix_entries[0].vector, fpa);
	free_irq(fpa->msix_entries[1].vector, fpa);

	pci_disable_msix(fpa->pdev);
}

static void fpa_init(struct fpapf *fpa)
{
	u64 gen_cfg = DEF_GEN_CFG_FLAGS;
	u64 fpa_reg;
	u32 max_maps;
	u32 max_auras;
	u32 max_pools;
	u32 stack_ln_ptrs;
	int i;

	/* RST FPA */
	fpa_reg_write(fpa, FPA_PF_SFT_RST, 0x1);
	/*A 100ms delay is required before reading the completion */
	msleep(100);
	while (fpa_reg_read(fpa, FPA_PF_SFT_RST))
		udelay(1);

	fpa_reg = fpa_reg_read(fpa, FPA_PF_CONST);
	max_pools = (fpa_reg >> FPA_CONST_POOLS_SHIFT) & FPA_CONST_POOLS_MASK;
	max_auras = (fpa_reg >> FPA_CONST_AURAS_SHIFT) & FPA_CONST_AURAS_MASK;
	stack_ln_ptrs = (fpa_reg >> FPA_CONST_STACK_LN_PTRS_SHIFT) &
		FPA_CONST_STACK_LN_PTRS_MASK;

	fpa_reg = fpa_reg_read(fpa, FPA_PF_CONST1);
	max_maps = (fpa_reg >> FPA_CONST1_MAPS_SHIFT) & FPA_CONST1_MAPS_MASK;

	/* set GEN CFG */
	fpa_reg_write(fpa, FPA_PF_GEN_CFG, gen_cfg);

	/* allow all coprocessors to use FPA */
	fpa_reg_write(fpa, FPA_PF_INP_CTL, 0x0);

	for (i = 0; i < max_maps; i++)
		fpa_reg_write(fpa, FPA_PF_MAPX(i), 0x0);

	/*initialize all pools to 0 */
	for (i = 0; i < max_pools; i++) {
		fpa_reg_write(fpa, FPA_PF_VFX_GMCTL(i), 0x0);
		fpa_reg_write(fpa, FPA_PF_POOLX_CFG(i), 0x0);
		fpa_reg_write(fpa, FPA_PF_POOLX_STACK_BASE(i), 0x0);
		fpa_reg_write(fpa, FPA_PF_POOLX_STACK_END(i), 0x0);
		fpa_reg_write(fpa, FPA_PF_POOLX_STACK_ADDR(i), 0x0);
		fpa_reg_write(fpa, FPA_PF_POOLX_OP_PC(i), 0x0);
		fpa_reg_write(fpa, FPA_PF_POOLX_FPF_MARKS(i), (0x80 << 16));
	}

	/* Initialize all AURAs to 0 */
	for (i = 0; i < max_auras; i++) {
		fpa_reg_write(fpa, FPA_PF_AURAX_POOL(i), 0x0);
		fpa_reg_write(fpa, FPA_PF_AURAX_CFG(i), 0x0);
		fpa_reg_write(fpa, FPA_PF_AURAX_POOL_LEVELS(i), 0x0);
		fpa_reg_write(fpa, FPA_PF_AURAX_CNT_LEVELS(i), 0x0);
	}

	fpa->stack_ln_ptrs = stack_ln_ptrs;

	dev_notice(&fpa->pdev->dev, "max_maps: %d max_pools: %d max_auras: %d\n",
		   max_maps, max_pools, max_auras);
}

static int fpa_sriov_configure(struct pci_dev *pdev, int numvfs)
{
	struct fpapf *fpa = pci_get_drvdata(pdev);
	int ret = -EBUSY;
	int disable = 0;

	if (fpa->vfs_in_use != 0)
		return ret;

	ret = 0;
	if (fpa->flags & FPA_SRIOV_ENABLED)
		disable = 1;

	if (disable) {
		pci_disable_sriov(pdev);
		fpa->flags &= ~FPA_SRIOV_ENABLED;
		fpa->total_vfs = 0;
	}

	if (numvfs > 0) {
		if (numvfs <= 16)
			numvfs = 16;
		else
			numvfs = 32;

		ret = pci_enable_sriov(pdev, numvfs);
		if (ret == 0) {
			fpa->flags |= FPA_SRIOV_ENABLED;
			fpa->total_vfs = numvfs;
			ret = numvfs;
		}
	}

	dev_notice(&fpa->pdev->dev, "VFs enabled: %d\n", ret);
	return ret;
}

static int fpa_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	struct device *dev = &pdev->dev;
	struct fpapf *fpa;
	int err = -ENOMEM;

	fpa = devm_kzalloc(dev, sizeof(*fpa), GFP_KERNEL);
	if (!fpa)
		return err;

	pci_set_drvdata(pdev, fpa);
	fpa->pdev = pdev;

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

	/*Map CFG registers */
	fpa->reg_base = pcim_iomap(pdev, PCI_FPA_PF_CFG_BAR, 0);
	if (!fpa->reg_base) {
		dev_err(dev, "Can't map CFG space\n");
		err = -ENOMEM;
		return err;
	}

	/*set FPA ID */
	fpa->id = atomic_add_return(1, &fpa_count);
	fpa->id -= 1;

	fpa_init(fpa);

	err = fpa_irq_init(fpa);
	if (err) {
		dev_err(dev, "failed init irqs\n");
		err = -EINVAL;
		return err;
	}

	err = fpa_sriov_configure(pdev, 16);
	if (err < 0) {
		dev_err(dev, "failed to configure sriov\n");
		fpa_irq_free(fpa);
		return err;
	}

	INIT_LIST_HEAD(&fpa->list);
	mutex_lock(&octeontx_fpa_devices_lock);
	list_add(&fpa->list, &octeontx_fpa_devices);
	mutex_unlock(&octeontx_fpa_devices_lock);

	return 0;
}

static void fpa_remove(struct pci_dev *pdev)
{
	struct device *dev = &pdev->dev;
	struct fpapf *fpa = pci_get_drvdata(pdev);
	struct fpapf *curr;

	if (!fpa)
		return;

	mutex_lock(&octeontx_fpa_devices_lock);
	list_for_each_entry(curr, &octeontx_fpa_devices, list) {
		if (curr == fpa) {
			list_del(&fpa->list);
			break;
		}
	}
	mutex_unlock(&octeontx_fpa_devices_lock);

	fpa_irq_free(fpa);
	fpa_sriov_configure(pdev, 0);
	pci_release_regions(pdev);
	pci_disable_device(pdev);
	pci_set_drvdata(pdev, NULL);
	devm_kfree(dev, fpa);
}

/* devices supported */
static const struct pci_device_id fpa_id_table[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_CAVIUM, PCI_DEVICE_ID_OCTEONTX_FPA_PF) },
	{ 0, } /* end of table */
};

static struct pci_driver fpa_driver = {
	.name = DRV_NAME,
	.id_table = fpa_id_table,
	.probe = fpa_probe,
	.remove = fpa_remove,
};

MODULE_AUTHOR("Tirumalesh Chalamarla");
MODULE_DESCRIPTION("Cavium OCTEONTX FPA Physical Function Driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRV_VERSION);
MODULE_DEVICE_TABLE(pci, fpa_id_table);

static int __init fpa_init_module(void)
{
	pr_info("%s, ver %s\n", DRV_NAME, DRV_VERSION);

	return pci_register_driver(&fpa_driver);
}

static void __exit fpa_cleanup_module(void)
{
	pci_unregister_driver(&fpa_driver);
}

module_init(fpa_init_module);
module_exit(fpa_cleanup_module);
