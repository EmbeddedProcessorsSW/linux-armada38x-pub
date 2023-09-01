// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2016 Cavium, Inc.
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

#include "sso.h"
#include "fpa.h"
#include "rst.h"

#define DRV_NAME "octeontx-sso"
#define DRV_VERSION "1.0"

static atomic_t sso_count = ATOMIC_INIT(0);
static DEFINE_MUTEX(octeontx_sso_devices_lock);
static LIST_HEAD(octeontx_sso_devices);
static DEFINE_MUTEX(pf_mbox_lock);

static struct fpapf_com_s *fpapf;
static struct fpavf_com_s *fpavf;
static struct fpavf *fpa;
static struct rst_com_s *rst;

void *ram_mbox_buf;	/* Temp backing buff for rambox */

static unsigned int max_grps = 32;
module_param(max_grps, uint, 0444);
MODULE_PARM_DESC(max_grps,
		 "Limit the number of sso groups(0=maximum number of groups");
static unsigned int max_events = 100000;
module_param(max_events, int, 0444);
MODULE_PARM_DESC(max_events,
		 "Number of DRAM event entries");

#define MAX_SSO_RST_TIMEOUT_US  3000

#define CLKS2NSEC(c, sclk_freq)		((c) * NSEC_PER_SEC / (sclk_freq))
#define NSEC2CLKS(ns, sclk_freq)	(((ns) * (sclk_freq)) / NSEC_PER_SEC)

#define MIN_NW_TIM_CLK	1024
#define MAX_NW_TIM_CLK	(1024 * 1024)	/* nw_tim is 10 bit wide ~ 1024*/

/* In Cavium OcteonTX SoCs, all accesses to the device registers are
 * implicitly strongly ordered.
 * So writeq_relaxed() and readq_relaxed() are safe to use
 * with out any memory barriers.
 */

/* Register read/write APIs */
static void sso_reg_write(struct ssopf *sso, u64 offset, u64 val)
{
	writeq_relaxed(val, sso->reg_base + offset);
}

static u64 sso_reg_read(struct ssopf *sso, u64 offset)
{
	return readq_relaxed(sso->reg_base + offset);
}

/* Caller is responsible for locks */
static struct ssopf_vf *get_vf(struct ssopf *sso, u16 domain_id,
			       u16 subdomain_id, size_t *vf_idx)

{
	size_t i;

	for (i = 0; i < sso->total_vfs; i++) {
		if (sso->vf[i].domain.in_use &&
		    sso->vf[i].domain.domain_id == domain_id &&
		    sso->vf[i].domain.subdomain_id == subdomain_id) {
			if (vf_idx)
				*vf_idx = i;
			return &sso->vf[i];
		}
	}

	return NULL;
}

static u64 sso_pf_min_tmo(u8 node)
{
	u64 ns, sclk_freq;

	/* Get SCLK */
	sclk_freq = rst->get_sclk_freq(node);

	/* Convert min_nw_tim to ns */
	ns = CLKS2NSEC(MIN_NW_TIM_CLK, sclk_freq);

	return ns;
}

static u64 sso_pf_max_tmo(u8 node)
{
	u64 ns, sclk_freq;

	/* Get SCLK */
	sclk_freq = rst->get_sclk_freq(node);

	/* Convert max_nw_tim to ns */
	ns = CLKS2NSEC(MAX_NW_TIM_CLK, sclk_freq);

	return ns;
}

static u64 sso_pf_get_tmo(struct ssopf *sso)
{
	u64 ns, sclk_freq;
	u64 nw_clk;

	/* Get current tick */
	nw_clk = sso_reg_read(sso, SSO_PF_NW_TIM) & 0x3ff;
	nw_clk += 1;

	/* Conevrt from set-Bit to multiple of 1024 clock cycles
	 * Refer HRM: SSO_NW_TIM
	 */
	nw_clk <<= 10;
	/* Get SCLK */
	sclk_freq = rst->get_sclk_freq(sso->id);

	/* Convert current tick to ns */
	ns = CLKS2NSEC(nw_clk, sclk_freq);
	return ns;
}

static void sso_pf_set_tmo(struct ssopf *sso, u64 ns)
{
	u64 sclk_freq, nw_clk;

	/* Get SCLK */
	sclk_freq = rst->get_sclk_freq(sso->id);

	/* Transalate nsec to clock */
	nw_clk = NSEC2CLKS(ns, sclk_freq);
	/* Conevrt from set-Bit to multiple of 1024 clock cycles
	 * Refer HRM: SSO_NW_TIM
	 */
	nw_clk >>= 10;
	if (nw_clk)
		nw_clk -= 1;

	/* write new clk value to Bit pos 9:0 of SSO_NW_TIM */
	sso_reg_write(sso, SSO_PF_NW_TIM, nw_clk & 0x3ff);
}

static u32 sso_pf_ns_to_iter(struct ssopf *sso, u32 wait_ns)
{
	u64 sclk_freq, new_tmo, cur_tmo;
	u32 getwork_iter;

	/* Get SCLK */
	sclk_freq = rst->get_sclk_freq(sso->id);

	/* Transalate nsec to clock */
	new_tmo = NSEC2CLKS(wait_ns, sclk_freq);

	/*Get NW_TIM clock and translate to sclk_freq */
	cur_tmo = sso_reg_read(sso, SSO_PF_NW_TIM) & 0x3ff;
	cur_tmo += 1;
	/* Conevrt from set-Bit to multiple of 1024 clock cycles
	 * Refer HRM: SSO_NW_TIM
	 */
	cur_tmo <<= 10;

	getwork_iter = new_tmo / cur_tmo;
	if (!getwork_iter)
		getwork_iter = 1;

	return getwork_iter;
}

static void identify(struct ssopf_vf *vf, u16 domain_id, u16 subdomain_id)
{
	u64 reg = (((u64)subdomain_id << 16) | domain_id);

	writeq_relaxed(reg, vf->domain.reg_base + SSO_VF_VHGRPX_AQ_THR(0));
}

int ssopf_master_send_message(struct mbox_hdr *hdr,
			      union mbox_data *req,
			      union mbox_data *resp,
			      void *master_data,
			      void *add_data)
{
	struct ssopf *sso = master_data;
	int ret;

	if (hdr->coproc == FPA_COPROC) {
		ret = fpapf->receive_message(sso->id, FPA_SSO_XAQ_GMID, hdr,
					     req, resp, add_data);
	} else {
		dev_err(&sso->pdev->dev, "SSO messahe dispatch, wrong VF type\n");
		ret = -1;
	}

	return ret;
}

static struct octeontx_master_com_t sso_master_com = {
	.send_message = ssopf_master_send_message,
};

static ssize_t group_work_sched_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct ssopf *curr, *sso = NULL;
	struct pci_dev *pdev = container_of(dev, struct pci_dev, dev);
	int vfid = pdev->devfn;
	u64 cnt;

	list_for_each_entry(curr, &octeontx_sso_devices, list) {
		if (curr->pdev == pdev->physfn) {
			sso = curr;
			break;
		}
	}
	if (!sso)
		return 0;

	cnt = readq_relaxed(sso->reg_base + SSO_PF_GRPX_WS_PC(vfid));
	return snprintf(buf, PAGE_SIZE, "%lld\n", cnt);
}

static struct device_attribute group_work_sched_attr = {
	.attr = {.name = "work_sched",  .mode = 0444},
	.show = group_work_sched_show,
	.store = NULL
};

static ssize_t group_work_admit_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct ssopf *curr, *sso = NULL;
	struct pci_dev *pdev = container_of(dev, struct pci_dev, dev);
	int vfid = pdev->devfn;
	u64 cnt;

	list_for_each_entry(curr, &octeontx_sso_devices, list) {
		if (curr->pdev == pdev->physfn) {
			sso = curr;
			break;
		}
	}
	if (!sso)
		return 0;

	cnt = readq_relaxed(sso->reg_base + SSO_PF_GRPX_WA_PC(vfid));
	return snprintf(buf, PAGE_SIZE, "%lld\n", cnt);
}

static struct device_attribute group_work_admit_attr = {
	.attr = {.name = "work_admit",  .mode = 0444},
	.show = group_work_admit_show,
	.store = NULL
};

static int sso_pf_destroy_domain(u32 id, u16 domain_id, struct kobject *kobj)
{
	struct ssopf *sso = NULL;
	struct pci_dev *virtfn;
	struct ssopf *curr;
	int i, vf_idx;
	u64 reg;

	vf_idx = 0;
	reg = 0;
	mutex_lock(&octeontx_sso_devices_lock);
	list_for_each_entry(curr, &octeontx_sso_devices, list) {
		if (curr->id == id) {
			sso = curr;
			break;
		}
	}

	if (!sso) {
		mutex_unlock(&octeontx_sso_devices_lock);
		return -ENODEV;
	}

	for (i = 0; i < sso->total_vfs; i++) {
		if (sso->vf[i].domain.in_use &&
		    sso->vf[i].domain.domain_id == domain_id) {
			sso->vf[i].domain.master_data = NULL;
			sso->vf[i].domain.master = NULL;
			sso->vf[i].domain.in_use = false;

			virtfn = pci_get_domain_bus_and_slot(pci_domain_nr
					(sso->pdev->bus),
					pci_iov_virtfn_bus(sso->pdev, i),
					pci_iov_virtfn_devfn(sso->pdev, i));
			if (virtfn && kobj) {
				sysfs_remove_file(&virtfn->dev.kobj,
						  &group_work_admit_attr.attr);
				sysfs_remove_file(&virtfn->dev.kobj,
						  &group_work_sched_attr.attr);
				sysfs_remove_link(kobj, virtfn->dev.kobj.name);
			}
			dev_info(&sso->pdev->dev,
				 "Free vf[%d] from domain:%d subdomain_id:%d\n",
				 i, sso->vf[i].domain.domain_id, vf_idx);

			/* Unmap groups */
			reg = SSO_MAP_VALID(0) | SSO_MAP_VHGRP(i) |
				SSO_MAP_GGRP(0) |
				SSO_MAP_GMID(sso->vf[i].domain.gmid);
			sso_reg_write(sso, SSO_PF_MAPX(i), reg);

			identify(&sso->vf[i], 0xFFFF, 0xFFFF);
			iounmap(sso->vf[i].domain.reg_base);
			vf_idx++;
		}
	}

	sso->vfs_in_use -= vf_idx;
	mutex_unlock(&octeontx_sso_devices_lock);
	return 0;
}

static u64 sso_pf_create_domain(u32 id, u16 domain_id,
				u32 num_grps, void *master, void *master_data,
				struct kobject *kobj)
{
	struct ssopf *sso = NULL;
	struct ssopf *curr;
	struct pci_dev *virtfn;
	resource_size_t vf_start;
	u64 i, reg = 0;
	unsigned long grp_mask = 0;
	int ret = 0, vf_idx = 0;

	if (!kobj)
		return 0;

	mutex_lock(&octeontx_sso_devices_lock);
	list_for_each_entry(curr, &octeontx_sso_devices, list) {
		if (curr->id == id) {
			sso = curr;
			break;
		}
	}
	if (!sso) {
		mutex_unlock(&octeontx_sso_devices_lock);
		return 0;
	}

	for (i = 0; i < sso->total_vfs; i++) {
		if (sso->vf[i].domain.in_use) {
			continue;
		} else {
			virtfn = pci_get_domain_bus_and_slot(pci_domain_nr
					(sso->pdev->bus),
					pci_iov_virtfn_bus(sso->pdev, i),
					pci_iov_virtfn_devfn(sso->pdev, i));
			if (!virtfn)
				goto err_unlock;
			ret = sysfs_create_link(kobj, &virtfn->dev.kobj,
						virtfn->dev.kobj.name);
			if (ret < 0)
				goto err_unlock;
			ret = sysfs_create_file(&virtfn->dev.kobj,
						&group_work_sched_attr.attr);
			if (ret < 0)
				goto err_unlock;
			ret = sysfs_create_file(&virtfn->dev.kobj,
						&group_work_admit_attr.attr);
			if (ret < 0)
				goto err_unlock;

			sso->vf[i].domain.domain_id = domain_id;
			sso->vf[i].domain.subdomain_id = vf_idx;
			sso->vf[i].domain.gmid = get_gmid(domain_id);

			sso->vf[i].domain.in_use = 1;
			sso->vf[i].domain.master = master;
			sso->vf[i].domain.master_data = master_data;

			/* Map num_grps resources
			 * - Assumes all the ggrp belong to one domain
			 */
			reg = SSO_MAP_VALID(1ULL) |
			     SSO_MAP_VHGRP(i) |
			     SSO_MAP_GGRP(sso->vf[i].domain.subdomain_id) |
			     SSO_MAP_GMID(sso->vf[i].domain.gmid);
			sso_reg_write(sso, SSO_PF_MAPX(i), reg);

			/* Configure default prio that can later be changed by
			 * SSO_GRP_SET_PRIORITY call from userspace
			 */
			reg = ((0x3fULL) << 16) | ((0xfULL) << 8) | (0ULL);
			sso_reg_write(sso, SSO_PF_GRPX_PRI(i), reg);
			vf_start = pci_resource_start(sso->pdev,
						      PCI_SSO_PF_CFG_BAR);
			vf_start += SSO_VF_OFFSET(i);

			sso->vf[i].domain.reg_base =
				ioremap_wc(vf_start, SSO_VF_CFG_SIZE);

			/* write did/sdid in temp register for vf probe
			 * to get to know his vf_idx/subdomainid
			 * this mechanism is simmilar to all VF types
			 */
			identify(&sso->vf[i], domain_id, vf_idx);

			mbox_init(&sso->vf[i].mbox,
				  sso->reg_base + SSO_PF_VHGRPX_MBOX(i, 0),
				  NULL, 0,
				  MBOX_SIDE_PF);

			vf_idx++;
			set_bit(i, &grp_mask);
			if (vf_idx == num_grps) {
				sso->vfs_in_use += num_grps;
				break;
			}
		}
	}
	if (vf_idx != num_grps)
		goto err_unlock;

	mutex_unlock(&octeontx_sso_devices_lock);
	return grp_mask;

err_unlock:
	mutex_unlock(&octeontx_sso_devices_lock);
	sso_pf_destroy_domain(id, domain_id, kobj);
	return 0;
}

static int sso_pf_send_message(u32 id, u16 domain_id,
			       struct mbox_hdr *hdr,
			       union mbox_data *req, union mbox_data *resp)
{
	struct ssopf *sso = NULL;
	struct ssopf *curr;
	int i;
	int vf_idx = -1;
	int ret;

	mutex_lock(&octeontx_sso_devices_lock);
	list_for_each_entry(curr, &octeontx_sso_devices, list) {
		if (curr->id == id) {
			sso = curr;
			break;
		}
	}

	if (!sso) {
		mutex_unlock(&octeontx_sso_devices_lock);
		return -ENODEV;
	}

	/* locate the SSO VF master of domain (vf_idx == 0) */
	for (i = 0; i < sso->total_vfs; i++) {
		if (sso->vf[i].domain.in_use &&
		    sso->vf[i].domain.domain_id == domain_id &&
		    sso->vf[i].domain.subdomain_id == 0) {
			vf_idx = i;
			break;
		}
	}

	mutex_unlock(&octeontx_sso_devices_lock);

	if (vf_idx == -1)
		return -ENODEV; /* SSOVF for domain not found */

	ret = mbox_send(&sso->vf[vf_idx].mbox, hdr, req, sizeof(*req), resp,
			sizeof(*resp));
	if (ret < 0) {
		dev_err(&sso->pdev->dev, "Error durring MBOX transmsion\n");
		return ret;
	}

	return 0;
}

static int sso_pf_set_mbox_ram(u32 node, u16 domain_id,
			       void *mbox_addr, u64 mbox_size)
{
	struct ssopf *sso = NULL;
	struct ssopf *curr;
	size_t i, vf_idx = -1;

	if (!mbox_addr || !mbox_size)
		return -EINVAL;

	mutex_lock(&octeontx_sso_devices_lock);
	list_for_each_entry(curr, &octeontx_sso_devices, list) {
		if (curr->id == node) {
			sso = curr;
			break;
		}
	}

	if (!sso) {
		mutex_unlock(&octeontx_sso_devices_lock);
		return -ENODEV;
	}

	/* locate the SSO VF master of domain (vf_idx == 0) */
	for (i = 0; i < sso->total_vfs; i++) {
		if (sso->vf[i].domain.in_use &&
		    sso->vf[i].domain.domain_id == domain_id &&
		    sso->vf[i].domain.subdomain_id == 0) {
			vf_idx = i;
			break;
		}
	}

	mutex_unlock(&octeontx_sso_devices_lock);

	if (vf_idx < 0)
		return -ENODEV; /* SSOVF for domain not found */

	mbox_init(&sso->vf[i].mbox,
		  sso->reg_base + SSO_PF_VHGRPX_MBOX(vf_idx, 0),
		  mbox_addr, mbox_size,
		  MBOX_SIDE_PF);

	return 0;
}

static int sso_pf_get_vf_count(u32 id)
{
	struct ssopf *sso = NULL;
	struct ssopf *curr;

	mutex_lock(&octeontx_sso_devices_lock);
	list_for_each_entry(curr, &octeontx_sso_devices, list) {
		if (curr->id == id) {
			sso = curr;
			break;
		}
	}

	if (!sso) {
		mutex_unlock(&octeontx_sso_devices_lock);
		return 0;
	}

	mutex_unlock(&octeontx_sso_devices_lock);
	return sso->total_vfs;
}

int sso_reset_domain(u32 id, u16 domain_id)
{
	struct ssopf *sso = NULL;
	struct ssopf *curr;
	int i;

	mutex_lock(&octeontx_sso_devices_lock);
	list_for_each_entry(curr, &octeontx_sso_devices, list) {
		if (curr->id == id) {
			sso = curr;
			break;
		}
	}

	if (!sso) {
		mutex_unlock(&octeontx_sso_devices_lock);
		return -EINVAL;
	}

	for (i = 0; i < sso->total_vfs; i++) {
		if (sso->vf[i].domain.in_use &&
		    sso->vf[i].domain.domain_id == domain_id) {
			identify(&sso->vf[i], domain_id,
				 sso->vf[i].domain.subdomain_id);
		}
	}

	mutex_unlock(&octeontx_sso_devices_lock);
	return 0;
}

int sso_pf_set_value(u32 id, u64 offset, u64 val)
{
	struct ssopf *sso = NULL;
	struct ssopf *curr;

	mutex_lock(&octeontx_sso_devices_lock);
	list_for_each_entry(curr, &octeontx_sso_devices, list) {
		if (curr->id == id) {
			sso = curr;
			break;
		}
	}
	if (!sso) {
		mutex_unlock(&octeontx_sso_devices_lock);
		return -EINVAL;
	}
	sso_reg_write(sso, offset, val);
	mutex_unlock(&octeontx_sso_devices_lock);
	return 0;
}
EXPORT_SYMBOL(sso_pf_set_value);

int sso_pf_get_value(u32 id, u64 offset, u64 *val)
{
	struct ssopf *sso = NULL;
	struct ssopf *curr;

	mutex_lock(&octeontx_sso_devices_lock);
	list_for_each_entry(curr, &octeontx_sso_devices, list) {
		if (curr->id == id) {
			sso = curr;
			break;
		}
	}
	if (!sso) {
		mutex_unlock(&octeontx_sso_devices_lock);
		return -EINVAL;
	}
	*val = sso_reg_read(sso, offset);
	mutex_unlock(&octeontx_sso_devices_lock);
	return 0;
}
EXPORT_SYMBOL(sso_pf_get_value);

int sso_vf_get_value(u32 id, int vf_id, u64 offset, u64 *val)
{
	struct ssopf *sso = NULL;
	struct ssopf *curr;

	mutex_lock(&octeontx_sso_devices_lock);
	list_for_each_entry(curr, &octeontx_sso_devices, list) {
		if (curr->id == id) {
			sso = curr;
			break;
		}
	}
	if (!sso) {
		mutex_unlock(&octeontx_sso_devices_lock);
		return -EINVAL;
	}

	*val = readq_relaxed((sso->vf[vf_id].domain.reg_base + offset));
	mutex_unlock(&octeontx_sso_devices_lock);
	return 0;
}
EXPORT_SYMBOL(sso_vf_get_value);

struct ssopf_com_s ssopf_com = {
	.create_domain = sso_pf_create_domain,
	.destroy_domain = sso_pf_destroy_domain,
	.reset_domain = sso_reset_domain,
	.send_message = sso_pf_send_message,
	.set_mbox_ram = sso_pf_set_mbox_ram,
	.get_vf_count = sso_pf_get_vf_count
};
EXPORT_SYMBOL(ssopf_com);

static int handle_mbox_msg_from_sso_vf(struct ssopf *sso,
				       u16 domain_id,
				       struct mbox_hdr *hdr,
				       union mbox_data *req,
				       union mbox_data *resp,
				       void *add_data)
{
	struct ssopf_vf *vf;
	struct mbox_sso_get_dev_info *get_dev_info = NULL;
	struct mbox_sso_getwork_wait *getwork_wait = NULL;
	struct mbox_sso_convert_ns_getworks_iter *ns_to_getworks_iter = NULL;
	struct mbox_sso_grp_priority *grp_prio = NULL;
	u64 reg;
	size_t vf_idx;
	int ret = -1;

	hdr->res_code = MBOX_RET_INVALID;
	resp->data = 0;

	vf = get_vf(sso, domain_id, hdr->vfid, &vf_idx);
	/* hdr->vfid could be out of range, use common error response */
	if (!vf)
		return -1;

	switch (hdr->msg) {
	case IDENTIFY:
		/* test code only */
		identify(vf, vf->domain.domain_id, vf->domain.subdomain_id);
		hdr->res_code = MBOX_RET_SUCCESS;
		ret = 0;
		break;
	case SSO_GET_DEV_INFO:
		get_dev_info = add_data;

		get_dev_info->min_getwork_wait_ns = sso_pf_min_tmo(sso->id);
		get_dev_info->max_getwork_wait_ns = sso_pf_max_tmo(sso->id);
		get_dev_info->max_events = max_events;

		hdr->res_code = MBOX_RET_SUCCESS;
		/* update len */
		resp->data = sizeof(struct mbox_sso_get_dev_info);
		ret = 0;
		break;
	case SSO_GET_GETWORK_WAIT:
		getwork_wait = add_data;

		getwork_wait->wait_ns = sso_pf_get_tmo(sso);

		hdr->res_code = MBOX_RET_SUCCESS;
		/* update len */
		resp->data = sizeof(*getwork_wait);
		ret = 0;
		break;
	case SSO_SET_GETWORK_WAIT:
		getwork_wait = add_data;

		sso_pf_set_tmo(sso, getwork_wait->wait_ns);

		hdr->res_code = MBOX_RET_SUCCESS;
		/* update len */
		resp->data = 0;
		ret = 0;
		break;
	case SSO_CONVERT_NS_GETWORK_ITER:
		ns_to_getworks_iter = add_data;

		ns_to_getworks_iter->getwork_iter =
		sso_pf_ns_to_iter(sso, ns_to_getworks_iter->wait_ns);

		hdr->res_code = MBOX_RET_SUCCESS;
		/* update len */
		resp->data = sizeof(*ns_to_getworks_iter);
		ret = 0;
		break;
	case SSO_GRP_GET_PRIORITY:
		/* NOTE: Until pf_mapping make way into pf driver
		 * ,. Follow simple mapping approach ie.. vhgrp = pool = vf_id
		 * Next change set for pf_mapping will address mapping.(Todo)
		 */
		grp_prio = add_data;

		reg = sso_reg_read(sso, SSO_PF_GRPX_PRI(vf_idx));

		/* now update struct _grp_priority fields {} */
		grp_prio->vhgrp_id = vf_idx;
		grp_prio->wgt_left = (reg >> 24) & 0x3f;
		grp_prio->weight = (reg >> 16) & 0x3f;
		grp_prio->affinity = (reg >> 8) & 0xf;
		grp_prio->pri = reg & 0x7;

		hdr->res_code = MBOX_RET_SUCCESS;
		/* update len */
		resp->data = sizeof(*grp_prio);
		ret = 0;
		break;
	case SSO_GRP_SET_PRIORITY:
		grp_prio = add_data;

		reg = 0;
		reg = (((u64)(grp_prio->wgt_left & 0x3f) << 24) |
			((u64)(grp_prio->weight & 0x3f) << 16) |
			((u64)(grp_prio->affinity & 0xf) << 8) |
			(grp_prio->pri & 0x7));

		sso_reg_write(sso, SSO_PF_GRPX_PRI(vf_idx), reg);

		hdr->res_code = MBOX_RET_SUCCESS;
		/* update len */
		resp->data = 0;
		ret = 0;
		break;
	default:
		dev_err(&sso->pdev->dev, "invalid mbox message to sso\n");
		ret = -1; /* use common error resp->nse */
		break;
	}

	return ret;
}

static void handle_mbox_msg_from_vf(struct ssopf *sso, int vf_idx)
{
	struct mbox_hdr hdr = {0};
	union mbox_data resp;
	union mbox_data req;
	int req_size;
	int ret = 0;
	const void *replymsg = NULL;
	size_t replysize;

	req_size = mbox_receive(&sso->vf[vf_idx].mbox, &hdr, ram_mbox_buf,
				MBOX_MAX_MSG_SIZE);
	if (req_size < 0) {
		dev_dbg(&sso->pdev->dev,
			"MBox return no message, spurious IRQ?\n");
		return;
	}

	/* For SSO_VF inactive or other than subdomain_id=0,
	 * we skip the message processing
	 * We still reply with error
	 */
	if (!sso->vf[vf_idx].domain.in_use ||
	    sso->vf[vf_idx].domain.subdomain_id != 0) {
		replymsg = NULL;
		replysize = 0;
		ret = -1;
		goto send_resp;
	}

	resp.data = 0;
	switch (hdr.coproc) {
	case SSO_COPROC:
		if (hdr.msg != SSO_GETDOMAINCFG) {
			memcpy(&req, ram_mbox_buf, sizeof(req));
			ret = handle_mbox_msg_from_sso_vf(sso,
					sso->vf[vf_idx].domain.domain_id,
					&hdr, &req /* Unused for sso */, &resp,
					ram_mbox_buf);
			/* prep for replymsg */
			replymsg = ram_mbox_buf;
			replysize = resp.data;
		}
		break;
	default:
		/* call octtx_master_receive_message for msg dispatch */
		ret = sso->vf[vf_idx].domain.master->receive_message(&hdr, &req,
				&resp, sso->vf[vf_idx].domain.master_data,
				ram_mbox_buf);

		/* prep for replymsg */
		replymsg = ram_mbox_buf;
		replysize = resp.data;
		break;
	}
	/* Note:
	 * resp.data = 0 --> set operation by drv
	 * resp.data > 0 --> get operation by drv
	 */
	if (resp.data >= MBOX_MAX_MSG_SIZE) {
		ret = -1;
		replymsg = NULL;
		replysize = 0;
	}
	dev_dbg(&sso->pdev->dev, "print replymsg %p replymsg_data %llx replysize %zu\n",
		replymsg, *(u64 *)replymsg, replysize);

send_resp:
	if (mbox_reply(&sso->vf[vf_idx].mbox,
		       ret ? MBOX_RET_INVALID : hdr.res_code,
		       replymsg, replysize))
		dev_err(&sso->pdev->dev, "MBox error on PF response\n");
}

static void handle_mbox(struct work_struct *wq)
{
	struct ssopf *sso = container_of(wq, struct ssopf, mbox_work);
	u64 reg;
	int vf_idx;

	mutex_lock(&pf_mbox_lock);
	while (true) {
		reg = sso_reg_read(sso, SSO_PF_MBOX_INT);
		if (!reg)
			break;

		for_each_set_bit(vf_idx, (unsigned long *)&reg,
				 sizeof(reg) * 8) {
			/* SSO VF should handle msg only if it is a domain
			 * master
			 */
			if (sso->vf[vf_idx].domain.in_use)
				handle_mbox_msg_from_vf(sso, vf_idx);
		}

		sso_reg_write(sso, SSO_PF_MBOX_INT, reg);
	}
	mutex_unlock(&pf_mbox_lock);
}

static irqreturn_t sso_pf_mbox_intr_handler (int irq, void *sso_irq)
{
	struct ssopf *sso = (struct ssopf *)sso_irq;

	/* schedule work and be done*/
	schedule_work(&sso->mbox_work);

	return IRQ_HANDLED;
}

static irqreturn_t sso_pf_err_intr_handler (int irq, void *sso_irq)
{
	struct ssopf *sso = (struct ssopf *)sso_irq;
	u64 sso_reg;

	dev_err(&sso->pdev->dev, "errors received\n");
	sso_reg = sso_reg_read(sso, SSO_PF_ERR0);
	dev_err(&sso->pdev->dev, "err0:%llx\n", sso_reg);
	sso_reg = sso_reg_read(sso, SSO_PF_ERR1);
	dev_err(&sso->pdev->dev, "err1:%llx\n", sso_reg);
	sso_reg = sso_reg_read(sso, SSO_PF_ERR2);
	dev_err(&sso->pdev->dev, "err2:%llx\n", sso_reg);

	sso_reg = sso_reg_read(sso, SSO_PF_UNMAP_INFO);
	dev_err(&sso->pdev->dev, "unamp_info:%llx\n", sso_reg);
	sso_reg = sso_reg_read(sso, SSO_PF_UNMAP_INFO2);
	dev_err(&sso->pdev->dev, "unamp_info2:%llx\n", sso_reg);

	/* clear all interrupts*/
	sso_reg = SSO_ERR0;
	sso_reg_write(sso, SSO_PF_ERR0, sso_reg);
	sso_reg = SSO_ERR1;
	sso_reg_write(sso, SSO_PF_ERR1, sso_reg);
	sso_reg = SSO_ERR2;
	sso_reg_write(sso, SSO_PF_ERR2, sso_reg);
	return IRQ_HANDLED;
}

static int sso_irq_init(struct ssopf *sso)
{
	int ret = 0;
	u64 sso_reg;
	int i;

	/* clear all interrupts*/
	sso_reg = SSO_ERR0;
	sso_reg_write(sso, SSO_PF_ERR0, sso_reg);
	sso_reg = SSO_ERR1;
	sso_reg_write(sso, SSO_PF_ERR1, sso_reg);
	sso_reg = SSO_ERR2;
	sso_reg_write(sso, SSO_PF_ERR2, sso_reg);
	sso_reg = SSO_MBOX;
	sso_reg_write(sso, SSO_PF_MBOX_INT, sso_reg);

	/*clear all ena */
	sso_reg = SSO_ERR0;
	sso_reg_write(sso, SSO_PF_ERR0_ENA_W1C, sso_reg);
	sso_reg = SSO_ERR1;
	sso_reg_write(sso, SSO_PF_ERR1_ENA_W1C, sso_reg);
	sso_reg = SSO_ERR2;
	sso_reg_write(sso, SSO_PF_ERR2_ENA_W1C, sso_reg);
	sso_reg = SSO_MBOX;
	sso_reg_write(sso, SSO_PF_MBOX_ENA_W1C, sso_reg);

	sso->msix_entries = devm_kzalloc(&sso->pdev->dev, SSO_PF_MSIX_COUNT *
					 sizeof(struct msix_entry), GFP_KERNEL);
	if (!sso->msix_entries)
		return -ENOMEM;
	for (i = 0; i < SSO_PF_MSIX_COUNT; i++)
		sso->msix_entries[i].entry = i;

	ret = pci_enable_msix_exact(sso->pdev, sso->msix_entries,
				    SSO_PF_MSIX_COUNT);
	if (ret < 0) {
		dev_err(&sso->pdev->dev, "Enabling msix failed(%d)\n", ret);
		return ret;
	}

	/* register ERR intr handler */
	for (i = 0; i < (SSO_PF_MSIX_COUNT - 1); i++) {
		ret = request_irq(sso->msix_entries[i].vector,
				  sso_pf_err_intr_handler, 0, "ssopf err", sso);
		if (ret)
			goto free_irq;
	}

	/* register MBOX intr handler */
	ret = request_irq(sso->msix_entries[i].vector, sso_pf_mbox_intr_handler,
			  0, "ssopf mbox", sso);
	if (ret)
		goto free_irq;

	/*Enable all intr */
	sso_reg = SSO_ERR0;
	sso_reg_write(sso, SSO_PF_ERR0_ENA_W1S, sso_reg);
	sso_reg = SSO_ERR1;
	sso_reg_write(sso, SSO_PF_ERR1_ENA_W1S, sso_reg);
	sso_reg = SSO_ERR2;
	sso_reg_write(sso, SSO_PF_ERR2_ENA_W1S, sso_reg);
	sso_reg = SSO_MBOX;
	sso_reg_write(sso, SSO_PF_MBOX_ENA_W1S, sso_reg);

	return 0;

free_irq:
	while (i) {
		free_irq(sso->msix_entries[i - 1].vector, sso);
		i--;
	}

	return ret;
}

static void sso_irq_free(struct ssopf *sso)
{
	int i;
	u64 sso_reg;

	/*clear all ena */
	sso_reg = SSO_ERR0;
	sso_reg_write(sso, SSO_PF_ERR0_ENA_W1C, sso_reg);
	sso_reg = SSO_ERR1;
	sso_reg_write(sso, SSO_PF_ERR1_ENA_W1C, sso_reg);
	sso_reg = SSO_ERR2;
	sso_reg_write(sso, SSO_PF_ERR2_ENA_W1C, sso_reg);
	sso_reg = SSO_MBOX;
	sso_reg_write(sso, SSO_PF_MBOX_ENA_W1C, sso_reg);

	for (i = 0; i < SSO_PF_MSIX_COUNT; i++)
		free_irq(sso->msix_entries[i].vector, sso);

	pci_disable_msix(sso->pdev);
}

static inline void sso_configure_on_chip_res(struct ssopf *sso, u16 ssogrps)
{
	u64 tmp, add, grp_thr, grp_rsvd;
	u64 iaq_free_cnt, iaq_max;
	u32 iaq_rsvd, iaq_rsvd_cnt = 0;
	u64 taq_free_cnt, taq_max;
	u32 taq_rsvd, taq_rsvd_cnt = 0;
	u32 counter = 0;
	u16 grp;

	/* Reset SSO */
	sso_reg_write(sso, SSO_PF_RESET, 0x1);
	/* After initiating reset, the SSO must not be sent any other
	 * operations for 2500 coprocessor (SCLK) cycles. Assuming SCLK running
	 * at >1MHz, A delay of 2500us would be enough for the worst case.
	 */
	mdelay(3);
	while (sso_reg_read(sso, SSO_PF_RESET)) {
		usleep_range(1, 100);
		if (++counter > MAX_SSO_RST_TIMEOUT_US) {
			dev_warn(&sso->pdev->dev, "failed to reset sso\n");
			break;
		}
	}

	/* Configure IAQ entries */
	iaq_free_cnt = sso_reg_read(sso, SSO_PF_AW_WE) &
				SSO_AW_WE_FREE_CNT_MASK;

	/* Give out half of buffers fairly, rest left floating */
	iaq_rsvd = iaq_free_cnt / ssogrps / 2;
	/* Enforce minimum per HRM */
	if (iaq_rsvd < 2)
		iaq_rsvd = 2;
	iaq_max = iaq_rsvd << 7;
	if (iaq_max >= (1 << 13))
		iaq_max = (1 << 13) - 1;
	dev_dbg(&sso->pdev->dev, "iaq: free_cnt:0x%llx rsvd:0x%x max:0x%llx\n",
		iaq_free_cnt, iaq_rsvd, iaq_max);

	/* Configure TAQ entries */
	taq_free_cnt = sso_reg_read(sso, SSO_PF_TAQ_CNT) &
					SSO_TAQ_CNT_FREE_CNT_MASK;
	/* Give out half of all buffers fairly, other half floats */
	taq_rsvd = taq_free_cnt / ssogrps / 2;
	/* Enforce minimum per HRM */
	if (taq_rsvd < 3)
		taq_rsvd = 3;

	taq_max = taq_rsvd << 3;
	if (taq_max >= (1 << 11))
		taq_max = (1 << 11) - 1;
	dev_dbg(&sso->pdev->dev, "taq: free_cnt:0x%llx rsvd:0x%x max:0x%llx\n",
		taq_free_cnt, taq_rsvd, taq_max);

	for (grp = 0; grp < ssogrps; grp++) {
		tmp = sso_reg_read(sso, SSO_PF_GRPX_IAQ_THR(grp));
		grp_rsvd = tmp & SSO_GRP_IAQ_THR_RSVD_MASK;
		add = iaq_rsvd - grp_rsvd;

		grp_thr = iaq_rsvd & SSO_GRP_IAQ_THR_RSVD_MASK;
		grp_thr |= ((iaq_max & SSO_GRP_IAQ_THR_MAX_MASK) <<
				 SSO_GRP_IAQ_THR_MAX_SHIFT);

		sso_reg_write(sso, SSO_PF_GRPX_IAQ_THR(grp), grp_thr);
		/* Add the delta of added rsvd iaq entries */
		if (add)
			sso_reg_write(sso, SSO_PF_AW_ADD,
				      ((add & SSO_AW_ADD_RSVD_MASK) <<
					 SSO_AW_ADD_RSVD_SHIFT));
		iaq_rsvd_cnt += iaq_rsvd;

		tmp = sso_reg_read(sso, SSO_PF_GRPX_TAQ_THR(grp));
		grp_rsvd = tmp & SSO_GRP_TAQ_THR_RSVD_MASK;
		add = taq_rsvd - grp_rsvd;

		grp_thr = taq_rsvd & SSO_GRP_TAQ_THR_RSVD_MASK;
		grp_thr |= ((taq_max & SSO_GRP_TAQ_THR_MAX_MASK) <<
				SSO_GRP_TAQ_THR_MAX_SHIFT);
		sso_reg_write(sso, SSO_PF_GRPX_TAQ_THR(grp), grp_thr);
		/* Add the delta of added rsvd taq entries */
		if (add)
			sso_reg_write(sso, SSO_PF_TAQ_ADD,
				      ((add & SSO_TAQ_ADD_RSVD_MASK) <<
					SSO_TAQ_ADD_RSVD_SHIFT));
		taq_rsvd_cnt += taq_rsvd;
	}

	dev_dbg(&sso->pdev->dev, "iaq-rsvd=0x%x/0x%llx taq-rsvd=0x%x/0x%llx\n",
		iaq_rsvd_cnt, iaq_free_cnt, taq_rsvd_cnt, taq_free_cnt);
	/* Verify SSO_AW_WE[RSVD_FREE], TAQ_CNT[RSVD_FREE] are greater than
	 * or equal to sum of IAQ[RSVD_THR], TAQ[RSRVD_THR] fields
	 */
	tmp = sso_reg_read(sso, SSO_PF_AW_WE) >> SSO_AW_WE_RSVD_CNT_SHIFT;
	tmp &= SSO_AW_WE_RSVD_CNT_MASK;
	if (tmp < iaq_rsvd_cnt) {
		dev_warn(&sso->pdev->dev, "wrong iaq res alloc math %llx:%x\n",
			 tmp, iaq_rsvd_cnt);
		sso_reg_write(sso, SSO_PF_AW_WE,
			      (iaq_rsvd_cnt & SSO_AW_WE_RSVD_CNT_MASK) <<
				SSO_AW_WE_RSVD_CNT_SHIFT);
	}
	tmp = sso_reg_read(sso, SSO_PF_TAQ_CNT) >> SSO_TAQ_CNT_RSVD_CNT_SHIFT;
	tmp &= SSO_TAQ_CNT_FREE_CNT_MASK;
	if (tmp < taq_rsvd_cnt) {
		dev_warn(&sso->pdev->dev, "wrong taq res alloc math %llx:%x\n",
			 tmp, taq_rsvd_cnt);
		sso_reg_write(sso, SSO_PF_TAQ_CNT,
			      (taq_rsvd_cnt & SSO_TAQ_CNT_RSVD_CNT_MASK) <<
				SSO_TAQ_CNT_RSVD_CNT_SHIFT);
	}
	/* Turn off SSO conditional clocking (Errata SSO-29000) */
	tmp = sso_reg_read(sso, SSO_PF_WS_CFG);
	tmp |= 0x1; /*SSO_CCLK_DIS*/
	sso_reg_write(sso, SSO_PF_WS_CFG, tmp);
}

static inline void sso_max_grps_update(struct ssopf *sso)
{
	u64 sso_reg;
	u16 nr_grps;

	sso_reg = sso_reg_read(sso, SSO_PF_CONST);
	nr_grps = (sso_reg >> SSO_CONST_GRP_SHIFT) &
		SSO_CONST_GRP_MASK;

	if (!max_grps || max_grps > nr_grps)
		max_grps = nr_grps;
}

static int sso_init(struct ssopf *sso)
{
	u64 sso_reg;
	u32 max_maps;
	u32 xaq_buf_size;
	u32 xae_waes;
	u16 nr_grps;
	int i;
	int err;
	u32 xaq_buffers;
	u64 xaq_buf;

	sso_configure_on_chip_res(sso, max_grps);

	/* init sso.domain.master/master_data/mbox_addr to null */
	for (i = 0; i < SSO_MAX_VF; i++) {
		sso->vf[i].domain.in_use = 0;
		sso->vf[i].domain.master = NULL;
		sso->vf[i].domain.master_data = NULL;
	}

	sso_reg = sso_reg_read(sso, SSO_PF_CONST1);
	xaq_buf_size = (sso_reg >> SSO_CONST1_XAQ_BUF_SIZE_SHIFT) &
		SSO_CONST1_XAQ_BUF_SIZE_MASK;
	max_maps = (sso_reg >> SSO_CONST1_MAPS_SHIFT) &
		SSO_CONST1_MAPS_MASK;
	xae_waes = (sso_reg >> SSO_CONST1_XAE_WAES_SHIFT) &
		SSO_CONST1_XAE_WAES_MASK;
	sso_reg = sso_reg_read(sso, SSO_PF_CONST);
	nr_grps = (sso_reg >> SSO_CONST_GRP_SHIFT) &
		SSO_CONST_GRP_MASK;

	sso_reg_write(sso, SSO_PF_NW_TIM, 0x4);

	sso->xaq_buf_size = xaq_buf_size;

	err = fpapf->create_domain(sso->id, FPA_SSO_XAQ_GMID, 1, NULL);
	if (!err) {
		dev_err(&sso->pdev->dev, "failed to create SSO_XAQ_DOMAIN\n");
		symbol_put(fpapf_com);
		return -ENODEV;
	}

	fpa = fpavf->get(FPA_SSO_XAQ_GMID, 0, &sso_master_com, sso);
	if (!fpa) {
		dev_err(&sso->pdev->dev, "failed to get fpavf\n");
		symbol_put(fpapf_com);
		symbol_put(fpavf_com);
		return -ENODEV;
	}

	xaq_buffers = (max_events + xae_waes - 1) / xae_waes;
	xaq_buffers = (nr_grps * 2) + 48 + xaq_buffers;

	dev_notice(&sso->pdev->dev, "Setup SSO_XAQ_DOMAIN: xaq_buffers %d, xaq_buf_size %d\n",
		   xaq_buffers, xaq_buf_size);
	err = fpavf->setup(fpa, xaq_buffers, xaq_buf_size,
			   &sso->pdev->dev);
	if (err) {
		dev_err(&sso->pdev->dev, "failed to setup fpavf\n");
		symbol_put(fpapf_com);
		symbol_put(fpavf_com);
		return -ENODEV;
	}

	/* Make sure the SSO is disabled */
	sso_reg = sso_reg_read(sso, SSO_PF_AW_CFG);
	sso_reg &= (~1ULL);
	sso_reg_write(sso, SSO_PF_AW_CFG, sso_reg);

	/* Init XAQ ring*/
	for (i = 0; i < nr_grps; i++) {
		xaq_buf = fpavf->alloc(fpa, FPA_SSO_XAQ_AURA);
		if (!xaq_buf) {
			dev_err(&sso->pdev->dev, "failed to setup XAQ:%d\n", i);
			goto err;
		}
		sso_reg_write(sso, SSO_PF_XAQX_HEAD_PTR(i), xaq_buf);
		sso_reg_write(sso, SSO_PF_XAQX_HEAD_NEXT(i), xaq_buf);
		sso_reg_write(sso, SSO_PF_XAQX_TAIL_PTR(i), xaq_buf);
		sso_reg_write(sso, SSO_PF_XAQX_TAIL_NEXT(i), xaq_buf);
	}

	sso_reg_write(sso, SSO_PF_XAQ_AURA, FPA_SSO_XAQ_AURA);
	sso_reg_write(sso, SSO_PF_XAQ_GMCTL, FPA_SSO_XAQ_GMID);

	dev_dbg(&sso->pdev->dev, "aura=%d gmid=%d xaq_buffers=%d\n",
		FPA_SSO_XAQ_AURA, FPA_SSO_XAQ_GMID, xaq_buffers);

	/* Enable XAQ*/
	sso_reg = sso_reg_read(sso, SSO_PF_AW_CFG);
	sso_reg |= 0xf;
	sso_reg_write(sso, SSO_PF_AW_CFG, sso_reg);
	return 0;
err:
	symbol_put(fpapf_com);
	symbol_put(fpavf_com);
	return -ENODEV;
}

static int sso_sriov_configure(struct pci_dev *pdev, int numvfs)
{
	struct ssopf *sso = pci_get_drvdata(pdev);
	int ret = -EBUSY;
	int disable = 0;

	if (sso->vfs_in_use != 0)
		return ret;

	ret = 0;
	if (sso->flags & SSO_SRIOV_ENABLED)
		disable = 1;

	if (disable) {
		pci_disable_sriov(pdev);
		sso->flags &= ~SSO_SRIOV_ENABLED;
		sso->total_vfs = 0;
	}

	if (numvfs > 0) {
		ret = pci_enable_sriov(pdev, numvfs);
		if (ret == 0) {
			sso->flags |= SSO_SRIOV_ENABLED;
			sso->total_vfs = numvfs;

			ret = numvfs;
		}
	}

	dev_notice(&sso->pdev->dev, "VFs enabled: %d\n", ret);
	return ret;
}

static int sso_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	struct device *dev = &pdev->dev;
	struct ssopf *sso;
	int err = -ENOMEM;

	sso = devm_kzalloc(dev, sizeof(*sso), GFP_KERNEL);
	if (!sso)
		return err;

	pci_set_drvdata(pdev, sso);
	sso->pdev = pdev;

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
	sso->reg_base = pcim_iomap(pdev, PCI_SSO_PF_CFG_BAR, 0);
	if (!sso->reg_base) {
		dev_err(dev, "Can't map CFG space\n");
		err = -ENOMEM;
		return err;
	}

	/* set SSO ID */
	sso->id = atomic_add_return(1, &sso_count);
	sso->id -= 1;

	sso_max_grps_update(sso);

	err = sso_init(sso);
	if (err) {
		atomic_sub_return(1, &sso_count);
		return err;
	}

	err = sso_irq_init(sso);
	if (err) {
		dev_err(dev, "failed init irqs\n");
		err = -EINVAL;
		return err;
	}

	/* Alloc local memory to copy message to/from rambox.
	 * This local memory in next revision will be removed.
	 * such that kernel/user does message rd/wr on single
	 * buffer called rambox, Implementing that design right
	 * demands gaurantee for true mutual execusion for message
	 * written accessed by one party at a time.
	 * Current rambox design not truely accommodate that;
	 * CAS implementation in future will gaurantee locking
	 * parity between user/kernel space, then will get rid
	 * of local buffer data copy approach.
	 * - Assuiming max message buffer mey not exceed 1024.
	 */
	ram_mbox_buf = kzalloc(MBOX_MAX_MSG_SIZE, GFP_KERNEL);
	if (!ram_mbox_buf) {
		err = -ENOMEM;
		return err;
	}

	INIT_WORK(&sso->mbox_work, handle_mbox);

	INIT_LIST_HEAD(&sso->list);
	mutex_lock(&octeontx_sso_devices_lock);
	list_add(&sso->list, &octeontx_sso_devices);
	mutex_unlock(&octeontx_sso_devices_lock);
	return 0;
}

static void sso_fini(struct ssopf *sso)
{
	// TODO: add the meat
	// look 11.13.2 Recovering Pointers
}

static void sso_remove(struct pci_dev *pdev)
{
	struct device *dev = &pdev->dev;
	struct ssopf *sso = pci_get_drvdata(pdev);
	u64 sso_reg, addr;
	u16 nr_grps;
	int i;

	if (!sso)
		return;

	flush_scheduled_work();
	/* Make sure the SSO is disabled */
	sso_reg = sso_reg_read(sso, SSO_PF_AW_CFG);
	sso_reg &= (~1ULL);
	sso_reg_write(sso, SSO_PF_AW_CFG, sso_reg);
	kfree(ram_mbox_buf);
	sso_reg = sso_reg_read(sso, SSO_PF_CONST);
	nr_grps = (sso_reg >> SSO_CONST_GRP_SHIFT) & SSO_CONST_GRP_MASK;
	for (i = 0; i < nr_grps; i++) {
		addr = sso_reg_read(sso, SSO_PF_XAQX_HEAD_PTR(i));
		if (addr)
			fpavf->free(fpa, FPA_SSO_XAQ_AURA, addr, 0);
	}

	dev_notice(&sso->pdev->dev, "Destroy SSO_XAQ_DOMAIN\n");
	fpavf->teardown(fpa);
	fpavf->put(fpa);
	fpapf->destroy_domain(sso->id, FPA_SSO_XAQ_GMID, NULL);
	sso_irq_free(sso);
	sso_sriov_configure(pdev, 0);
	sso_fini(sso);

	/* release probed resources */
	mutex_lock(&octeontx_sso_devices_lock);
	list_del(&sso->list);
	mutex_unlock(&octeontx_sso_devices_lock);
	pci_release_regions(pdev);
	pci_disable_device(pdev);
	pci_set_drvdata(pdev, NULL);
	devm_kfree(dev, sso);
}

/* devices supported */
static const struct pci_device_id sso_id_table[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_CAVIUM, PCI_DEVICE_ID_OCTEONTX_SSO_PF) },
	{ 0, }  /* end of table */
};

static struct pci_driver sso_driver = {
	.name = DRV_NAME,
	.id_table = sso_id_table,
	.probe = sso_probe,
	.remove = sso_remove,
	.sriov_configure = sso_sriov_configure,
};

MODULE_AUTHOR("Tirumalesh Chalamarla");
MODULE_DESCRIPTION("Cavium OCTEONTX SSO Physical Function Driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRV_VERSION);
MODULE_DEVICE_TABLE(pci, sso_id_table);

static int __init sso_init_module(void)
{
	pr_info("%s, ver %s\n", DRV_NAME, DRV_VERSION);
	rst = try_then_request_module(symbol_get(rst_com), "rst");
	if (!rst)
		return -ENODEV;

	fpapf = try_then_request_module(symbol_get(fpapf_com), "fpapf");
	if (!fpapf) {
		symbol_put(rst);
		return -ENODEV;
	}

	fpavf = try_then_request_module(symbol_get(fpavf_com), "fpavf");
	if (!fpavf) {
		symbol_put(rst);
		symbol_put(fpapf_com);
		return -ENODEV;
	}

	return pci_register_driver(&sso_driver);
}

static void __exit sso_cleanup_module(void)
{
	pci_unregister_driver(&sso_driver);
	symbol_put(rst_com);
	symbol_put(fpapf_com);
	symbol_put(fpavf_com);
}

module_init(sso_init_module);
module_exit(sso_cleanup_module);
