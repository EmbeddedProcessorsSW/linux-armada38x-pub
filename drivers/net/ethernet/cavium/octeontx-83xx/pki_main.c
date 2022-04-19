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
#include <linux/random.h>

#include "pki.h"

#define DRV_NAME "octeontx-pki"
#define DRV_VERSION "1.0"

static atomic_t pki_count = ATOMIC_INIT(0);

static DEFINE_MUTEX(octeontx_pki_devices_lock);
static LIST_HEAD(octeontx_pki_devices);

static irqreturn_t pki_gen_intr_handler(int irq, void *pki_irq)
{
	struct pki_t *pki = (struct pki_t *)pki_irq;
	u64 gen_int = pki_reg_read(pki, PKI_GEN_INT);

	printk_once("Received GEN INT(%llx)", gen_int);

	pki_reg_write(pki, PKI_GEN_INT, gen_int);
	return IRQ_HANDLED;
}

static irqreturn_t pki_ecc_intr_handler(int irq, void *pki_irq)
{
	struct pki_t *pki = (struct pki_t *)pki;
	u64 reg;

	dev_err(&pki->pdev->dev, "Received ECC INT\n");
	reg = pki_reg_read(pki, PKI_ECC0_INT);
	dev_err(&pki->pdev->dev, "ecc0:%llx \t", reg);
	pki_reg_write(pki, PKI_ECC0_INT, reg);
	reg = pki_reg_read(pki, PKI_ECC1_INT);
	dev_err(&pki->pdev->dev, "ecc1:%llx \t", reg);
	pki_reg_write(pki, PKI_ECC1_INT, reg);
	reg = pki_reg_read(pki, PKI_ECC2_INT);
	dev_err(&pki->pdev->dev, "ecc2:%llx\n", reg);
	pki_reg_write(pki, PKI_ECC2_INT, reg);

	return IRQ_HANDLED;
}

static irqreturn_t pki_cl_intr_handler(int irq, void *pki_irq)
{
	struct pki_t *pki = (struct pki_t *)pki_irq;
	u64 reg;

	dev_err(&pki->pdev->dev, "Cluster INT received\n");
	reg = pki_reg_read(pki, PKI_CLX_INT(0));
	dev_err(&pki->pdev->dev, "cl0_int: %llx \t", reg);
	pki_reg_write(pki, PKI_CLX_INT(0), reg);
	reg = pki_reg_read(pki, PKI_CLX_INT(1));
	dev_err(&pki->pdev->dev, "cl1_int: %llx\n", reg);
	pki_reg_write(pki, PKI_CLX_INT(1), reg);

	return IRQ_HANDLED;
}

static irqreturn_t pki_cl_ecc_intr_handler(int irq, void *pki_irq)
{
	struct pki_t *pki = (struct pki_t *)pki_irq;
	u64 reg;

	dev_err(&pki->pdev->dev, "Cluster ECC INT received\n");
	reg = pki_reg_read(pki, PKI_CLX_ECC_INT(0));
	dev_err(&pki->pdev->dev, "cl0_ecc0_int: %llx \t", reg);
	pki_reg_write(pki, PKI_CLX_ECC_INT(0), reg);
	reg = pki_reg_read(pki, PKI_CLX_ECC_INT(1));
	dev_err(&pki->pdev->dev, "cl1_ecc1_int: %llx\n", reg);
	pki_reg_write(pki, PKI_CLX_ECC_INT(1), reg);

	return IRQ_HANDLED;
}

static irqreturn_t pki_alloc_flt_intr_handler(int irq, void *pki_irq)
{
	struct pki_t *pki = (struct pki_t *)pki_irq;
	u64 reg;

	dev_err(&pki->pdev->dev, "FPA alloc failed\n");
	reg = pki_reg_read(pki, PKI_ALLOC_FLTX_INT(0));
	dev_err(&pki->pdev->dev, "flt0: %llx \t", reg);
	pki_reg_write(pki, PKI_ALLOC_FLTX_INT(0), reg);
	reg = pki_reg_read(pki, PKI_ALLOC_FLTX_INT(1));
	dev_err(&pki->pdev->dev, "flt1: %llx \t", reg);
	pki_reg_write(pki, PKI_ALLOC_FLTX_INT(1), reg);
	reg = pki_reg_read(pki, PKI_ALLOC_FLTX_INT(2));
	dev_err(&pki->pdev->dev, "flt2: %llx \t", reg);
	pki_reg_write(pki, PKI_ALLOC_FLTX_INT(2), reg);
	reg = pki_reg_read(pki, PKI_ALLOC_FLTX_INT(3));
	dev_err(&pki->pdev->dev, "flt3: %llx\n", reg);
	pki_reg_write(pki, PKI_ALLOC_FLTX_INT(3), reg);

	return IRQ_HANDLED;
}

static irqreturn_t pki_store_flt_intr_handler(int irq, void *pki_irq)
{
	struct pki_t *pki = (struct pki_t *)pki_irq;
	u64 reg;

	dev_err(&pki->pdev->dev, "NCB store fualt\n");
	reg = pki_reg_read(pki, PKI_STRM_FLTX_INT(0));
	dev_err(&pki->pdev->dev, "strm0: %llx \t", reg);
	pki_reg_write(pki, PKI_STRM_FLTX_INT(0), reg);
	reg = pki_reg_read(pki, PKI_STRM_FLTX_INT(1));
	dev_err(&pki->pdev->dev, "strm1: %llx \t", reg);
	pki_reg_write(pki, PKI_STRM_FLTX_INT(1), reg);
	reg = pki_reg_read(pki, PKI_STRM_FLTX_INT(2));
	dev_err(&pki->pdev->dev, "strm2: %llx \t", reg);
	pki_reg_write(pki, PKI_STRM_FLTX_INT(2), reg);
	reg = pki_reg_read(pki, PKI_STRM_FLTX_INT(3));
	dev_err(&pki->pdev->dev, "strm3: %llx \t", reg);
	pki_reg_write(pki, PKI_STRM_FLTX_INT(3), reg);

	return IRQ_HANDLED;
}

static struct intr_hand intr[] = {
	{0x2, "pki gen intr", PKI_GEN_INT_ENA_W1C,
		PKI_GEN_INT_ENA_W1S, pki_gen_intr_handler},
	{0xffff, "pki ecc0 intr", PKI_ECC0_INT_ENA_W1C,
		PKI_ECC0_INT_ENA_W1S, pki_ecc_intr_handler},
	{0x3ffc0f3cff, "pki ecc1 intr", PKI_ECC1_INT_ENA_W1C,
		PKI_ECC1_INT_ENA_W1S, pki_ecc_intr_handler},
	{0x3, "pki ecc2 intr", PKI_ECC2_INT_ENA_W1C,
		PKI_ECC2_INT_ENA_W1S, pki_ecc_intr_handler},
	{0xf, "pki cluster0 intr", PKI_CLX_INT_ENA_W1C(0),
		PKI_CLX_INT_ENA_W1S(0), pki_cl_intr_handler},
	{0xf, "pki cluster1 intr", PKI_CLX_INT_ENA_W1C(1),
		PKI_CLX_INT_ENA_W1S(1), pki_cl_intr_handler},
	{0xff, "pki cluster0 ecc intr", PKI_CLX_ECC_INT_ENA_W1C(0),
		PKI_CLX_ECC_INT_ENA_W1S(0), pki_cl_ecc_intr_handler},
	{0xff, "pki cluster1 ecc intr", PKI_CLX_ECC_INT_ENA_W1C(1),
		PKI_CLX_ECC_INT_ENA_W1S(1), pki_cl_ecc_intr_handler},
	{0xffffffffffffffff, "pki NCB store intr(0)",
		PKI_STRM_FLTX_INT_ENA_W1C(0),
		PKI_STRM_FLTX_INT_ENA_W1S(0), pki_store_flt_intr_handler},
	{0xffffffffffffffff, "pki NCB store intr(1)",
		PKI_STRM_FLTX_INT_ENA_W1C(1),
		PKI_STRM_FLTX_INT_ENA_W1S(1), pki_store_flt_intr_handler},
	{0xffffffffffffffff, "pki NCB store intr(2)",
		PKI_STRM_FLTX_INT_ENA_W1C(2),
		PKI_STRM_FLTX_INT_ENA_W1S(2), pki_store_flt_intr_handler},
	{0xffffffffffffffff, "pki NCB store intr(3)",
		PKI_STRM_FLTX_INT_ENA_W1C(3),
		PKI_STRM_FLTX_INT_ENA_W1S(3), pki_store_flt_intr_handler},
	{0xffffffffffffffff, "pki Alloc fualt intr(0)",
		PKI_ALLOC_FLTX_INT_ENA_W1C(0),
		PKI_ALLOC_FLTX_INT_ENA_W1S(0), pki_alloc_flt_intr_handler},
	{0xffffffffffffffff, "pki Alloc fualt intr(1)",
		PKI_ALLOC_FLTX_INT_ENA_W1C(1),
		PKI_ALLOC_FLTX_INT_ENA_W1S(1), pki_alloc_flt_intr_handler},
	{0xffffffffffffffff, "pki Alloc fualt intr(2)",
		PKI_ALLOC_FLTX_INT_ENA_W1C(2),
		PKI_ALLOC_FLTX_INT_ENA_W1S(2), pki_alloc_flt_intr_handler},
	{0xffffffffffffffff, "pki Alloc fualt intr(3)",
		PKI_ALLOC_FLTX_INT_ENA_W1C(3),
		PKI_ALLOC_FLTX_INT_ENA_W1S(3), pki_alloc_flt_intr_handler}
};

static inline void write_ltype(struct pki_t *pki, u64 ltype, u64 beltype)
{
	u64 reg = PKI_BELTYPE(beltype);

	pki_reg_write(pki, PKI_LTYPEX_MAP(PKI_LTYPE(ltype)), reg);
}

static inline void setup_ltype_map(struct pki_t *pki)
{
	write_ltype(pki, PKI_LTYPE_E_NONE, PKI_BLTYPE_E_NONE);
	write_ltype(pki, PKI_LTYPE_E_ENET, PKI_BLTYPE_E_MISC);
	write_ltype(pki, PKI_LTYPE_E_VLAN, PKI_BLTYPE_E_MISC);
	write_ltype(pki, PKI_LTYPE_E_SNAP_PAYLD, PKI_BLTYPE_E_MISC);
	write_ltype(pki, PKI_LTYPE_E_ARP, PKI_BLTYPE_E_MISC);
	write_ltype(pki, PKI_LTYPE_E_RARP, PKI_BLTYPE_E_MISC);
	write_ltype(pki, PKI_LTYPE_E_IP4, PKI_BLTYPE_E_IP4);
	write_ltype(pki, PKI_LTYPE_E_IP4_OPT, PKI_BLTYPE_E_IP4);
	write_ltype(pki, PKI_LTYPE_E_IP6, PKI_BLTYPE_E_IP6);
	write_ltype(pki, PKI_LTYPE_E_IP6_OPT, PKI_BLTYPE_E_IP6);
	write_ltype(pki, PKI_LTYPE_E_IPSEC_ESP, PKI_BLTYPE_E_MISC);
	write_ltype(pki, PKI_LTYPE_E_IPFRAG, PKI_BLTYPE_E_MISC);
	write_ltype(pki, PKI_LTYPE_E_IPCOMP, PKI_BLTYPE_E_MISC);
	write_ltype(pki, PKI_LTYPE_E_TCP, PKI_BLTYPE_E_TCP);
	write_ltype(pki, PKI_LTYPE_E_UDP, PKI_BLTYPE_E_UDP);
	write_ltype(pki, PKI_LTYPE_E_SCTP, PKI_BLTYPE_E_SCTP);
	write_ltype(pki, PKI_LTYPE_E_UDP_VXLAN, PKI_BLTYPE_E_UDP);
	write_ltype(pki, PKI_LTYPE_E_GRE, PKI_BLTYPE_E_MISC);
	write_ltype(pki, PKI_LTYPE_E_NVGRE, PKI_BLTYPE_E_MISC);
	write_ltype(pki, PKI_LTYPE_E_GTP, PKI_BLTYPE_E_MISC);
	write_ltype(pki, PKI_LTYPE_E_UDP_GENEVE, PKI_BLTYPE_E_UDP);
	write_ltype(pki, PKI_LTYPE_E_SW28, PKI_BLTYPE_E_MISC);
	write_ltype(pki, PKI_LTYPE_E_SW29, PKI_BLTYPE_E_MISC);
	write_ltype(pki, PKI_LTYPE_E_SW30, PKI_BLTYPE_E_MISC);
	write_ltype(pki, PKI_LTYPE_E_SW31, PKI_BLTYPE_E_MISC);
}

static int load_ucode(struct pki_t *pki)
{
	extern const u64 PKI_MICROCODE_CN83XX_LENGTH;
	extern const u64 PKI_MICROCODE_CN83XX[];
	unsigned int i;

	if (PKI_MICROCODE_CN83XX_LENGTH > PKI_SRAM_SZIE)
		return -1;

	for (i = 0; i < PKI_MICROCODE_CN83XX_LENGTH; i++)
		pki_reg_write(pki, PKI_IMEM(i), PKI_MICROCODE_CN83XX[i]);

	return 0;
}

static void init_pcam_entries(struct pki_t *pki)
{
	int i, index, bank;
	u64 term_reg = 0;
	u64 offset;

	/* Disable all pcam entries */
	set_field(&term_reg, PKI_PCAM_TERM_VALID_MASK,
		  PKI_PCAM_TERM_VALID_SHIFT, 0);

	for (i = 0; i < pki->max_cls; i++)
		for (bank = 0; bank < pki->max_pcams; bank++)
			for (index = 0; index < pki->max_pcam_ents; index++) {
				offset = PKI_CLX_PCAMX_TERMX(i, bank, index);
				pki_reg_write(pki, offset, term_reg);
			}
}

/*locks should be used by caller
 */
static struct pki_t *pki_get_pf(u32 id)
{
	struct pki_t *pki = NULL;
	struct pki_t *curr;

	list_for_each_entry(curr, &octeontx_pki_devices, list) {
		if (curr->id == id) {
			pki = curr;
			break;
		}
	}
	return pki;
}

static struct pkipf_vf *pki_get_vf(u32 id, u16 domain_id)
{
	struct pki_t *pki = NULL;
	struct pki_t *curr;
	int i;
	int vf_idx = -1;

	list_for_each_entry(curr, &octeontx_pki_devices, list) {
		if (curr->id == id) {
			pki = curr;
			break;
		}
	}

	if (!pki)
		return NULL;

	for (i = 0; i < PKI_MAX_VF; i++) {
		if (pki->vf[i].domain.domain_id == domain_id) {
			vf_idx = i;
			break;
		}
	}
	if (vf_idx >= 0)
		return &pki->vf[vf_idx];
	else
		return NULL;
}

static void identify(struct pkipf_vf *vf, u16 domain_id, u16 subdomain_id)
{
	u64 reg = (((u64)subdomain_id << 16) | domain_id);

	writeq_relaxed(reg, vf->domain.reg_base);
}

static int pki_destroy_domain(u32 id, u16 domain_id, struct kobject *kobj)
{
	struct pki_t *pki = NULL;
	struct pci_dev *virtfn;
	struct pki_t *curr;
	int i, port, vf_idx = 0;

	mutex_lock(&octeontx_pki_devices_lock);
	list_for_each_entry(curr, &octeontx_pki_devices, list) {
		if (curr->id == id) {
			pki = curr;
			break;
		}
	}
	if (!pki) {
		mutex_unlock(&octeontx_pki_devices_lock);
		return -ENODEV;
	}

	for (i = 0; i < PKI_MAX_VF; i++) {
		if (pki->vf[i].domain.in_use &&
		    pki->vf[i].domain.domain_id == domain_id) {
			free_loop_pkind_lbk(&pki->vf[i], domain_id);

			pki->vf[i].domain.in_use = false;

			virtfn = pci_get_domain_bus_and_slot(pci_domain_nr
					(pki->pdev->bus),
					pci_iov_virtfn_bus(pki->pdev, i),
					pci_iov_virtfn_devfn(pki->pdev, i));
			if (virtfn && kobj)
				sysfs_remove_link(kobj, virtfn->dev.kobj.name);

			dev_info(&pki->pdev->dev,
				 "Free vf[%d] from domain:%d subdomain_id:%d\n",
				 i, pki->vf[i].domain.domain_id, vf_idx);

			for (port = 0; port < MAX_PKI_PORTS; port++) {
				pki->vf[i].bgx_port[port].valid = false;
				pki->vf[i].lbk_port[port].valid = false;
			}
			identify(&pki->vf[i], 0x0, 0x0);
			vf_idx++;
		}
	}

	pki->vfs_in_use -= vf_idx;
	mutex_unlock(&octeontx_pki_devices_lock);
	return 0;
}

static u64 pki_create_domain(u32 id, u16 domain_id,
			     struct octeontx_master_com_t *master_com,
			     void *data, struct kobject *kobj)
{
	struct pki_t *pki = NULL;
	resource_size_t vf_start;
	struct pci_dev *virtfn;
	struct pki_t *curr;
	int i, ret = 0, vf_idx = 0;
	unsigned long pki_mask = 0;
	u8 stream;
	u64 cfg;

	if (!kobj)
		return 0;

	mutex_lock(&octeontx_pki_devices_lock);
	list_for_each_entry(curr, &octeontx_pki_devices, list) {
		if (curr->id == id) {
			pki = curr;
			break;
		}
	}

	if (!pki)
		goto err_unlock;

	for (i = 0; i < PKI_MAX_VF; i++) {
		if (pki->vf[i].domain.in_use) {/* pki port config */

			continue;
		} else {
			virtfn = pci_get_domain_bus_and_slot(pci_domain_nr
					(pki->pdev->bus),
					pci_iov_virtfn_bus(pki->pdev, i),
					pci_iov_virtfn_devfn(pki->pdev, i));
			if (!virtfn)
				break;
			ret = sysfs_create_link(kobj, &virtfn->dev.kobj,
						virtfn->dev.kobj.name);
			if (ret < 0)
				goto err_unlock;

			pki->vf[i].domain.domain_id = domain_id;
			pki->vf[i].domain.subdomain_id = 0;
			pki->vf[i].domain.gmid = get_gmid(domain_id);
			pki->vf[i].domain.in_use = true;
			stream = i + 1;
			pki->vf[i].stream_id = stream;
			pki->vf[i].pki = pki;
			/* TO_DO if pki resource virtualization implemented*/
			pki->vf[i].max_fstyles = pki->max_fstyles;
			pki->vf[i].max_auras = pki->max_auras;
			pki->vf[i].max_qpgs = pki->max_qpgs;
			pki->vf[i].max_pcam_ents = pki->max_pcam_ents;
			cfg = (pki->vf[i].domain.gmid) &
			       PKI_STRM_CFG_GMID_MASK;
			pki_reg_write(pki, PKI_STRMX_CFG(stream), cfg);
			vf_start = PKI_VF_BASE(i);
			pki->vf[i].domain.reg_base = ioremap_wc(vf_start,
								PKI_VF_SIZE);
			if (!pki->vf[i].domain.reg_base)
				goto err_unlock;

			identify(&pki->vf[i], pki->vf[i].domain.domain_id,
				 pki->vf[i].domain.subdomain_id);
			vf_idx++;
			set_bit(i, &pki_mask);
			break;
		}
	}

	if (!vf_idx)
		goto err_unlock;

	pki->vfs_in_use += vf_idx;
	mutex_unlock(&octeontx_pki_devices_lock);
	return pki_mask;

err_unlock:
	mutex_unlock(&octeontx_pki_devices_lock);
	pki_destroy_domain(id, domain_id, kobj);
	return 0;
}

static int pki_receive_message(u32 id, u16 domain_id,
			       struct mbox_hdr *hdr,
			       union mbox_data *req,
			       union mbox_data *resp, void *mdata)
{
	struct pkipf_vf *vf = NULL;

	if (!mdata)
		return -ENOMEM;

	hdr->res_code = MBOX_RET_SUCCESS;
	resp->data = 0;
	mutex_lock(&octeontx_pki_devices_lock);

	vf = pki_get_vf(id, domain_id);

	if (!vf) {
		hdr->res_code = MBOX_RET_INVALID;
		mutex_unlock(&octeontx_pki_devices_lock);
		return -ENODEV;
	}

	switch (hdr->msg) {
	case MBOX_PKI_PORT_OPEN:
		hdr->res_code = pki_port_open(vf, hdr->vfid, mdata);
		break;
	case MBOX_PKI_PORT_CREATE_QOS:
		hdr->res_code = pki_port_create_qos(vf, hdr->vfid,
						    mdata);
		break;
	case MBOX_PKI_PORT_MODIFY_QOS:
		hdr->res_code = pki_port_modify_qos(vf, hdr->vfid,
						    mdata);
		break;
	case MBOX_PKI_PORT_DELETE_QOS:
		hdr->res_code = pki_port_delete_qos(vf, hdr->vfid,
						    mdata);
		break;
	case MBOX_PKI_PORT_ALLOC_QPG:
		hdr->res_code = pki_port_alloc_qpg(vf, hdr->vfid,
						   mdata);
		break;
	case MBOX_PKI_PORT_FREE_QPG:
		hdr->res_code = pki_port_free_qpg(vf, hdr->vfid,
						  mdata);
		break;
	case MBOX_PKI_SET_PORT_CONFIG:
		hdr->res_code = pki_set_port_config(vf, hdr->vfid,
						    mdata);
		break;
	case MBOX_PKI_PORT_START:
		hdr->res_code = pki_port_start(vf, hdr->vfid, mdata);
		break;
	case MBOX_PKI_PORT_STOP:
		hdr->res_code = pki_port_stop(vf, hdr->vfid, mdata);
		break;
	case MBOX_PKI_PORT_CLOSE:
		hdr->res_code = pki_port_close(vf, hdr->vfid, mdata);
		break;
	case MBOX_PKI_PORT_PKTBUF_CONFIG:
		hdr->res_code = pki_port_pktbuf_cfg(vf, hdr->vfid,
						    mdata);
		break;
	case MBOX_PKI_PORT_ERRCHK_CONFIG:
		hdr->res_code = pki_port_errchk(vf, hdr->vfid,
						mdata);
		break;
	case MBOX_PKI_PORT_HASH_CONFIG:
		hdr->res_code = pki_port_hashcfg(vf, hdr->vfid, mdata);
		break;
	case MBOX_PKI_PORT_VLAN_FILTER_CONFIG:
		hdr->res_code = pki_port_vlan_fltr_cfg(vf, hdr->vfid, mdata);
		break;
	case MBOX_PKI_PORT_VLAN_FILTER_ENTRY_CONFIG:
		hdr->res_code = pki_port_vlan_fltr_entry_cfg(vf, hdr->vfid,
							     mdata);
		break;
	case MBOX_PKI_PORT_PCAM_GET:
		hdr->res_code = pki_port_pcam_get(vf, hdr->vfid, mdata,
						  &resp->data);
		break;
	case MBOX_PKI_PORT_PCAM_ALLOC:
		hdr->res_code = pki_port_pcam_alloc(vf, hdr->vfid, mdata,
						    &resp->data);
		break;
	case MBOX_PKI_PORT_PCAM_FREE:
		hdr->res_code = pki_port_pcam_free(vf, hdr->vfid, mdata,
						   &resp->data);
		break;
	}

	mutex_unlock(&octeontx_pki_devices_lock);
	return 0;
}

int pki_reset_domain(u32 id, u16 domain_id)
{
	int i;
	struct pkipf_vf *vf = NULL;
	struct pki_port *port;

	mutex_lock(&octeontx_pki_devices_lock);

	vf = pki_get_vf(id, domain_id);
	if (!vf) {
		mutex_unlock(&octeontx_pki_devices_lock);
		return -ENODEV;
	}

	for (i = 0; i < MAX_PKI_PORTS; i++) {
		if (vf->bgx_port[i].valid) {
			port = &vf->bgx_port[i];
			free_port_pcam_entries(vf->pki, port);
		}
		if (vf->lbk_port[i].valid)
			port = &vf->lbk_port[i];

		if (qpg_range_free(vf->pki, port->qpg_base, port->num_entry,
				   vf->domain.domain_id) < 0)
			return -EINVAL;

		/* Free up all the resources and reset regs */
		pki_port_reset_regs(vf->pki, port);
		port->init_style = PKI_DROP_STYLE;
		port->qpg_base = QPG_INVALID;
		port->num_entry = 0;
		port->shared_mask = 0;
		port->state = PKI_PORT_CLOSE;
	}

	identify(vf, vf->domain.domain_id, vf->domain.subdomain_id);

	mutex_unlock(&octeontx_pki_devices_lock);
	return 0;
}

/* Add a new port to PKI
 * return PKIND on success, -ERR on failure
 */
int pki_add_bgx_port(u32 id, u16 domain_id, struct octtx_bgx_port *port)
{
	struct pkipf_vf *vf = NULL;
	int pkind;

	mutex_lock(&octeontx_pki_devices_lock);

	vf = pki_get_vf(id, domain_id);
	if (!vf) {
		mutex_unlock(&octeontx_pki_devices_lock);
		return -ENODEV;
	}

	pkind = assign_pkind_bgx(vf, port);

	mutex_unlock(&octeontx_pki_devices_lock);
	return pkind;
}

int pki_add_lbk_port(u32 id, u16 domain_id, struct octtx_lbk_port *port)
{
	struct pkipf_vf *vf = NULL;
	int pkind;

	mutex_lock(&octeontx_pki_devices_lock);

	vf = pki_get_vf(id, domain_id);
	if (!vf) {
		mutex_unlock(&octeontx_pki_devices_lock);
		return -ENODEV;
	}

	/*TO_DO it needs channel number too*/
	pkind = assign_pkind_lbk(vf, port);

	mutex_unlock(&octeontx_pki_devices_lock);
	return pkind;
}

int pki_add_sdp_port(u32 id, u16 domain_id, struct octtx_sdp_port *port)
{
	struct pkipf_vf *vf = NULL;
	int pkind;

	mutex_lock(&octeontx_pki_devices_lock);

	vf = pki_get_vf(id, domain_id);
	if (!vf) {
		mutex_unlock(&octeontx_pki_devices_lock);
		return -ENODEV;
	}

	pkind = assign_pkind_sdp(vf, port);

	mutex_unlock(&octeontx_pki_devices_lock);
	return pkind;
}

int pki_get_bgx_port_stats(struct octtx_bgx_port *port)
{
	struct pki_t *pki;
	u64 reg;

	pki = pki_get_pf(port->node);
	if (!pki)
		return -EINVAL;
	reg = pki_reg_read(pki, PKI_STATX_STAT5(port->pkind));
	port->stats.rxbcast = reg & ((1ull << 47) - 1);
	reg = pki_reg_read(pki, PKI_STATX_STAT6(port->pkind));
	port->stats.rxmcast = reg & ((1ull << 47) - 1);
	reg = pki_reg_read(pki, PKI_STATX_STAT3(port->pkind));
	port->stats.rxdrop = reg & ((1ull << 47) - 1);
	return 0;
}

struct pki_com_s pki_com  = {
	.create_domain = pki_create_domain,
	.destroy_domain = pki_destroy_domain,
	.reset_domain = pki_reset_domain,
	.receive_message = pki_receive_message,
	.add_bgx_port = pki_add_bgx_port,
	.add_lbk_port = pki_add_lbk_port,
	.add_sdp_port = pki_add_sdp_port,
	.get_bgx_port_stats = pki_get_bgx_port_stats
};
EXPORT_SYMBOL(pki_com);

static void pki_irq_free(struct pki_t *pki)
{
	int i;

	/*clear intr */
	for (i = 0; i < PKI_MSIX_COUNT; i++) {
		pki_reg_write(pki, intr[i].coffset, intr[i].mask);
		if (pki->msix_entries[i].vector)
			free_irq(pki->msix_entries[i].vector, pki);
	}
	pci_disable_msix(pki->pdev);
}

static int pki_irq_init(struct pki_t *pki)
{
	int i;
	int ret = 0;

	/*clear intr */
	for (i = 0; i < PKI_MSIX_COUNT; i++)
		pki_reg_write(pki, intr[i].coffset, intr[i].mask);

	pki->msix_entries = devm_kzalloc(&pki->pdev->dev,
					 PKI_MSIX_COUNT *
					 sizeof(struct msix_entry), GFP_KERNEL);

	if (!pki->msix_entries)
		return -ENOMEM;

	for (i = 0; i < PKI_MSIX_COUNT; i++)
		pki->msix_entries[i].entry = i;

	ret = pci_enable_msix_exact(pki->pdev, pki->msix_entries,
				    PKI_MSIX_COUNT);
	if (ret < 0) {
		dev_err(&pki->pdev->dev, "Enabling msix failed\n");
		return ret;
	}

	for (i = 0; i < PKI_MSIX_COUNT; i++) {
		ret = request_irq(pki->msix_entries[i].vector, intr[i].handler,
				  0, intr[i].name, pki);
		if (ret)
			goto free_irq;
	}

	/*enable intr */
	for (i = 0; i < PKI_MSIX_COUNT; i++)
		pki_reg_write(pki, intr[i].soffset, intr[i].mask);

	return 0;
free_irq:
	for ( ; i < PKI_MSIX_COUNT; i++)
		pki->msix_entries[i].vector = 0;
	pki_irq_free(pki);
	return ret;
}

static int pki_init(struct pki_t *pki)
{
	struct pcam *pcam;
	struct pcam_bank *bank;
	int res = 0, i;
	u64 reg;
	u32 delay;

	/* wait till SFT rest is feasable*/
	while (true) {
		if (!pki_reg_read(pki, PKI_SFT_RST))
			break;
	}
	pki_reg_write(pki, PKI_SFT_RST, 0x1);
	/* wait till RST complete */
	while (true) {
		if (!pki_reg_read(pki, PKI_SFT_RST))
			break;
	}

	reg = pki_reg_read(pki, PKI_CONST);
	pki->max_auras = (reg >> PKI_CONST_AURAS_SHIFT) & PKI_CONST_AURAS_MASK;
	pki->max_bpid = (reg >> PKI_CONST_BPID_SHIFT) & PKI_CONST_BPID_MASK;
	pki->max_pkinds = (reg >> PKI_CONST_PKNDS_SHIFT) & PKI_CONST_PKNDS_MASK;
	pki->max_fstyles = (reg >> PKI_CONST_FSTYLES_SHIFT) &
			PKI_CONST_FSTYLES_MASK;

	reg = pki_reg_read(pki, PKI_CONST1);
	pki->max_cls = (reg >> PKI_CONST1_CLS_SHIFT) & PKI_CONST1_CLS_MASK;
	pki->max_ipes = (reg >> PKI_CONST1_IPES_SHIFT) & PKI_CONST1_IPES_MASK;
	pki->max_pcams = (reg >> PKI_CONST1_PCAMS_SHIFT) &
			PKI_CONST1_PCAMS_MASK;

	reg = pki_reg_read(pki, PKI_CONST2);
	pki->max_pcam_ents = (reg >> PKI_CONST2_PCAM_ENTS_SHIFT) &
			PKI_CONST2_PCAM_ENTS_MASK;
	pki->max_qpgs = (reg >> PKI_CONST2_QPGS_SHIFT) & PKI_CONST2_QPGS_MASK;
	pki->max_dstats = (reg >> PKI_CONST2_DSTATS_SHIFT) &
			PKI_CONST2_DSTATS_MASK;
	pki->max_stats = (reg >> PKI_CONST2_STATS_SHIFT) &
			PKI_CONST2_STATS_MASK;

	pki->qpg_domain = vmalloc(sizeof(*pki->qpg_domain) * pki->max_qpgs);
	if (!pki->qpg_domain) {
		res = -ENOMEM;
		goto err;
	}
	memset(pki->qpg_domain, 0, sizeof(*pki->qpg_domain) * pki->max_qpgs);

	pki->loop_pkind_domain = vzalloc(sizeof(*pki->loop_pkind_domain) *
					 (MAX_LBK_LOOP_PKIND + 1));
	if (!pki->loop_pkind_domain) {
		res = -ENOMEM;
		goto err;
	}

	pcam = &pki->pcam;
	for (i = 0; i < 2; i++) {
		bank = &pcam->bank[i];
		bank->rsrc.max = pki->max_pcam_ents;
		bank->rsrc.bmap = kcalloc(BITS_TO_LONGS(bank->rsrc.max),
					  sizeof(long), GFP_KERNEL);
		if (!bank->rsrc.bmap) {
			res = -ENOMEM;
			goto err;
		}

		bank->idx2style = vmalloc(sizeof(*bank->idx2style) *
					  bank->rsrc.max);
		if (!bank->idx2style) {
			kfree(bank->rsrc.bmap);
			res = -ENOMEM;
			goto err;
		}
		memset(bank->idx2style, 0, sizeof(*bank->idx2style) *
						  bank->rsrc.max);
	}
	mutex_init(&pcam->lock);

	load_ucode(pki);
	delay = max(0xa0, (800 / pki->max_cls));
	reg = PKI_ICG_CFG_MAXIPE_USE(0x14) | PKI_ICG_CFG_CLUSTERS(0x3) |
	       PKI_ICG_CFG_PENA(1) | PKI_ICG_CFG_DELAY(delay);
	pki_reg_write(pki, PKI_ICGX_CFG(0), reg);

	setup_ltype_map(pki);
	init_styles(pki);
	init_pcam_entries(pki);
	/*enable PKI*/
	reg = pki_reg_read(pki, PKI_BUF_CTL);
	reg |= 0x1;
	pki_reg_write(pki, PKI_BUF_CTL, reg);

err:
	return res;
}

static int pki_sriov_configure(struct pci_dev *pdev, int numvfs)
{
	struct pki_t *pki = pci_get_drvdata(pdev);
	int ret = -EBUSY;
	int disable = 0;

	if (pki->vfs_in_use != 0)
		return ret;

	ret = 0;
	if (pki->flags & PKI_SRIOV_ENABLED)
		disable = 1;

	if (disable) {
		pci_disable_sriov(pdev);
		pki->flags &= ~PKI_SRIOV_ENABLED;
		pki->total_vfs = 0;
	}

	if (numvfs > 0) {
		ret = pci_enable_sriov(pdev, numvfs);
		if (ret == 0) {
			pki->flags |= PKI_SRIOV_ENABLED;
			pki->total_vfs = numvfs;
			ret = numvfs;
		}
	}

	dev_notice(&pki->pdev->dev, "VFs enabled: %d\n", ret);
	return ret;
}

static int pki_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	struct device *dev = &pdev->dev;
	struct pki_t *pki;
	int err = -ENOMEM;

	pki = devm_kzalloc(dev, sizeof(*pki), GFP_KERNEL);
	if (!pki)
		return err;

	pci_set_drvdata(pdev, pki);
	pki->pdev = pdev;

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
	pki->reg_base = pcim_iomap(pdev, PCI_PKI_CFG_BAR, 0);
	if (!pki->reg_base) {
		dev_err(dev, "Can't map CFG space\n");
		err = -ENOMEM;
		return err;
	}

	/*set PKI ID */
	pki->id = atomic_add_return(1, &pki_count);
	pki->id -= 1;

	err = pki_init(pki);
	if (err) {
		dev_err(dev, "failed init pki\n");
		err = -ENOMEM;
		return err;
	}

	err = pki_irq_init(pki);
	if (err) {
		dev_err(dev, "failed init irqs\n");
		err = -EINVAL;
		return err;
	}

	INIT_LIST_HEAD(&pki->list);
	mutex_lock(&octeontx_pki_devices_lock);
	list_add(&pki->list, &octeontx_pki_devices);
	mutex_unlock(&octeontx_pki_devices_lock);

	return 0;
}

static void pki_remove(struct pci_dev *pdev)
{
	struct pki_t *pki = pci_get_drvdata(pdev);
	struct pki_t *curr;

	if (!pki)
		return;

	mutex_lock(&octeontx_pki_devices_lock);
	list_for_each_entry(curr, &octeontx_pki_devices, list) {
		if (curr == pki) {
			list_del(&pki->list);
			break;
		}
	}
	mutex_unlock(&octeontx_pki_devices_lock);

	pki_sriov_configure(pdev, 0);
	vfree(pki->qpg_domain);
	vfree(pki->loop_pkind_domain);
	vfree(pki->pcam.bank[0].idx2style);
	kfree(pki->pcam.bank[0].rsrc.bmap);
	vfree(pki->pcam.bank[1].idx2style);
	kfree(pki->pcam.bank[1].rsrc.bmap);
	pki_irq_free(pki);
}

/* devices supported */
static const struct pci_device_id pki_id_table[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_CAVIUM, PCI_DEVICE_ID_OCTEONTX_PKI) },
	{ 0, }  /* end of table */
};

static struct pci_driver pki_driver = {
	.name = DRV_NAME,
	.id_table = pki_id_table,
	.probe = pki_probe,
	.remove = pki_remove,
	.sriov_configure = pki_sriov_configure,
};

MODULE_AUTHOR("Tirumalesh Chalamarla");
MODULE_DESCRIPTION("Cavium OCTEONTX input packet parser(PKI) Driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRV_VERSION);
MODULE_DEVICE_TABLE(pci, pki_id_table);

static int __init pki_init_module(void)
{
	pr_info("%s, ver %s\n", DRV_NAME, DRV_VERSION);

	return pci_register_driver(&pki_driver);
}

static void __exit pki_cleanup_module(void)
{
	pci_unregister_driver(&pki_driver);
}

module_init(pki_init_module);
module_exit(pki_cleanup_module);
