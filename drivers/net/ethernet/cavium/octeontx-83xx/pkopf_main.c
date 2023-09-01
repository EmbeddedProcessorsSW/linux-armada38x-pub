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

#include "pko.h"
#include "fpa.h"

#define DRV_NAME "octeontx-pko"
#define DRV_VERSION "1.0"

/* PKO MAC type (enumerated by PKO_LMAC_E) */
#define PKO_MAC_LBK	0
#define PKO_MAC_BGX	1
#define PKO_MAC_HOST	2

#define PKO_CHAN_NULL	0xFFF

#define LBK_CHAN_BASE	0x0
#define LBK_CHAN_RANGE	BIT(8)

#define SDP_MAC_NUM     2
#define SDP_CHAN_BASE	0x400
#define SDP_CHAN_RANGE	BIT(8)

#define BGX_CHAN_BASE	0x800
#define BGX_CHAN_RANGE	BIT(8)

#define PKO_BUFFERS	4096
#define PKO_MAX_PORTS	16 /* Maximum number simultaneously used ports.*/
#define PKO_MAX_DQ_BUFS	(PKO_BUFFERS / PKO_MAX_PORTS)

/* SKID value should be synchronized with DQ setting in RM user program.
 * Each DQ has maxiumum depth of PKO_MAX_DQ_BUFS FPA 4KB buffers.
 * SKID is half of this value.
 */
#define PKO_FC_SKID	(PKO_MAX_DQ_BUFS / 2)

static atomic_t pko_count = ATOMIC_INIT(0);
static DEFINE_MUTEX(octeontx_pko_devices_lock);
static LIST_HEAD(octeontx_pko_devices);

static struct fpapf_com_s *fpapf;
static struct fpavf_com_s *fpavf;
static struct fpavf *fpa;
static int pko_pstree_setup(struct pkopf *pko, int vf, u32 max_frame,
			    int mac_num, int mac_mode, int channel);
static void pko_pstree_teardown(struct pkopf *pko, int vf, int mac_num,
				int channel);

/* In Cavium OcteonTX SoCs, all accesses to the device registers are
 * implicitly strongly ordered.
 * So writeq_relaxed() and readq_relaxed() are safe to use
 * with out any memory barriers.
 */

/* Register read/write APIs */
static void pko_reg_write(struct pkopf *pko, u64 offset, u64 val)
{
	writeq_relaxed(val, pko->reg_base + offset);
}

static u64 pko_reg_read(struct pkopf *pko, u64 offset)
{
	return readq_relaxed(pko->reg_base + offset);
}

static int pko_get_bgx_chan(int bgx, int lmac, int chan)
{
	return BGX_CHAN_BASE + (BGX_CHAN_RANGE * bgx) + (0x10 * lmac) + chan;
}

static int pko_get_bgx_mac(int bgx, int lmac)
{
	/* 3 = PKO BGX base MAC, 0x4 = number of MACs used by BGX. */
	return 3 + (0x4 * bgx) + lmac;
}

static int pko_get_sdp_chan(int sdp, int lmac, int chan)
{
	return  SDP_CHAN_BASE + (chan);
}

static int pko_get_sdp_mac(int sdp, int lmac)
{
	return lmac;
}

static int pko_get_lbk_chan(int lbk_base_chan, int lbk_port)
{
	int chan;

	/* lbk0 ports are 0-15 and are cross connected in PKO channels */
	/* i.e channel 0 connected to 1 and vice versa and so on until */
	/* channel 15 */
	if (lbk_port < LBK_PORT_PN_BASE_IDX)
		chan = (lbk_port & 0x1) ? (lbk_port - 1) : (lbk_port + 1);
	/* lbk1/lbk2 port is connected to base channel id */
	else if (lbk_port < LBK_PORT_PP_LOOP_BASE_IDX)
		chan = lbk_base_chan;
	else /* regular loopback ports */
		chan = lbk_port;

	return chan;
}

static int pko_get_lbk_mac(int lbk)
{
	/* Only LBK0 and LBK2 are connected to PKO, which are mapped
	 * to PKO MAC as follows: LBK0 => MAC0, LBK2 => MAC1.
	 */
	return (lbk) ? 1 : 0;
}

int pkopf_master_send_message(struct mbox_hdr *hdr,
			      union mbox_data *req,
			      union mbox_data *resp,
			      void *master_data,
			      void *add_data)
{
	struct pkopf *pko = master_data;
	int ret;

	if (hdr->coproc == FPA_COPROC) {
		ret = fpapf->receive_message(pko->id, FPA_PKO_DPFI_GMID, hdr,
					     req, resp, add_data);
	} else {
		dev_err(&pko->pdev->dev,
			"PKO message dispatch, wrong VF type\n");
		ret = -1;
	}

	return ret;
}

static struct octeontx_master_com_t pko_master_com = {
	.send_message = pkopf_master_send_message
};

static irqreturn_t pko_ecc_intr_handler(int irq, void *pko_irq)
{
	struct pkopf *pko = (struct pkopf *)pko_irq;

	dev_err(&pko->pdev->dev, "ECC received\n");
	return IRQ_HANDLED;
}

static irqreturn_t pko_peb_err_intr_handler(int irq, void *pko_irq)
{
	struct pkopf *pko = (struct pkopf *)pko_irq;
	u64 reg;

	reg = pko_reg_read(pko, PKO_PF_PEB_ERR_INT_W1C);
	dev_err(&pko->pdev->dev, "val @PKO_PEB_ERR_INT_W1C: %llx\n", reg);

	dev_err(&pko->pdev->dev, "peb err received\n");
	reg = pko_reg_read(pko, PKO_PF_PEB_PAD_ERR_INFO);
	dev_err(&pko->pdev->dev, "peb pad err info:%llx\n", reg);
	reg = pko_reg_read(pko, PKO_PF_PEB_PSE_FIFO_ERR_INFO);
	dev_err(&pko->pdev->dev, "peb pse fifo err info:%llx\n", reg);
	reg = pko_reg_read(pko, PKO_PF_PEB_SUBD_ADDR_ERR_INFO);
	dev_err(&pko->pdev->dev, "peb subd addr err info:%llx\n", reg);
	reg = pko_reg_read(pko, PKO_PF_PEB_SUBD_SIZE_ERR_INFO);
	dev_err(&pko->pdev->dev, "peb subd size err info:%llx\n", reg);
	return IRQ_HANDLED;
}

static irqreturn_t pko_pq_drain_intr_handler(int irq, void *pko_irq)
{
	struct pkopf *pko = (struct pkopf *)pko_irq;

	dev_err(&pko->pdev->dev, "pq drain received\n");
	pko_reg_write(pko, PKO_PF_PQ_DRAIN_W1C, 0x1);
	return IRQ_HANDLED;
}

static irqreturn_t pko_pdm_sts_intr_handler(int irq, void *pko_irq)
{
	struct pkopf *pko = (struct pkopf *)pko_irq;
	u64 reg1, reg2, val;

	reg1 = pko_reg_read(pko, PKO_PF_PDM_STS_INFO);
	reg2 = pko_reg_read(pko, PKO_PF_PDM_STS_W1C);

	/* Ignore DQ not created error as it can happen when we query
	 * DQ status during domain reset
	 */
	val = ((reg2 >> 25) & 0x1) && (((reg1 >> 26) & 0xF) == 0xD);
	if (!val) {
		dev_err(&pko->pdev->dev, "pdm sts received\n");
		dev_err(&pko->pdev->dev, "sts info: %llx\n", reg1);
		dev_err(&pko->pdev->dev, "sts w1c: %llx\n", reg2);
	}

	pko_reg_write(pko, PKO_PF_PDM_STS_W1C, reg2);
	return IRQ_HANDLED;
}

static irqreturn_t pko_pdm_ncb_int_intr_handler(int irq, void *pko_irq)
{
	struct pkopf *pko = (struct pkopf *)pko_irq;
	u64 reg;

	dev_err(&pko->pdev->dev, "pdm ncb int received\n");
	reg = pko_reg_read(pko, PKO_PF_PDM_NCB_MEM_FAULT);
	dev_err(&pko->pdev->dev, "pdm ncb mem fualt:%llx\n", reg);
	reg = pko_reg_read(pko, PKO_PF_PDM_NCB_TX_ERR_WORD);
	dev_err(&pko->pdev->dev, "pdm ncb err word:%llx\n", reg);
	reg = pko_reg_read(pko, PKO_PF_PDM_NCB_TX_ERR_INFO);
	dev_err(&pko->pdev->dev, "pdm ncb err info:%llx\n", reg);
	return IRQ_HANDLED;
}

static irqreturn_t pko_peb_ncb_int_intr_handler(int irq, void *pko_irq)
{
	struct pkopf *pko = (struct pkopf *)pko_irq;
	u64 reg;

	dev_err(&pko->pdev->dev, "peb ncb int received\n");
	reg = pko_reg_read(pko, PKO_PF_PEB_NCB_MEM_FAULT);
	dev_err(&pko->pdev->dev, "peb ncb mem fualt:%llx\n", reg);
	return IRQ_HANDLED;
}

static struct intr_hand intr[] = {
	{0x8000000000000000ULL, "pko lut ecc sbe",
		PKO_PF_LUT_ECC_SBE_INT_ENA_W1C,
		PKO_PF_LUT_ECC_SBE_INT_ENA_W1S, pko_ecc_intr_handler},
	{0x8000000000000000ULL, "pko lut ecc dbe",
		PKO_PF_LUT_ECC_DBE_INT_ENA_W1C,
		PKO_PF_LUT_ECC_DBE_INT_ENA_W1S, pko_ecc_intr_handler},
	{0xffffffffffffffffULL, "pko dq ecc sbe", PKO_PF_DQ_ECC_SBE_INT_ENA_W1C,
		PKO_PF_DQ_ECC_SBE_INT_ENA_W1S, pko_ecc_intr_handler},
	{0xffffffffffffffffULL, "pko dq ecc dbe", PKO_PF_DQ_ECC_DBE_INT_ENA_W1C,
		PKO_PF_DQ_ECC_DBE_INT_ENA_W1S, pko_ecc_intr_handler},
	{0xffffffffffffffffULL, "pko l2 ecc sbe", PKO_PF_L2_ECC_SBE_INT_ENA_W1C,
		PKO_PF_L2_ECC_SBE_INT_ENA_W1S, pko_ecc_intr_handler},
	{0xffffffffffffffffULL, "pko l2 ecc dbe", PKO_PF_L2_ECC_DBE_INT_ENA_W1C,
		PKO_PF_L2_ECC_DBE_INT_ENA_W1S, pko_ecc_intr_handler},
	{0xffffffffffffffffULL, "pko l3 ecc sbe", PKO_PF_L3_ECC_SBE_INT_ENA_W1C,
		PKO_PF_L3_ECC_SBE_INT_ENA_W1S, pko_ecc_intr_handler},
	{0xffffffffffffffffULL, "pko l3 ecc dbe", PKO_PF_L3_ECC_DBE_INT_ENA_W1C,
		PKO_PF_L3_ECC_DBE_INT_ENA_W1S, pko_ecc_intr_handler},
	{0xffffffffffffffffULL, "pko pdm ecc sbe",
		PKO_PF_PDM_ECC_SBE_INT_ENA_W1C,
		PKO_PF_PDM_ECC_SBE_INT_ENA_W1S, pko_ecc_intr_handler},
	{0xffffffffffffffffULL, "pko pdm ecc dbe",
		PKO_PF_PDM_ECC_DBE_INT_ENA_W1C,
		PKO_PF_PDM_ECC_DBE_INT_ENA_W1S, pko_ecc_intr_handler},
	{0xffffffffffffffffULL, "pko l1 ecc sbe", PKO_PF_L1_ECC_SBE_INT_ENA_W1C,
		PKO_PF_L1_ECC_SBE_INT_ENA_W1S, pko_ecc_intr_handler},
	{0xffffffffffffffffULL, "pko l1 ecc dbe", PKO_PF_L1_ECC_DBE_INT_ENA_W1C,
		PKO_PF_L1_ECC_DBE_INT_ENA_W1S, pko_ecc_intr_handler},
	{0xffffffffffffffffULL, "pko pq ecc sbe", PKO_PF_PQ_ECC_SBE_INT_ENA_W1C,
		PKO_PF_PQ_ECC_SBE_INT_ENA_W1S, pko_ecc_intr_handler},
	{0xffffffffffffffffULL, "pko pq ecc dbe", PKO_PF_PQ_ECC_DBE_INT_ENA_W1C,
		PKO_PF_PQ_ECC_DBE_INT_ENA_W1S, pko_ecc_intr_handler},
	{0xffffffffffffffffULL, "pko pdmncb eccsbe",
		PKO_PF_PDM_NCB_ECC_SBE_INT_ENA_W1C,
		PKO_PF_PDM_NCB_ECC_SBE_INT_ENA_W1S, pko_ecc_intr_handler},
	{0xffffffffffffffffULL, "pko pdmncb eccdbe",
		PKO_PF_PDM_NCB_ECC_DBE_INT_ENA_W1C,
		PKO_PF_PDM_NCB_ECC_DBE_INT_ENA_W1S, pko_ecc_intr_handler},
	{0xffffffffffffffffULL, "pko peb sbe", PKO_PF_PEB_ECC_SBE_INT_ENA_W1C,
		PKO_PF_PEB_ECC_SBE_INT_ENA_W1S, pko_ecc_intr_handler},
	{0xffffffffffffffffULL, "pko peb dbe", PKO_PF_PEB_ECC_DBE_INT_ENA_W1C,
		PKO_PF_PEB_ECC_DBE_INT_ENA_W1S, pko_ecc_intr_handler},
	{0x3ffULL, "pko peb err", PKO_PF_PEB_ERR_INT_ENA_W1C,
		PKO_PF_PEB_ERR_INT_ENA_W1S, pko_peb_err_intr_handler},
	{0x1ULL, "pko pq drain", PKO_PF_PQ_DRAIN_INT_ENA_W1C,
		PKO_PF_PQ_DRAIN_INT_ENA_W1S, pko_pq_drain_intr_handler},
	{0x270200C209ULL, "pko pdm sts", PKO_PF_PDM_STS_INT_ENA_W1C,
		PKO_PF_PDM_STS_INT_ENA_W1S, pko_pdm_sts_intr_handler},
	{0x7ULL, "pko pdm ncb int", PKO_PF_PDM_NCB_INT_ENA_W1C,
		PKO_PF_PDM_NCB_INT_ENA_W1S, pko_pdm_ncb_int_intr_handler},
	{0x4ULL, "pko peb ncb int", PKO_PF_PEB_NCB_INT_ENA_W1C,
		PKO_PF_PEB_NCB_INT_ENA_W1S, pko_peb_ncb_int_intr_handler},
	{0xffffffffffffffffULL, "pko pebncb eccsbe",
		PKO_PF_PEB_NCB_ECC_SBE_INT_ENA_W1C,
		PKO_PF_PEB_NCB_ECC_SBE_INT_ENA_W1S, pko_ecc_intr_handler},
	{0xffffffffffffffffULL, "pko pebncb eccdbe",
		PKO_PF_PEB_NCB_ECC_DBE_INT_ENA_W1C,
		PKO_PF_PEB_NCB_ECC_DBE_INT_ENA_W1S, pko_ecc_intr_handler}

};

static void identify(struct pkopf_vf *vf, u16 domain_id,
		     u16 subdomain_id)
{
	u64 reg = ((subdomain_id) << 16 | (domain_id)) << 7;

	writeq_relaxed(reg, vf->domain.reg_base + PKO_VF_DQ_FC_CONFIG);
}

static int pko_pf_destroy_domain(u32 id, u16 domain_id, struct kobject *kobj)
{
	struct pkopf *pko = NULL;
	struct pci_dev *virtfn;
	struct pkopf *curr;
	int i, vf_idx = 0;

	mutex_lock(&octeontx_pko_devices_lock);
	list_for_each_entry(curr, &octeontx_pko_devices, list) {
		if (curr->id == id) {
			pko = curr;
			break;
		}
	}

	if (!pko) {
		mutex_unlock(&octeontx_pko_devices_lock);
		return -ENODEV;
	}

	for (i = 0; i < pko->total_vfs; i++) {
		if (pko->vf[i].domain.in_use &&
		    pko->vf[i].domain.domain_id == domain_id) {
			pko->vf[i].domain.in_use = false;
			pko_pstree_teardown(pko, i, pko->vf[i].mac_num,
					    pko->vf[i].chan);
			identify(&pko->vf[i], 0x0, 0x0);
			iounmap(pko->vf[i].domain.reg_base);

			virtfn = pci_get_domain_bus_and_slot(pci_domain_nr
					(pko->pdev->bus),
					pci_iov_virtfn_bus(pko->pdev, i),
					pci_iov_virtfn_devfn(pko->pdev, i));
			if (virtfn && kobj)
				sysfs_remove_link(kobj, virtfn->dev.kobj.name);

			dev_info(&pko->pdev->dev,
				 "Free vf[%d] from domain:%d subdomain_id:%d\n",
				 i, pko->vf[i].domain.domain_id, vf_idx);
			vf_idx++;
		}
	}

	pko->vfs_in_use -= vf_idx;
	mutex_unlock(&octeontx_pko_devices_lock);

	return 0;
}

static void pko_pf_gmctl_init(struct pkopf *pf, int vf, u16 gmid)
{
	u64 reg;

	/* Write stream and GMID settings */
	reg = PKO_PF_VFX_GMCTL_GET_BE(pko_reg_read(pf, PKO_PF_VFX_GMCTL(vf)));
	reg = PKO_PF_VFX_GMCTL_BE(reg) | PKO_PF_VFX_GMCTL_GMID(gmid) |
	      PKO_PF_VFX_GMCTL_STRM(vf + 1);
	pko_reg_write(pf, PKO_PF_VFX_GMCTL(vf), reg);
	reg = pko_reg_read(pf, PKO_PF_VFX_GMCTL(vf));
}

static u64 pko_pf_create_domain(u32 id, u16 domain_id, u32 pko_vf_count,
				struct octtx_bgx_port *bgx_port, int bgx_count,
				struct octtx_lbk_port *lbk_port, int lbk_count,
				struct octtx_sdp_port *sdp_port, int sdp_count,
				void *master, void *master_data,
				struct kobject *kobj)
{
	struct pkopf *pko = NULL;
	struct pkopf *curr;
	struct pci_dev *virtfn;
	resource_size_t vf_start;
	int i, pko_mac, tx_fifo_len = 0;
	int vf_idx = 0, port_idx = 0;
	int mac_num, mac_mode, chan, ret = 0;
	const u32 max_frame = 0xffff;
	unsigned long pko_mask = 0;

	if (!kobj)
		return 0;

	if (sdp_count)
		pko_mac = PKO_MAC_HOST;
	else if (bgx_count)
		pko_mac = PKO_MAC_BGX;
	else
		pko_mac = PKO_MAC_LBK;

	mutex_lock(&octeontx_pko_devices_lock);
	list_for_each_entry(curr, &octeontx_pko_devices, list) {
		if (curr->id == id) {
			pko = curr;
			break;
		}
	}

	if (!pko)
		goto err_unlock;

	/**
	 * pko_vfs are partitioned among SDP, BGX and LBK as:
	 *   SDP <= PKO_VF[(0) .. (sdp_count-1)]
	 *   BGX <= PKO_VF[(sdp_count) .. (sdp_count + bgx_count - 1)]
	 *   LBK <= PKO_VF[(sdp_count + bgx_count) .. (sdp_count + bgx_count +
	 *                                             lbk_count- 1)]
	 */
	for (i = 0; i < pko->total_vfs; i++) {
		if (pko->vf[i].domain.in_use) {
			continue;
		} else {
			virtfn = pci_get_domain_bus_and_slot(pci_domain_nr
					   (pko->pdev->bus),
					   pci_iov_virtfn_bus(pko->pdev, i),
					   pci_iov_virtfn_devfn(pko->pdev, i));
			if (!virtfn)
				break;

			ret = sysfs_create_link(kobj, &virtfn->dev.kobj,
						virtfn->dev.kobj.name);
			if (ret < 0)
				goto err_unlock;

			pko->vf[i].domain.domain_id = domain_id;
			pko->vf[i].domain.subdomain_id = vf_idx;
			pko->vf[i].domain.gmid = get_gmid(domain_id);

			pko->vf[i].domain.in_use = true;
			pko->vf[i].domain.master = master;
			pko->vf[i].domain.master_data = master_data;

			vf_start = pci_resource_start(pko->pdev,
						      PCI_PKO_PF_CFG_BAR);
			vf_start += PKO_VF_OFFSET(i);

			pko->vf[i].domain.reg_base =
				ioremap_wc(vf_start, PKO_VF_CFG_SIZE);

			if (!pko->vf[i].domain.reg_base)
				break;

			identify(&pko->vf[i], domain_id, vf_idx);
			pko_pf_gmctl_init(pko, i, get_gmid(domain_id));

			/* Setup the PKO Scheduling tree: PQ/SQ/DQ.
			 */
			if (pko_mac == PKO_MAC_HOST) {
				mac_num =
				pko_get_sdp_mac(sdp_port[port_idx].sdp,
						sdp_port[port_idx].lmac);
				chan = pko_get_sdp_chan(sdp_port[port_idx].sdp,
							sdp_port[port_idx].lmac,
							0);
				mac_mode = sdp_port[port_idx].lmac_type;
				port_idx++;
				if (port_idx >= sdp_count) {
					pko_mac = bgx_count ? PKO_MAC_BGX :
						  PKO_MAC_LBK;
					port_idx = 0;
				}
			} else if (pko_mac == PKO_MAC_BGX) {
				mac_num =
				pko_get_bgx_mac(bgx_port[port_idx].bgx,
						bgx_port[port_idx].lmac);
				chan = pko_get_bgx_chan(bgx_port[port_idx].bgx,
							bgx_port[port_idx].lmac,
							0);
				mac_mode = bgx_port[port_idx].lmac_type;
				tx_fifo_len = bgx_port[port_idx].tx_fifo_sz;
				port_idx++;
				if (port_idx >= bgx_count) {
					pko_mac = PKO_MAC_LBK;
					port_idx = 0;
				}
			} else if (pko_mac == PKO_MAC_LBK) {
				mac_num =
				pko_get_lbk_mac(lbk_port[port_idx].olbk);
				chan = pko_get_lbk_chan
					(lbk_port[port_idx].olbk_base_chan,
					 lbk_port[port_idx].glb_port_idx);
				mac_mode = 0;
				port_idx++;
			} else {
				break;
			}
			pko->vf[i].mac_num = mac_num;
			pko->vf[i].chan = chan;
			pko->vf[i].tx_fifo_sz = tx_fifo_len;

			dev_dbg(&pko->pdev->dev,
				"i: %d, max_frame: %d, mac_num: %d, mac_mode: %d, chan: %d\n",
				i, max_frame, mac_num, mac_mode, chan);

			pko_pstree_setup(pko, i, max_frame,
					 mac_num, mac_mode, chan);
			set_bit(i, &pko_mask);
			vf_idx++;
			if (vf_idx == pko_vf_count) {
				pko->vfs_in_use += pko_vf_count;
				break;
			}
		}
	}

	if (vf_idx != pko_vf_count)
		goto err_unlock;

	mutex_unlock(&octeontx_pko_devices_lock);
	return pko_mask;

err_unlock:
	mutex_unlock(&octeontx_pko_devices_lock);
	pko_pf_destroy_domain(id, domain_id, kobj);
	return 0;
}

/*caller is responsible for locks
 */
static struct pkopf_vf *get_vf(u32 id, u16 domain_id, u16 subdomain_id,
			       struct pkopf **master)
{
	struct pkopf *pko = NULL;
	struct pkopf *curr;
	int i;
	int vf_idx = -1;

	list_for_each_entry(curr, &octeontx_pko_devices, list) {
		if (curr->id == id) {
			pko = curr;
			break;
		}
	}

	if (!pko)
		return NULL;

	for (i = 0; i < pko->total_vfs; i++) {
		if (pko->vf[i].domain.domain_id == domain_id &&
		    pko->vf[i].domain.subdomain_id == subdomain_id) {
			vf_idx = i;
			if (master)
				*master = pko;
			break;
		}
	}
	if (vf_idx >= 0)
		return &pko->vf[vf_idx];
	else
		return NULL;
}

static inline void set_field(u64 *ptr, u64 field_mask, u8 field_shift, u64 val)
{
	*ptr &= ~(field_mask << field_shift);
	*ptr |= (val & field_mask) << field_shift;
}

static int pko_port_mtu_cfg(struct pkopf *pko, struct pkopf_vf *vf, u16 vf_id,
			    mbox_pko_mtu_cfg_t *cfg)
{
	int queue_base = vf_id * 8;
	u64 reg, tx_credit;
	int mac_num;

	if (cfg->mtu > OCTTX_HW_MAX_FRS || cfg->mtu < OCTTX_HW_MIN_FRS)
		return MBOX_RET_INVALID;

	mac_num = vf->mac_num;

	tx_credit = ((vf->tx_fifo_sz) - cfg->mtu) / 16;

	/* Setting up the channel credit for L1_SQ */
	reg = pko_reg_read(pko, PKO_PF_L1_SQX_LINK(mac_num));
	if (mac_num > SDP_MAC_NUM) /* BGX MAC specific configuration */
		set_field(&reg, PKO_PF_CC_WORD_CNT_MASK,
			  PKO_PF_CC_WORD_CNT_SHIFT, tx_credit);
	pko_reg_write(pko, PKO_PF_L1_SQX_LINK(mac_num), reg);

	dev_dbg(&pko->pdev->dev, "PKO: VF[%d] L1_SQ[%d]_LINK ::0x%llx\n",
		vf_id, mac_num, pko_reg_read(pko, PKO_PF_L1_SQX_LINK(mac_num)));

	/* Setting up the channel credit for L3_L2_SQ */
	reg = pko_reg_read(pko, PKO_PF_L3_L2_SQX_CHANNEL(queue_base));
	if (mac_num > SDP_MAC_NUM) /* BGX MAC specific configuration */
		set_field(&reg, PKO_PF_CC_WORD_CNT_MASK,
			  PKO_PF_CC_WORD_CNT_SHIFT, tx_credit);
	pko_reg_write(pko, PKO_PF_L3_L2_SQX_CHANNEL(queue_base), reg);
	dev_dbg(&pko->pdev->dev, "PKO: L3_L2_SQ[%d]_CHANNEL ::0x%llx\n",
		queue_base, pko_reg_read(pko,
					 PKO_PF_L3_L2_SQX_CHANNEL(queue_base)));

	return MBOX_RET_SUCCESS;
}

static int pko_pf_receive_message(u32 id, u16 domain_id,
				  struct mbox_hdr *hdr,
				  union mbox_data *req,
				  union mbox_data *resp,
				  void *mdata)
{
	struct pkopf_vf *vf;
	struct pkopf *pko = NULL;

	mutex_lock(&octeontx_pko_devices_lock);

	vf = get_vf(id, domain_id, hdr->vfid, &pko);

	if (!vf) {
		hdr->res_code = MBOX_RET_INVALID;
		mutex_unlock(&octeontx_pko_devices_lock);
		return -ENODEV;
	}

	resp->data = 0;
	hdr->res_code = MBOX_RET_SUCCESS;

	switch (hdr->msg) {
	case IDENTIFY:
		identify(vf, domain_id, hdr->vfid);
		break;
	case MBOX_PKO_MTU_CONFIG:
		hdr->res_code = pko_port_mtu_cfg(pko, vf, hdr->vfid, mdata);
		break;
	default:
		hdr->res_code = MBOX_RET_INVALID;
	}

	mutex_unlock(&octeontx_pko_devices_lock);
	return 0;
}

static int pko_pf_get_vf_count(u32 id)
{
	struct pkopf *pko = NULL;
	struct pkopf *curr;

	mutex_lock(&octeontx_pko_devices_lock);
	list_for_each_entry(curr, &octeontx_pko_devices, list) {
		if (curr->id == id) {
			pko = curr;
			break;
		}
	}

	if (!pko) {
		mutex_unlock(&octeontx_pko_devices_lock);
		return 0;
	}

	mutex_unlock(&octeontx_pko_devices_lock);
	return pko->total_vfs;
}

static inline uint64_t reg_ldadd_u64(void *addr, int64_t off)
{
	u64 old_val;

	__asm__ volatile("  .cpu		generic+lse\n"
			 "  ldadd	%1, %0, [%2]\n"
			 : "=r" (old_val) : "r" (off), "r" (addr) : "memory");
	return old_val;
}

int pko_reset_domain(u32 id, u16 domain_id)
{
	struct pkopf *pko = NULL;
	struct pkopf *curr;
	u64 reg;
	int retry, queue_base;
	int i, j, mac_num;

	mutex_lock(&octeontx_pko_devices_lock);
	list_for_each_entry(curr, &octeontx_pko_devices, list) {
		if (curr->id == id) {
			pko = curr;
			break;
		}
	}

	if (!pko) {
		mutex_unlock(&octeontx_pko_devices_lock);
		return -ENODEV;
	}

	for (i = 0; i < pko->total_vfs; i++) {
		if (pko->vf[i].domain.in_use &&
		    pko->vf[i].domain.domain_id == domain_id) {
			/* TODO When traffic manager work is completed channel
			 * credits have to be preserved during domain reset
			 */

			mac_num = pko->vf[i].mac_num;
			queue_base = i * DQS_PER_VF;

			/* change link to NULL_FIFO */
			pko_reg_write(pko, PKO_PF_L1_SQX_SW_XOFF(mac_num),
				      0x1);
			reg = pko_reg_read(pko,
					   PKO_PF_L1_SQX_TOPOLOGY(mac_num));
			set_field(&reg, PKO_PF_TOPOLOGY_LINK_MASK,
				  PKO_PF_TOPOLOGY_LINK_SHIFT, NULL_FIFO);
			pko_reg_write(pko, PKO_PF_L1_SQX_TOPOLOGY(mac_num),
				      reg);

			/* clear XOFF bit at each level starting from DQs */
			for (j = 0; j < DQS_PER_VF; j++) {
				writeq_relaxed(0x0,
					       pko->vf[i].domain.reg_base +
					       PKO_VF_DQX_SW_XOFF(j));
			}
			pko_reg_write(pko, PKO_PF_L3_SQX_SW_XOFF(queue_base),
				      0x0);
			pko_reg_write(pko, PKO_PF_L2_SQX_SW_XOFF(queue_base),
				      0x0);
			pko_reg_write(pko, PKO_PF_L1_SQX_SW_XOFF(mac_num),
				      0x0);

			/* wait for DQs to clear */
			j = 0;
			retry = 0;
			while (j < DQS_PER_VF) {
				reg = readq_relaxed(pko->vf[i].domain.reg_base +
						    PKO_VF_DQX_WM_CNT(j));
				if (!reg)
					j++;
				mdelay(10);
				if (retry++ > 10) {
					dev_err(&pko->pdev->dev, "Failed to clear DQ domain_id:%d, vf_id:%d, dq_id:%d\n",
						pko->vf[i].domain.domain_id,
						i, j);
					break;
				}
			}

			/* wait for L1 SQ node to clear from meta */
			retry = 0;
			while (true) {
				reg = pko_reg_read(pko,
						   PKO_PF_L1_SQX_PICK(mac_num));
				reg = (reg >> PKO_PF_PICK_ADJUST_SHIFT) &
				       PKO_PF_PICK_ADJUST_MASK;

				if (reg != PKO_PF_VALID_META)
					break;
				mdelay(10);
				if (retry++ > 10) {
					dev_err(&pko->pdev->dev, "Failed to clear L1 SQ node domain_id:%d, vf_id:%d\n",
						pko->vf[i].domain.domain_id, i);
					break;
				}
			}

			/* try to close DQs if they are open */
			for (j = 0; j < DQS_PER_VF; j++) {
				reg = readq_relaxed(pko->vf[i].domain.reg_base
						    + PKO_VF_DQX_OP_QUERY(j));
				reg = (reg >> PKO_VF_DQ_OP_DQSTATUS_SHIFT) &
				       PKO_VF_DQ_OP_DQSTATUS_MASK;
				if (!reg)
					reg_ldadd_u64(pko->vf[i].domain.reg_base
						      + PKO_VF_DQX_OP_CLOSE(j),
						      0x0);
			}

			/* change link back to original value */
			reg = pko_reg_read(pko,
					   PKO_PF_L1_SQX_TOPOLOGY(mac_num));
			set_field(&reg, PKO_PF_TOPOLOGY_LINK_MASK,
				  PKO_PF_TOPOLOGY_LINK_SHIFT, mac_num);
			pko_reg_write(pko, PKO_PF_L1_SQX_TOPOLOGY(mac_num),
				      reg);

			identify(&pko->vf[i], domain_id,
				 pko->vf[i].domain.subdomain_id);
		}
	}

	mutex_unlock(&octeontx_pko_devices_lock);
	return 0;
}

struct pkopf_com_s pkopf_com  = {
	.create_domain = pko_pf_create_domain,
	.destroy_domain = pko_pf_destroy_domain,
	.reset_domain = pko_reset_domain,
	.receive_message = pko_pf_receive_message,
	.get_vf_count = pko_pf_get_vf_count
};
EXPORT_SYMBOL(pkopf_com);

static void pko_irq_free(struct pkopf *pko)
{
	int i;

	/*clear intr */
	for (i = 0; i < PKO_MSIX_COUNT; i++) {
		pko_reg_write(pko, intr[i].coffset, intr[i].mask);
		if (pko->msix_entries[i].vector)
			free_irq(pko->msix_entries[i].vector, pko);
	}
	pci_disable_msix(pko->pdev);
}

static int pko_irq_init(struct pkopf *pko)
{
	int i;
	int ret = 0;

	/*clear intr */
	for (i = 0; i < PKO_MSIX_COUNT; i++)
		pko_reg_write(pko, intr[i].coffset, intr[i].mask);

	pko->msix_entries = devm_kzalloc(&pko->pdev->dev,
					 PKO_MSIX_COUNT *
					 sizeof(struct msix_entry), GFP_KERNEL);

	if (!pko->msix_entries)
		return -ENOMEM;

	for (i = 0; i < PKO_MSIX_COUNT; i++)
		pko->msix_entries[i].entry = i;

	ret = pci_enable_msix_exact(pko->pdev, pko->msix_entries,
				    PKO_MSIX_COUNT);
	if (ret < 0) {
		dev_err(&pko->pdev->dev, "Enabling msix failed\n");
		return ret;
	}

	for (i = 0; i < PKO_MSIX_COUNT; i++) {
		ret = request_irq(pko->msix_entries[i].vector, intr[i].handler,
				  0, intr[i].name, pko);
		if (ret)
			goto free_irq;
	}

	/*enable intr */
	for (i = 0; i < PKO_MSIX_COUNT; i++)
		pko_reg_write(pko, intr[i].soffset, intr[i].mask);

	return 0;
free_irq:
	for (; i < PKO_MSIX_COUNT; i++)
		pko->msix_entries[i].vector = 0;
	pko_irq_free(pko);
	return ret;
}

static int pko_mac_init(struct pkopf *pko, int mac_num, int mac_mode)
{
	u64 reg, fifo;
	int ptgf;
	u64 bgx_txfifo_sz = 0xC000; /* BGX(x)_CONST[tx_fifosz] */
	u64 fifo_size = 2500; /* 2.5KB */
	u64 size = 0; /* {2.5, 2.5, 2.5, 2.5}KB */
	u64 skid = 0x0; /* 16 */
	u64 rate = 0x0; /* 6.25 Gpbs (6 inflight packets) */
	u64 min_pad = 0x1; /* Minimum padding enable */

	/* 1. FIFO group assignment map:
	 * G0 (FIFOs: 0,1,2,3)     -- BGX0            (mac_num = 3,4,5,6)
	 * G1 (FIFOs: 4,5,6,7)     -- BGX1            (mac_num = 7,8,9,10)
	 * G2 (FIFOs: 8,9,10,11)   -- BGX2            (mac_num = 11,12,13,14)
	 * G3 (FIFOs: 12,13,14,15) -- BGX3            (mac_num = 15,16,17,18)
	 * G4 (FIFOs: 16,17,18,19) -- LBK0, LBK1, DPI (mac_num = 0,1,2)
	 * G5 (FIFOs: Virtual)     -- NULL            (mac_num = 19)
	 *
	 * 2. TODO: The combined bit rate among all FIFOs should not exceed
	 * 125 Gbps (80 inflight packets).
	 */
	if (mac_num >= 3 && mac_num <= 18) { /* BGX */
		fifo = mac_num - 3;
		ptgf = fifo / 4;
		switch (mac_mode) {
		case OCTTX_BGX_LMAC_TYPE_40GR:
			fifo_size = 10000; /* 10KB */
			size = 4; /* {10.0, ---, ---, ---}KB */
			rate = 0x3; /* 50 Gpbs (48 inflight packets) */
			skid = 0x2; /* 64 */
			break;
		case OCTTX_BGX_LMAC_TYPE_XAUI: /* Or DXAUI */
			fifo_size = 10000; /* 10KB */
			size = 4; /* {10.0, ---, ---, ---}KB */
			rate = 0x2; /* 25.0 Gpbs (24 inflight packets) */
			skid = 0x1; /* 32 */
			break;
		case OCTTX_BGX_LMAC_TYPE_RXAUI:
			/* TODO: RXAUI takes two BGX LMACs. Thus, the proper
			 * FIFO-LMAC map would be using 2 * 5KB FIFOs (size=3).
			 * Though, currently, there is some mess with the RXAUI
			 * setup in U-Boot and THUNDER driver, so, for now,
			 * it is safer to go with the default map.
			 */
			fifo_size = 2500; /* 2.5KB */
			size = 0; /* {2.5, 2.5, 2.5, 2.5}KB */
			rate = 0x2; /* 25.0 Gpbs (24 inflight packets) */
			skid = 0x1; /* 32 */
			bgx_txfifo_sz /= 2;
			break;
		case OCTTX_BGX_LMAC_TYPE_10GR: /* XFI */
			fifo_size = 2500; /* 2.5KB */
			size = 0; /* {2.5, 2.5, 2.5, 2.5}KB */
			rate = 0x1; /* 12.5 Gpbs (12 inflight packets) */
			skid = 0x1; /* 32 */
			bgx_txfifo_sz /= 4;
			break;
		default: /* SGMII, ... */
			bgx_txfifo_sz /= 4;
			break;
		}
	} else if (mac_num >= 0 && mac_num <= 2) { /* LBK/SDP */
		fifo = mac_num + 16;
		ptgf = 4;
		size = 2; /* {2.5, 2.5, 5.0, ---}KB */
		if (mac_num == SDP_MAC_NUM) { /* SDP */
			rate = 0x3; /* 50 Gpbs (48 inflight packets) */
			skid = 0x2; /* 64 */
			min_pad = 0x0;
		}
	} else if (mac_num == 19) { /* NULL */
		fifo = 19;
		ptgf = 5;
	} else {
		return -EINVAL;
	}
	reg = fifo | (skid << 5) | (0x0 << 15) | (min_pad << 16);
	pko_reg_write(pko, PKO_PF_MACX_CFG(mac_num), reg);

	if (mac_num == SDP_MAC_NUM) { /* SDP MAC specific configuration */
		/* SDP bug #28683 in mcbuggin  */
		reg = 32; /* MAX_CRED_LIM */
		dev_dbg(&pko->pdev->dev, "  write %016llx PKO_MCI1_MAX_CRED%d\n",
			reg, mac_num);
		pko_reg_write(pko, PKO_PF_MCI1_MAX_CREDX(mac_num), reg);
	} else {  /* BGX MAC specific configuration */
		reg = bgx_txfifo_sz / 16; /* MAX_CRED_LIM */
		pko_reg_write(pko, PKO_PF_MCI1_MAX_CREDX(mac_num), reg);
	}

	reg = (rate << 3) | size;
	pko_reg_write(pko, PKO_PF_PTGFX_CFG(ptgf), reg);

	reg = (1ull << 63)
	       | fpa->pool_iova /* dummy read address -- required by any
				 * descriptor segment that does not have
				 * a native data read fetch associated
				 * with it (eg. SEND_JUMP, SEND_IMM).
				 */
	       | 0x10; /* 0x10 -- recommended in HRM.*/
	       /* Note: For XFI interface, this value may be big and can create
		* "underflow" condition in the BGX TX FIFO. If this happens,
		* use value = 3..6.
		*/
	pko_reg_write(pko, PKO_PF_PTF_IOBP_CFG, reg);

	return 0;
}

static void pko_mac_teardown(struct pkopf *pko, int mac_num)
{
	pko_reg_write(pko, PKO_PF_MACX_CFG(mac_num), 0x1F);
	pko_reg_write(pko, PKO_PF_MCI1_MAX_CREDX(mac_num), 0x0);
}

static void pko_pq_init(struct pkopf *pko, int vf, int mac_num, u32 max_frame)
{
	u64 queue_base = vf * 8;
	u64 reg, tx_credit;

	/* Non-BGX links perform DDWR on prio 0 */
	reg = (mac_num << 16) | (queue_base << 32);
	/* BGX MACs have only a single child, so PRIO must be 0xF */
	if (mac_num > SDP_MAC_NUM)
		reg |= 0xFull << 1;

	pko_reg_write(pko, PKO_PF_L1_SQX_TOPOLOGY(mac_num), reg);

	dev_dbg(&pko->pdev->dev, "PKO: VF[%d] L1_SQ[%d]_TOPOLOGY ::0x%llx\n",
		vf, mac_num,
		pko_reg_read(pko, PKO_PF_L1_SQX_TOPOLOGY(mac_num)));

	reg = (mac_num << 13);
	pko_reg_write(pko, PKO_PF_L1_SQX_SHAPE(mac_num), reg);

	dev_dbg(&pko->pdev->dev, "PKO: VF[%d] L1_SQ[%d]_SHAPE ::0x%llx\n",
		vf, mac_num, pko_reg_read(pko, PKO_PF_L1_SQX_SHAPE(mac_num)));

	if (mac_num != SDP_MAC_NUM) {  /* BGX MAC specific configuration */
		reg = min(max_frame + 40, (u32)0xffffff);
		pko_reg_write(pko, PKO_PF_L1_SQX_SCHEDULE(mac_num), reg);
		dev_dbg(&pko->pdev->dev,
			"PKO: VF[%d] L1_SQ[%d]_SCHEDULE ::0x%llx\n",
			vf, mac_num,
			pko_reg_read(pko, PKO_PF_L1_SQX_SCHEDULE(mac_num)));
	}

	/* Setting up the channel credit for L1_SQ */
	reg = (mac_num | 0ULL) << 44;
	if (mac_num > SDP_MAC_NUM) {  /* BGX MAC specific configuration */
		tx_credit = ((pko->vf[vf].tx_fifo_sz) - OCTTX_HW_MAX_FRS) / 16;
		/* Enable credits and set credit pkt count to max allowed */
		reg |=  (tx_credit << 12) | (0x1FF << 2) | BIT_ULL(1);
	}
	pko_reg_write(pko, PKO_PF_L1_SQX_LINK(mac_num), reg);

	dev_dbg(&pko->pdev->dev, "PKO: VF[%d] L1_SQ[%d]_LINK ::0x%llx\n",
		vf, mac_num, pko_reg_read(pko, PKO_PF_L1_SQX_LINK(mac_num)));
}

static void pko_pq_teardown(struct pkopf *pko, int mac_num)
{
	u64 reg;

	reg = 0x13 << 16;
	pko_reg_write(pko, PKO_PF_L1_SQX_TOPOLOGY(mac_num), reg);
	pko_reg_write(pko, PKO_PF_L1_SQX_SHAPE(mac_num), 0x0);
	pko_reg_write(pko, PKO_PF_L1_SQX_SCHEDULE(mac_num), 0x0);
	pko_reg_write(pko, PKO_PF_L1_SQX_LINK(mac_num), 0x0);
}

static void pko_lX_set_schedule(struct pkopf *pko, int level, int q, u64 reg)
{
	dev_dbg(&pko->pdev->dev, "  write %016llx PKO_L%d_SQ%d_SCHEDULE\n",
		reg, level, q);
	switch (level) {
	case 2:
		pko_reg_write(pko, PKO_PF_L2_SQX_SCHEDULE(q), reg);
		break;
	case 3:
		pko_reg_write(pko, PKO_PF_L3_SQX_SCHEDULE(q), reg);
		break;
	case 4:
	case 5:
		break;
	}
}

static void pko_lX_set_topology(struct pkopf *pko, int level, int q, u64 reg)
{
	dev_dbg(&pko->pdev->dev, "  write %016llx PKO_L%d_SQ%d_TOPOLOGY\n",
		reg, level, q);
	switch (level) {
	case 1:
		pko_reg_write(pko, PKO_PF_L1_SQX_TOPOLOGY(q), reg);
		break;
	case 2:
		pko_reg_write(pko, PKO_PF_L2_SQX_TOPOLOGY(q), reg);
		break;
	case 3:
		pko_reg_write(pko, PKO_PF_L3_SQX_TOPOLOGY(q), reg);
		break;
	case 4:
	case 5:
		break;
	}
}

static void pko_lX_set_shape(struct pkopf *pko, int level, int q, u64 reg)
{
	dev_dbg(&pko->pdev->dev, "  write %016llx PKO_L%d_SQ%d_SHAPE\n",
		reg, level, q);
	switch (level) {
	case 2:
		pko_reg_write(pko, PKO_PF_L2_SQX_SHAPE(q), reg);
		break;
	case 3:
		pko_reg_write(pko, PKO_PF_L3_SQX_SHAPE(q), reg);
		break;
	case 4:
	case 5:
		break;
	}
}

static int pko_sq_init(struct pkopf *pko, int vf, int level, u32 channel,
		       int mac_num, u32 max_frame, int parent_sq)
{
	int channel_level;
	int queue_base;
	u64 reg, tx_credit;

	channel_level = pko_reg_read(pko, PKO_PF_CHANNEL_LEVEL);
	channel_level += 2;

	dev_dbg(&pko->pdev->dev, "%s: channel_level: %d\n",
		__func__, channel_level);
	if (mac_num != SDP_MAC_NUM)  /* BGX MAC specific configuration */
		queue_base = (vf * 8);
	else  /* SDP MAC specific configuration */
		queue_base = (vf * 8) + (channel & 0x3f);

	reg = min(max_frame + 40, (u32)0xffffff);
	pko_lX_set_schedule(pko, level, queue_base, reg);

	reg = 0;
	pko_lX_set_shape(pko, level, queue_base, reg);

	reg = (parent_sq << 16);

	if (mac_num == SDP_MAC_NUM) {  /* SDP MAC specific configuration */
		reg |= ((0ULL | queue_base) << 32);
	} else {  /* BGX MAC specific configuration */
		if (level != pko->max_levels) {
			reg |= ((0ULL | queue_base) << 32);
			reg |= (0xf << 1);
		}
	}
	pko_lX_set_topology(pko, level, queue_base, reg);

	dev_dbg(&pko->pdev->dev,
		"%s: level: %d, channel_level: %d pko->max_levels: %d\n",
		 __func__, level, channel_level, pko->max_levels);
	if (level == channel_level) {
		reg = ((channel | 0ULL) & 0xffful) << 32;
		/* BGX MAC specific configuration */
		if (mac_num > SDP_MAC_NUM) {
			tx_credit = ((pko->vf[vf].tx_fifo_sz) -
				     OCTTX_HW_MAX_FRS) / 16;
			/* Enable cc and set credit pkt count to max allowed */
			reg |=  (tx_credit << 12) | (0x1FF << 2) | BIT_ULL(1);
		}
		pko_reg_write(pko, PKO_PF_L3_L2_SQX_CHANNEL(queue_base), reg);
		dev_dbg(&pko->pdev->dev, "PKO: L3_L2_SQ[%d]_CHANNEL ::0x%llx\n",
			queue_base,
			pko_reg_read(pko,
				     PKO_PF_L3_L2_SQX_CHANNEL(queue_base)));

		reg = (queue_base) | (1Ull << 15) | (mac_num << 9);
		pko_reg_write(pko, PKO_PF_LUTX(channel), reg);
		dev_dbg(&pko->pdev->dev, "PKO: PF_LTUX[%d] ::0x%llx\n",
			channel, pko_reg_read(pko, PKO_PF_LUTX(channel)));
	}

	return queue_base;
}

static void pko_sq_teardown(struct pkopf *pko, int vf, int level, u32 channel,
			    int mac_num)
{
	int channel_level;
	int queue_base;

	channel_level = pko_reg_read(pko, PKO_PF_CHANNEL_LEVEL);
	channel_level += 2;

	queue_base = (vf * 8);

	pko_lX_set_schedule(pko, level, queue_base, 0x0);
	pko_lX_set_shape(pko, level, queue_base, 0x0);
	pko_lX_set_topology(pko, level, queue_base, 0x0);

	if (level == channel_level) {
		pko_reg_write(pko, PKO_PF_L3_L2_SQX_CHANNEL(queue_base),
			      (u64)PKO_CHAN_NULL << 32);
		pko_reg_write(pko, PKO_PF_LUTX(channel), 0x0);
	}
}

static void pko_dq_init(struct pkopf *pko, int vf, int mac_num)
{
	int queue_base, i;
	u64 reg;

	queue_base = vf * 8;
	reg = queue_base << 16;
	for (i = 0; i < 8; i++) {
		/* for SDP MAC, 8 dqs are mapped to 8 sq_l3s */
		if (mac_num == SDP_MAC_NUM)  /* SDP specific configuration */
			reg = (queue_base + i) << 16;

		pko_reg_write(pko, PKO_PF_DQX_TOPOLOGY(queue_base + i), reg);
		dev_dbg(&pko->pdev->dev, "PKO: DQ[%d]_TOPOLOGY ::0x%llx\n",
			(queue_base + i),
			pko_reg_read(pko, PKO_PF_DQX_TOPOLOGY(queue_base + i)));

		/* PRIO = 0, RR_QUANTUM = max */
		pko_reg_write(pko, PKO_PF_DQX_SCHEDULE(queue_base + i),
			      0xffffff);
		dev_dbg(&pko->pdev->dev, "PKO: DQ[%d]_SCHEDULE ::0x%llx\n",
			(queue_base + i),
			pko_reg_read(pko, PKO_PF_DQX_SCHEDULE(queue_base + i)));

		if (mac_num != SDP_MAC_NUM) {
			pko_reg_write(pko, PKO_PF_DQX_SHAPE(queue_base + i),
				      0x0);
			pko_reg_write(pko,
				      PKO_PF_PDM_DQX_MINPAD(queue_base + i),
				      0x1);
		}
	}
}

static void pko_dq_teardown(struct pkopf *pko, int vf)
{
	int queue_base, i;

	queue_base = vf * 8;
	for (i = 0; i < 8; i++) {
		pko_reg_write(pko, PKO_PF_DQX_TOPOLOGY(queue_base + i), 0x0);
		pko_reg_write(pko, PKO_PF_DQX_SCHEDULE(queue_base + i), 0x0);
		pko_reg_write(pko, PKO_PF_DQX_SHAPE(queue_base + i), 0x0);
		pko_reg_write(pko, PKO_PF_PDM_DQX_MINPAD(queue_base + i), 0x0);
	}
}

static int pko_pstree_setup(struct pkopf *pko, int vf, u32 max_frame,
			    int mac_num, int mac_mode, int channel)
{
	int lvl;
	int err, i, dq_cnt;

	err = pko_mac_init(pko, mac_num, mac_mode);
	if (err)
		return -ENODEV;

	dev_dbg(&pko->pdev->dev, "%s: vf: %d, mac_num: %d, max_frame: %d\n",
		__func__, vf, mac_num, max_frame);
	pko_pq_init(pko, vf, mac_num, max_frame);

	err = mac_num;
	if (mac_num != SDP_MAC_NUM) { /* BGX MAC specific configuration */
		for (lvl = 2; lvl <= pko->max_levels; lvl++)
			err = pko_sq_init(pko, vf, lvl, channel, mac_num,
					  max_frame, err);
	} else { /* SDP MAC specific configuration */
		/* TODO pko_vf:1 is for SDP, Map sq_l2, sq_l3 for all the DQs.*/
		dq_cnt = 8;
		lvl = 2;

		for (i = 0; i < dq_cnt; i++) {
			err = mac_num;
			dev_dbg(&pko->pdev->dev, "%s: vf: %d, lvl: %d, channel: %d, i : %d, max_frame: %d, err: %d\n",
				__func__, vf, lvl, channel, i, max_frame, err);
			pko_sq_init(pko, vf, lvl, channel + i, mac_num,
				    max_frame, err);
		}

		lvl = 3;

		for (i = 0; i < dq_cnt; i++) {
			err = (vf * 8) + i;
			dev_dbg(&pko->pdev->dev, "%s: vf: %d, lvl: %d, channel: %d, i : %d, max_frame: %d, err: %d\n",
				__func__, vf, lvl, channel, i, max_frame, err);
			pko_sq_init(pko, vf, lvl, channel + i, mac_num,
				    max_frame, err);
		}
	}
	pko_dq_init(pko, vf, mac_num);

	return 0;
}

static void pko_pstree_teardown(struct pkopf *pko, int vf, int mac_num,
				int channel)
{
	int lvl;

	pko_dq_teardown(pko, vf);
	for (lvl = pko->max_levels; lvl > 1; lvl--)
		pko_sq_teardown(pko, vf, lvl, channel, mac_num);

	pko_pq_teardown(pko, mac_num);
	pko_mac_teardown(pko, mac_num);
}

static int pko_enable(struct pkopf *pko)
{
	u64 reg;
	int retry = 0;

	pko_reg_write(pko, PKO_PF_ENABLE, 0x1);

	while (true) {
		reg = pko_reg_read(pko, PKO_PF_STATUS);
		if ((reg & 0x1FF) == 0x1FF)
			break;
		usleep_range(10000, 20000);
		retry++;
		if (retry > 10)
			return -ENODEV;
	}

	return 0;
}

static int pko_disable(struct pkopf *pko)
{
	u64 reg;
	int retry = 0;

	pko_reg_write(pko, PKO_PF_ENABLE, 0x0);

	while (true) {
		reg = pko_reg_read(pko, PKO_PF_STATUS);
		if ((reg & 0x100) == 0)
			break;
		usleep_range(10000, 20000);
		retry++;
		if (retry > 10)
			return -ENODEV;
	}

	return 0;
}

static int setup_dpfi(struct pkopf *pko)
{
	int err, retry = 0;
	u64 reg;

	err = fpapf->create_domain(pko->id, FPA_PKO_DPFI_GMID, 1, NULL);
	if (!err) {
		dev_err(&pko->pdev->dev, "failed to create PKO_DPFI_DOMAIN\n");
		symbol_put(fpapf_com);
		return -ENODEV;
	}

	fpa = fpavf->get(FPA_PKO_DPFI_GMID, 0, &pko_master_com, pko);
	if (!fpa) {
		dev_err(&pko->pdev->dev, "failed to get fpavf\n");
		symbol_put(fpapf_com);
		symbol_put(fpavf_com);
		return -ENODEV;
	}
	dev_notice(&pko->pdev->dev, "Setup PKO_DPFI_DOMAIN: pdm_buffers %d, pdm_buf_size %d\n",
		   PKO_BUFFERS, pko->pdm_buf_size);
	err = fpavf->setup(fpa, PKO_BUFFERS, pko->pdm_buf_size,
			   &pko->pdev->dev);
	if (err) {
		dev_err(&pko->pdev->dev, "failed to setup fpavf\n");
		symbol_put(fpapf_com);
		symbol_put(fpavf_com);
		return -ENODEV;
	}

	pko_reg_write(pko, PKO_PF_DPFI_FPA_AURA, 0);
	pko_reg_write(pko, PKO_PF_DPFI_GMCTL, FPA_PKO_DPFI_GMID);
	pko_reg_write(pko, PKO_PF_DPFI_FLUSH, 0);
	pko_reg_write(pko, PKO_PF_DPFI_ENA, 0x1);

	while (true) {
		reg = pko_reg_read(pko, PKO_PF_DPFI_STATUS);
		if (!(reg & 0x2))
			break;
		usleep_range(10000, 20000);
		retry++;
		if (retry > 10)
			return -ENODEV;
	}
	return 0;
}

static int teardown_dpfi(struct pkopf *pko)
{
	int retry = 0;
	u64 reg;

	pko_reg_write(pko, PKO_PF_DPFI_FLUSH, 1);

	while (true) {
		reg = pko_reg_read(pko, PKO_PF_DPFI_STATUS);
		if ((reg & 0x1) == 0x1)
			break;
		usleep_range(10000, 20000);
		retry++;
		if (retry > 10) {
			dev_err(&pko->pdev->dev, "Failed to flush DPFI.\n");
			return -ENODEV;
		}
	}
	if ((reg >> 32) > 0)
		dev_err(&pko->pdev->dev,
			"DPFI cache not empty after flush, left %lld\n",
			reg >> 32);
	pko_reg_write(pko, PKO_PF_DPFI_GMCTL, 0);
	pko_reg_write(pko, PKO_PF_DPFI_ENA, 0);

	dev_notice(&pko->pdev->dev, "Destroy PKO_DPFI_DOMAIN\n");
	fpavf->teardown(fpa);
	fpavf->put(fpa);
	fpapf->destroy_domain(pko->id, FPA_PKO_DPFI_GMID, NULL);

	return 0;
}

static int pko_init(struct pkopf *pko)
{
	u64 reg;
	int retry = 0;
	int n;
	int i;

	reg = pko_reg_read(pko, PKO_PF_CONST);

	pko->max_levels = PKO_CONST_GET_LEVELS(reg);
	pko->max_ptgfs = PKO_CONST_GET_PTGFS(reg);
	pko->max_formats = PKO_CONST_GET_FORMATS(reg);
	pko->pdm_buf_size = PKO_CONST_GET_PDM_BUF_SIZE(reg);
	pko->dqs_per_vf = PKO_CONST_GET_DQS_PER_VM(reg);

	while (true) {
		reg = pko_reg_read(pko, PKO_PF_STATUS);
		if ((reg & 0xFF) == 0xFB)
			break;
		usleep_range(10000, 20000);
		retry++;
		if (retry > 10)
			return -ENODEV;
	}

	reg = PKO_PDM_CFG_SET_PAD_MINLEN(PKO_PAD_MINLEN) |
		PKO_PDM_CFG_SET_DQ_FC_SKID(PKO_FC_SKID) |
		PKO_PDM_CFG_SET_EN(1);
	pko_reg_write(pko, PKO_PF_PDM_CFG, reg);

	pko_reg_write(pko, PKO_PF_SHAPER_CFG, 0x1);
	/*use L3 SQs */
	pko_reg_write(pko, PKO_PF_CHANNEL_LEVEL, 0x1);

	n = pko_reg_read(pko, PKO_PF_L1_CONST);
	for (i = 0; i < n; i++)
		pko_reg_write(pko, PKO_PF_L1_SQX_TOPOLOGY(i), 19 << 16);

	for (i = 0; i < pko->max_formats; i++)
		pko_reg_write(pko, PKO_PF_FORMATX_CTL(i), 0x0);
	pko_reg_write(pko, PKO_PF_FORMATX_CTL(1), 0x101);

	reg = pko_reg_read(pko, PKO_PF_DQ_CONST);
	n = (reg & ((1ull << 16) - 1)) / pko->dqs_per_vf;
#ifdef __BIG_ENDIAN
	reg = 1ull << 24; /*BE*/
#else
	reg = 0;
#endif
	for (i = 0; i < n; i++)
		pko_reg_write(pko, PKO_PF_VFX_GMCTL(i), reg);
	return 0;
}

static int pko_sriov_configure(struct pci_dev *pdev, int numvfs)
{
	struct pkopf *pko = pci_get_drvdata(pdev);
	int ret = -EBUSY;
	int disable = 0;

	if (pko->vfs_in_use != 0)
		return ret;

	ret = 0;
	if (pko->flags & PKO_SRIOV_ENABLED)
		disable = 1;

	if (disable) {
		pci_disable_sriov(pdev);
		pko->flags &= ~PKO_SRIOV_ENABLED;
		pko->total_vfs = 0;
	}

	if (numvfs > 0) {
		ret = pci_enable_sriov(pdev, numvfs);
		if (ret == 0) {
			pko->flags |= PKO_SRIOV_ENABLED;
			pko->total_vfs = numvfs;
			ret = numvfs;
		}
	}

	dev_notice(&pko->pdev->dev, "VFs enabled: %d\n", ret);
	return ret;
}

static int pko_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	struct device *dev = &pdev->dev;
	struct pkopf *pko;
	int err = -ENOMEM;

	pko = devm_kzalloc(dev, sizeof(*pko), GFP_KERNEL);
	if (!pko)
		return err;

	pci_set_drvdata(pdev, pko);
	pko->pdev = pdev;

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
	pko->reg_base = pcim_iomap(pdev, PCI_PKO_PF_CFG_BAR, 0);
	if (!pko->reg_base) {
		dev_err(dev, "Can't map CFG space\n");
		err = -ENOMEM;
		return err;
	}

	/*set PKO ID */
	pko->id = atomic_add_return(1, &pko_count);
	pko->id -= 1;

	err = pko_init(pko);
	if (err) {
		dev_err(dev, "Failed to init PKO\n");
		atomic_sub_return(1, &pko_count);
		return err;
	}

	err = setup_dpfi(pko);
	if (err) {
		dev_err(dev, "Failed to init DPFI\n");
		atomic_sub_return(1, &pko_count);
		return err;
	}

	err = pko_irq_init(pko);
	if (err) {
		atomic_sub_return(1, &pko_count);
		dev_err(dev, "failed init irqs\n");
		err = -EINVAL;
		return err;
	}

	err = pko_enable(pko);
	if (err) {
		atomic_sub_return(1, &pko_count);
		dev_err(dev, "failed to enable pko\n");
		err = -EINVAL;
		return err;
	}

	INIT_LIST_HEAD(&pko->list);
	mutex_lock(&octeontx_pko_devices_lock);
	list_add(&pko->list, &octeontx_pko_devices);
	mutex_unlock(&octeontx_pko_devices_lock);
	return 0;
}

static void pko_remove(struct pci_dev *pdev)
{
	struct pkopf *pko = pci_get_drvdata(pdev);

	if (!pko)
		return;

	pko_disable(pko);
	teardown_dpfi(pko);
	pko_irq_free(pko);
	pko_sriov_configure(pdev, 0);
}

/* devices supported */
static const struct pci_device_id pko_id_table[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_CAVIUM, PCI_DEVICE_ID_OCTEONTX_PKO_PF) },
	{ 0, }  /* end of table */
};

static struct pci_driver pko_driver = {
	.name = DRV_NAME,
	.id_table = pko_id_table,
	.probe = pko_probe,
	.remove = pko_remove,
	.sriov_configure = pko_sriov_configure,
};

MODULE_AUTHOR("Tirumalesh Chalamarla");
MODULE_DESCRIPTION("Cavium OCTEONTX PKO Physical Function Driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRV_VERSION);
MODULE_DEVICE_TABLE(pci, pko_id_table);

static int __init pko_init_module(void)
{
	pr_info("%s, ver %s\n", DRV_NAME, DRV_VERSION);
	fpapf = try_then_request_module(symbol_get(fpapf_com), "fpapf");
	if (!fpapf)
		return -ENODEV;

	fpavf = try_then_request_module(symbol_get(fpavf_com), "fpavf");
	if (!fpavf) {
		symbol_put(fpapf_com);
		return -ENODEV;
	}

	return pci_register_driver(&pko_driver);
}

static void __exit pko_cleanup_module(void)
{
	pci_unregister_driver(&pko_driver);
	symbol_put(fpapf_com);
	symbol_put(fpavf_com);
}

module_init(pko_init_module);
module_exit(pko_cleanup_module);
