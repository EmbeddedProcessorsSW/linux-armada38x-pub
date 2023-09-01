// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018 Cavium, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License
 * as published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/random.h>

#include "sli.h"
#include "rst.h"

#define SLI_DRV_NAME    "octeontx-sli"
#define SLI_DRV_VERSION "0.1"

#define PKI_CHAN_E_SDP_CHX(a) (0x400 + (a))

#define PKO_LMAC_E_SDP   2
#define SDP_INVALID_ID  (-1)

static struct rst_com_s *rst;

/* Global lists of SDP devices and ports */
static DEFINE_MUTEX(octeontx_sli_devices_lock);
static DEFINE_MUTEX(octeontx_sdp_lock);
static LIST_HEAD(octeontx_sli_devices);
static LIST_HEAD(octeontx_sdp_ports);

static int poll_for_ep_mode_miss_count;
struct delayed_work work;
struct workqueue_struct *ep_mode_handshake;

static u64 get_coproc_clk(u8 node)
{
	u64 freq_in_mhz, sclk_freq;

	/* Get SCLK */
	sclk_freq = rst->get_sclk_freq(node);

	freq_in_mhz = sclk_freq / (1000 * 1000);

	return freq_in_mhz;
}

/* Register read/write APIs */
static void sli_reg_write(struct slipf *sli, u64 offset, u64 val)
{
	writeq_relaxed(val, sli->reg_base + offset);
}

static u64 sli_reg_read(struct slipf *sli, u64 offset)
{
	return readq_relaxed(sli->reg_base + offset);
}

static struct slipf *get_sli_dev(int node, int sli_idx)
{
	struct slipf *sli_dev = NULL;

	mutex_lock(&octeontx_sli_devices_lock);
	list_for_each_entry(sli_dev, &octeontx_sli_devices, list) {
		if (sli_dev && sli_dev->node == node &&
		    sli_dev->sli_idx == sli_idx)
			break;
	}
	mutex_unlock(&octeontx_sli_devices_lock);
	return sli_dev;
}

static struct octtx_sdp_port *get_sdp_port(int domain_id,
					   int port_idx)
{
	struct octtx_sdp_port *sdp_port = NULL;

	mutex_lock(&octeontx_sdp_lock);
	list_for_each_entry(sdp_port, &octeontx_sdp_ports, list) {
		if (sdp_port && (sdp_port->domain_id == domain_id) &&
		    (sdp_port->dom_port_idx == port_idx))
			break;
	}
	mutex_unlock(&octeontx_sdp_lock);
	return sdp_port;
}

/* SLI Interface functions. */
static int sli_get_num_ports(int node)
{
	struct octtx_sdp_port *sdp_port = NULL;
	int count = 0;

	mutex_lock(&octeontx_sdp_lock);
	list_for_each_entry(sdp_port, &octeontx_sdp_ports, list) {
		if (sdp_port && (sdp_port->node == node))
			count++;
	}
	mutex_unlock(&octeontx_sdp_lock);
	return count;
}

static bool sli_get_link_status(int node __maybe_unused,
				int sdp __maybe_unused,
				int lmac __maybe_unused)
{
	return true;
}

/* Main MBOX message processing function.  */
static int sdp_port_open(struct octtx_sdp_port *port);
static int sdp_port_close(struct octtx_sdp_port *port);
static int sdp_port_start(struct octtx_sdp_port *port);
static int sdp_port_stop(struct octtx_sdp_port *port);
static int sdp_port_config(struct octtx_sdp_port *port,
			   struct mbox_sdp_port_conf *conf);
static int sdp_port_status(struct octtx_sdp_port *port,
			   struct mbox_sdp_port_status *stat);
static int sdp_port_stats_get(struct octtx_sdp_port *port,
			      struct mbox_sdp_port_stats *stat);
static int sdp_port_stats_clr(struct octtx_sdp_port *port);
static int sdp_port_link_status(struct octtx_sdp_port *port, u8 *up);
static void sdp_reg_read(struct mbox_sdp_reg *reg);
static void sdp_reg_write(struct mbox_sdp_reg *reg);

static int sli_receive_message(u32 id, u16 domain_id, struct mbox_hdr *hdr,
			       union mbox_data *req,
			       union mbox_data *resp, void *mdata)
{
	struct octtx_sdp_port *sdp_port = NULL;

	if (!mdata)
		return -ENOMEM;

	sdp_port = get_sdp_port(domain_id, hdr->vfid);
	if (!sdp_port) {
		hdr->res_code = MBOX_RET_INVALID;
		return -ENODEV;
	}

	switch (hdr->msg) {
	case MBOX_SDP_PORT_OPEN:
		sdp_port_open(sdp_port);
		sdp_port_config(sdp_port, mdata);
		resp->data = sizeof(struct mbox_sdp_port_conf);
		break;

	case MBOX_SDP_PORT_CLOSE:
		sdp_port_close(sdp_port);
		resp->data = 0;
		break;

	case MBOX_SDP_PORT_START:
		sdp_port_start(sdp_port);
		resp->data = 0;
		break;

	case MBOX_SDP_PORT_STOP:
		sdp_port_stop(sdp_port);
		resp->data = 0;
		break;

	case MBOX_SDP_PORT_GET_CONFIG:
		sdp_port_config(sdp_port, mdata);
		resp->data = sizeof(struct mbox_sdp_port_conf);
		break;

	case MBOX_SDP_PORT_GET_STATUS:
		sdp_port_status(sdp_port, mdata);
		resp->data = sizeof(struct mbox_sdp_port_status);
		break;

	case MBOX_SDP_PORT_GET_STATS:
		sdp_port_stats_get(sdp_port, mdata);
		resp->data = sizeof(struct mbox_sdp_port_stats);
		break;

	case MBOX_SDP_PORT_CLR_STATS:
		sdp_port_stats_clr(sdp_port);
		resp->data = 0;
		break;

	case MBOX_SDP_PORT_GET_LINK_STATUS:
		sdp_port_link_status(sdp_port, mdata);
		resp->data = sizeof(u8);
		break;

	case MBOX_SDP_REG_READ:
		sdp_reg_read(mdata);
		resp->data = sizeof(struct mbox_sdp_reg);
		break;

	case MBOX_SDP_REG_WRITE:
		sdp_reg_write(mdata);
		resp->data = 0;
		break;

	default:
		hdr->res_code = MBOX_RET_INVALID;
		return -EINVAL;
	}

	hdr->res_code = MBOX_RET_SUCCESS;
	return 0;
}

/* MBOX message processing support functions. */
int sdp_port_open(struct octtx_sdp_port *port __maybe_unused)
{
	return 0;
}

int sdp_port_close(struct octtx_sdp_port *port __maybe_unused)
{
	return 0;
}

int sdp_port_start(struct octtx_sdp_port *port)
{
	int pf_id = port->glb_port_idx >> 8;
	int vf_id = port->glb_port_idx & 0xff;
	struct slipf *sli = get_sli_dev(0, 0);
	u64 reg_val = 0ULL;

	reg_val = sli_reg_read(sli, SLI_EPFX_SCRATCH(pf_id));
	if (vf_id)
		reg_val |= 1ULL << vf_id;
	else
		reg_val |= 1ULL; /* Bit 0 is to represent PF*/

	sli_reg_write(sli, SLI_EPFX_SCRATCH(pf_id), reg_val);

	return 0;
}

int sdp_port_stop(struct octtx_sdp_port *port __maybe_unused)
{
	return 0;
}

int sdp_port_config(struct octtx_sdp_port *sdp_port,
		    struct mbox_sdp_port_conf *conf)
{
	struct slipf *sli = get_sli_dev(0, 0);
	int i = 0;
	u64 bp_en_val = 0x0ULL;

	conf->node  = sdp_port->node;
	conf->sdp   = sdp_port->sdp;
	conf->lmac  = sdp_port->lmac;
	conf->base_chan = sdp_port->base_chan;
	conf->num_chans = sdp_port->num_chans;

	sli_reg_write(sli, SDP_OUT_WMARK, 0x100);

	bp_en_val = sli_reg_read(sli, SDP_OUT_BP_ENx_W1S(0));
	for (i = sdp_port->base_chan - SDP_CHANNEL_START;
	     i < sdp_port->num_chans; i++) {
		bp_en_val |= 0x1ULL << i;
	}
	sli_reg_write(sli, SDP_OUT_BP_ENx_W1S(0), bp_en_val);

	return 0;
}

int sdp_port_status(struct octtx_sdp_port *port __maybe_unused,
		    struct mbox_sdp_port_status *stat __maybe_unused)
{
	return 0;
}

int sdp_port_stats_get(struct octtx_sdp_port *port __maybe_unused,
		       struct mbox_sdp_port_stats *stats __maybe_unused)
{
	return 0;
}

int sdp_port_stats_clr(struct octtx_sdp_port *port __maybe_unused)
{
	return 0;
}

int sdp_port_link_status(struct octtx_sdp_port *port __maybe_unused,
			 u8 *up __maybe_unused)
{
	return 0;
}

void sdp_reg_read(struct mbox_sdp_reg *reg)
{
	struct slipf *sli = get_sli_dev(0, 0); /* Hard coding for now */

	reg->val = sli_reg_read(sli, reg->addr);
}

void sdp_reg_write(struct mbox_sdp_reg *reg)
{
	struct slipf *sli = get_sli_dev(0, 0); /* Hard coding for now */

	sli_reg_write(sli, reg->addr, reg->val);
}

/* Domain create function.  */
static int sli_create_domain(u32 id, u16 domain_id,
			     struct octtx_sdp_port *port_tbl, int ports,
			     struct octeontx_master_com_t *com, void *domain,
			     struct kobject *kobj __maybe_unused)
{
	struct octtx_sdp_port *sdp_port = NULL, *gport = NULL;
	struct slipf *sli  = NULL;
	struct slipf *curr = NULL;
	int port_idx;

	/* TODO: Add sysfs entries */

	mutex_lock(&octeontx_sli_devices_lock);
	list_for_each_entry(curr, &octeontx_sli_devices, list) {
		if (curr->id == id) {
			sli = curr;
			break;
		}
	}
	mutex_unlock(&octeontx_sli_devices_lock);

	if (!sli)
		return -ENODEV;

	/* For each domain port, find requested entry in the list of
	 * global ports and sync up those two port structures.
	 */
	mutex_lock(&octeontx_sdp_lock);
	for (port_idx = 0; port_idx < ports; port_idx++) {
		sdp_port = &port_tbl[port_idx];

		list_for_each_entry(gport, &octeontx_sdp_ports, list) {
			if (gport->node != id ||
			    gport->glb_port_idx != sdp_port->glb_port_idx)
				continue;
			/* Check for conflicts with other domains. */
			if (gport->domain_id != SDP_INVALID_ID) {
				mutex_unlock(&octeontx_sdp_lock);
				return -EINVAL;
			}

			/* TODO: Add sysfs entries */
			/* Domain port: */
			sdp_port->node = gport->node;
			sdp_port->lmac = gport->lmac;
			sdp_port->lmac_type = gport->lmac_type;
			sdp_port->base_chan = gport->base_chan;
			sdp_port->num_chans = gport->num_chans;

			/* Global port: */
			gport->domain_id = domain_id;
			gport->dom_port_idx = port_idx;
		}
	}
	mutex_unlock(&octeontx_sdp_lock);
	dev_dbg(&sli->pdev->dev, "sli domain creation is successful\n");

	return 0;
}

/* Domain destroy function.  */
static int sli_destroy_domain(u32 id, u16 domain_id,
			      struct kobject *kobj __maybe_unused)
{
	struct octtx_sdp_port *sdp_port = NULL;

	/* TODO: Add sysfs entries */

	mutex_lock(&octeontx_sdp_lock);
	list_for_each_entry(sdp_port, &octeontx_sdp_ports, list) {
		if (sdp_port && (sdp_port->node == id) &&
		    (sdp_port->domain_id == domain_id)) {
			sdp_port->domain_id = SDP_INVALID_ID;
			sdp_port->dom_port_idx = SDP_INVALID_ID;
		}
	}
	mutex_unlock(&octeontx_sdp_lock);

	return 0;
}

/* Domain reset function. */
static int sli_reset_domain(u32 id, u16 domain_id)
{
	struct octtx_sdp_port *sdp_port = NULL;

	mutex_lock(&octeontx_sdp_lock);
	list_for_each_entry(sdp_port, &octeontx_sdp_ports, list) {
		if (sdp_port && (sdp_port->node == id) &&
		    (sdp_port->domain_id == domain_id))
			sdp_port_stop(sdp_port);
	}
	mutex_unlock(&octeontx_sdp_lock);

	return 0;
}

/* Set pkind for a given port. */
static int sli_set_pkind(u32 id, u16 domain_id, int port, int pkind)
{
	struct octtx_sdp_port *gport = NULL;
	u64 reg_val = 0x0ULL;
	struct slipf *sli = get_sli_dev(0, 0);

	gport = get_sdp_port(domain_id, port);
	if (!gport)
		return -ENODEV;

	sli_reg_write(sli, SDP_PKIND_VALID, 0x0ULL);

	reg_val = sli_reg_read(sli, SDP_GBL_CONTROL);

	reg_val = 0x1ULL << 3;/* SET PKIPFVAL */
	set_sdp_field(&reg_val, SDP_GBL_CONTROL_BPKIND_MASK,
		      SDP_GBL_CONTROL_BPKIND_SHIFT, pkind);
	sli_reg_write(sli, SDP_GBL_CONTROL, reg_val);

	gport->pkind = pkind;

	return 0;
}

static void poll_for_ep_mode(struct work_struct *wrk)
{
	int i = 0, j = 0;
	struct slipf *sli = NULL;
	u64 scratch_addr = 0ULL, scratch_val = 0ULL;
	struct sli_epf *epf = NULL;
	struct octtx_sdp_port *sdp_port = NULL;
	int node = 0;
	struct list_head *pos, *tmppos;

	sli = get_sli_dev(0, 0);

	for (i = 0; i < SLI_LMAC_MAX_PFS; i++) {
		epf = &sli->epf[i];
		if (epf->hs_done)
			continue;

		scratch_addr = SLI_EPFX_SCRATCH(i);
		scratch_val = sli_reg_read(sli, scratch_addr);
		if (scratch_val != SDP_HOST_LOADED) {
			poll_for_ep_mode_miss_count++;
		} else {
			scratch_val = SDP_GET_HOST_INFO;
			sli_reg_write(sli, scratch_addr, scratch_val);

			/* wait for ep_mode to write the information */
			while (sli_reg_read(sli, scratch_addr) ==
			       SDP_GET_HOST_INFO)
				;

			scratch_val = sli_reg_read(sli, scratch_addr);
			epf->rpvf = scratch_val & 0xff;
			epf->vf_srn = (scratch_val >> 8) & 0xff;
			epf->num_vfs = (scratch_val >> 16) & 0xff;
			epf->rppf = (scratch_val >> 24) & 0xff;
			epf->pf_srn = (scratch_val >> 32) & 0xff;
			epf->app_mode = (scratch_val >> 40) & 0xff;

			scratch_val = (SDP_HOST_INFO_RECEIVED << 16) |
				      (sli->ticks_per_us & 0xffff);
			sli_reg_write(sli, scratch_addr, scratch_val);

			/* wait for ep_mode to write the completion */
			while ((sli_reg_read(sli, scratch_addr) >> 16) ==
					     SDP_HOST_INFO_RECEIVED)
				;
			scratch_val = sli_reg_read(sli, scratch_addr);
			if (scratch_val == SDP_HANDSHAKE_COMPLETED)
				epf->hs_done = 1;

			scratch_val = 0x0ULL;
			sli_reg_write(sli, scratch_addr, scratch_val);

			/* Populate the sdp port for PF */
			sdp_port = kzalloc(sizeof(*sdp_port), GFP_KERNEL);
			if (!sdp_port)
				goto err_free_sdp_ports;

			sdp_port->glb_port_idx = i << 8;
			sdp_port->sdp = i << 8;
			sdp_port->node = node;
			sdp_port->lmac = PKO_LMAC_E_SDP;
			sdp_port->base_chan = PKI_CHAN_E_SDP_CHX(epf->pf_srn);
			sdp_port->num_chans = epf->rppf;
			sdp_port->domain_id = SDP_INVALID_ID;
			sdp_port->dom_port_idx = SDP_INVALID_ID;

			INIT_LIST_HEAD(&sdp_port->list);
			mutex_lock(&octeontx_sdp_lock);
			list_add(&sdp_port->list, &octeontx_sdp_ports);
			mutex_unlock(&octeontx_sdp_lock);
			sdp_port = NULL;

			/* Populate the sdp ports for VFs of PF */
			for (j = 1; j < epf->num_vfs; j++) {
				sdp_port = kzalloc(sizeof(*sdp_port),
						   GFP_KERNEL);
				if (!sdp_port)
					goto err_free_sdp_ports;

				sdp_port->glb_port_idx = (i << 8) | j;
				sdp_port->sdp = (i << 8) | j;
				sdp_port->node = node;
				sdp_port->lmac = PKO_LMAC_E_SDP;
				sdp_port->base_chan =
					PKI_CHAN_E_SDP_CHX(epf->vf_srn +
							   (j - 1) * epf->rpvf);
				sdp_port->num_chans = epf->rpvf;
				sdp_port->domain_id = SDP_INVALID_ID;
				sdp_port->dom_port_idx = SDP_INVALID_ID;

				INIT_LIST_HEAD(&sdp_port->list);
				mutex_lock(&octeontx_sdp_lock);
				list_add(&sdp_port->list, &octeontx_sdp_ports);
				mutex_unlock(&octeontx_sdp_lock);

				sdp_port = NULL;
			}
		}
	}

	if (poll_for_ep_mode_miss_count > 10)
		return;

	sli = get_sli_dev(0, 0);
	for (i = 0; i < SLI_LMAC_MAX_PFS; i++) {
		epf = &sli->epf[i];
		if (!epf->hs_done)
			break;
	}

	if (i != SLI_LMAC_MAX_PFS)
		queue_delayed_work(ep_mode_handshake, &work, HZ * 1);

	return;
err_free_sdp_ports:
	dev_err(&sli->pdev->dev, "octeontx-sli: sdp port alloc failed!\n");
	mutex_lock(&octeontx_sdp_lock);
	list_for_each_safe(pos, tmppos, &octeontx_sdp_ports) {
		sdp_port = list_entry(pos, struct octtx_sdp_port, list);
		if (sdp_port) {
			list_del(pos);
			kfree(sdp_port);
		}
	}
	mutex_unlock(&octeontx_sdp_lock);
}

static int sli_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	struct device *dev = &pdev->dev;
	struct slipf *sli = NULL;

	int node = 0;
	int err = 0;

	sli = devm_kzalloc(dev, sizeof(*sli), GFP_KERNEL);
	if (!sli)
		return -ENOMEM;

	pci_set_drvdata(pdev, sli);
	sli->pdev = pdev;

	err = pcim_enable_device(pdev);
	if (err) {
		dev_err(dev, "Failed to enable PCI device\n");
		goto err_free_device;
	}

	err = pci_request_regions(pdev, SLI_DRV_NAME);
	if (err) {
		dev_err(dev, "PCI request regions failed\n");
		goto err_disable_device;
	}

	/* Map BAR space CFG registers */
	sli->reg_base = pcim_iomap(pdev, PCI_SLI_PF_CFG_BAR, 0);
	if (!sli->reg_base) {
		dev_err(dev, "Can't map CFG space\n");
		err = -ENOMEM;
		goto err_release_regions;
	}

	sli->node = node;
	sli->sli_idx = 0;
	INIT_LIST_HEAD(&sli->list);

	sli->ticks_per_us = get_coproc_clk(0);

	sli->epf[0].hs_done = 0;
	ep_mode_handshake = alloc_workqueue("ksli_ep_modefw_hs",
					    WQ_MEM_RECLAIM, 0);
	mutex_lock(&octeontx_sli_devices_lock);
	list_add(&sli->list, &octeontx_sli_devices);
	mutex_unlock(&octeontx_sli_devices_lock);
	INIT_DELAYED_WORK(&work, poll_for_ep_mode);
	queue_delayed_work(ep_mode_handshake, &work, 0);

	return 0;

err_release_regions:
	if (sli->reg_base)
		iounmap(sli->reg_base);
	pci_release_regions(pdev);

err_disable_device:
	pci_disable_device(pdev);

err_free_device:
	pci_set_drvdata(pdev, NULL);
	devm_kfree(dev, sli);

	return err;
}

static void sli_remove(struct pci_dev *pdev)
{
	struct device *dev = &pdev->dev;
	struct slipf *sli = pci_get_drvdata(pdev);
	struct slipf *curr = NULL;
	struct octtx_sdp_port *sdp_port = NULL;
	struct list_head *pos, *tmppos;

	if (!sli)
		return;

	/* First of all, this SLI device's SDP port list and its memory has
	 * to be freed.
	 */
	mutex_lock(&octeontx_sdp_lock);
	list_for_each_safe(pos, tmppos, &octeontx_sdp_ports) {
		sdp_port = list_entry(pos, struct octtx_sdp_port, list);
		if (sdp_port) {
			list_del(pos);
			kfree(sdp_port);
		}
	}
	mutex_unlock(&octeontx_sdp_lock);

	mutex_lock(&octeontx_sli_devices_lock);
	list_for_each_entry(curr, &octeontx_sli_devices, list) {
		if (curr == sli) {
			list_del(&sli->list);
			break;
		}
	}
	mutex_unlock(&octeontx_sli_devices_lock);

	cancel_delayed_work_sync(&work);
	flush_workqueue(ep_mode_handshake);
	destroy_workqueue(ep_mode_handshake);

	pci_release_regions(pdev);
	pci_disable_device(pdev);
	pci_set_drvdata(pdev, NULL);
	devm_kfree(dev, sli);
}

/* Interface with the main OCTEONTX driver. */
struct slipf_com_s slipf_com  = {
	.create_domain   = sli_create_domain,
	.destroy_domain     = sli_destroy_domain,
	.reset_domain    = sli_reset_domain,
	.receive_message = sli_receive_message,
	.get_num_ports   = sli_get_num_ports,
	.get_link_status = sli_get_link_status,
	.set_pkind       = sli_set_pkind
};
EXPORT_SYMBOL(slipf_com);

/* devices supported */
static const struct pci_device_id sli_id_table[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_CAVIUM, PCI_DEVICE_ID_OCTEONTX_SLI_PF) },
	{ 0, } /* end of table */
};

static struct pci_driver sli_driver = {
	.name     = SLI_DRV_NAME,
	.id_table = sli_id_table,
	.probe    = sli_probe,
	.remove   = sli_remove,
};

MODULE_AUTHOR("Cavium");
MODULE_DESCRIPTION("Cavium OCTEONTX SLI Physical Function Driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(SLI_DRV_VERSION);
MODULE_DEVICE_TABLE(pci, sli_id_table);

static int __init sli_init_module(void)
{
	pr_info("%s, ver %s\n", SLI_DRV_NAME, SLI_DRV_VERSION);
	rst = try_then_request_module(symbol_get(rst_com), "rst");
	if (!rst)
		return -ENODEV;

	return pci_register_driver(&sli_driver);
}

static void __exit sli_cleanup_module(void)
{
	pci_unregister_driver(&sli_driver);
	symbol_put(rst_com);
}

module_init(sli_init_module);
module_exit(sli_cleanup_module);
