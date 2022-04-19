// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2016 Cavium, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License
 * as published by the Free Software Foundation.
 */

#include <linux/bitfield.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/random.h>

#include "../thunder/thunder_bgx.h"
#include "../thunder/nic.h"
#include "pki.h"
#include "bgx.h"

#define NIC_PORT_CTX_LINUX      0 /* Control plane/Linux */
#define NIC_PORT_CTX_DATAPLANE  1 /* Data plane */

static int bgx_port_open(struct octtx_bgx_port *port);
static int bgx_port_close(struct octtx_bgx_port *port);
static int bgx_port_start(struct octtx_bgx_port *port);
static int bgx_port_stop(struct octtx_bgx_port *port);
static int bgx_port_config(struct octtx_bgx_port *port,
			   mbox_bgx_port_conf_t *conf);
static int bgx_port_status(struct octtx_bgx_port *port,
			   mbox_bgx_port_status_t *stat);
static int bgx_port_stats_get(struct octtx_bgx_port *port,
			      mbox_bgx_port_stats_t *stat);
static int bgx_port_stats_clr(struct octtx_bgx_port *port);
static int bgx_port_link_status(struct octtx_bgx_port *port, u8 *up);
static int bgx_port_set_link_state(struct octtx_bgx_port *port, bool enable);
static int bgx_port_promisc_set(struct octtx_bgx_port *port, u8 on);
static int bgx_port_macaddr_set(struct octtx_bgx_port *port, u8 macaddr[]);
static int bgx_port_macaddr_add(struct octtx_bgx_port *port,
				struct mbox_bgx_port_mac_filter *filter);
static int bgx_port_macaddr_del(struct octtx_bgx_port *port, int index);
static int bgx_port_macaddr_max_entries_get(struct octtx_bgx_port *port,
					    int *resp);
static int bgx_port_bp_set(struct octtx_bgx_port *port, u8 on);
static int bgx_port_bcast_set(struct octtx_bgx_port *port, u8 on);
static int bgx_port_mcast_set(struct octtx_bgx_port *port, u8 on);
static int bgx_port_mtu_set(struct octtx_bgx_port *port, u16 mtu);
static int bgx_port_get_fifo_cfg(struct octtx_bgx_port *port,
				 struct mbox_bgx_port_fifo_cfg *conf);
static int bgx_port_flow_ctrl_cfg(struct octtx_bgx_port *port,
				  struct mbox_bgx_port_fc_cfg *conf);
static int bgx_port_change_mode(struct octtx_bgx_port *port,
				void *conf);

#define BGX_LMAC_NUM_CHANS 16
#define BGX_LMAC_BASE_CHAN(__bgx, __lmac) \
	(0x800 | ((__bgx) << 8) | ((__lmac) << 4)) /* PKI_CHAN_E */

#define BGX_INVALID_ID	(-1)

#define BGX_LMAC_DFLT_PAUSE_TIME		0x7FF

/* BGX CSRs (offsets from the PF base address for particular BGX:LMAC).
 * NOTE: Most of the CSR definitions are provided in thunder_bgx.h.
 * Here, only missing registers or those, which do not match OCTEONTX
 * definions in HRM.
 * TODO: Consider to provide here a full list of CSRs and use them instead
 * of those in the thunder driver.
 */
#define BGX_CMR_CONFIG			0x0
#define BGX_CMR_GLOBAL_CONFIG		0x8
#define BGX_CMR_RX_BP_ON		0xD0
#define BGX_CMR_RX_BP_OFF		0xD8
#define BGX_CMR_RX_BP_STATUS		0xF0
#define BGX_CMR_RX_DMAC_CAM(__dmac)	(0x200 + ((__dmac) * 0x8))
#define BGX_CMR_RX_DMAC_CAM_ACCEPT	BIT_ULL(3)
#define BGX_CMR_RX_DMAC_MCAST_ENABLE	BIT_ULL(1)
#define BGX_CMR_RX_DMAC_BCAST_ENABLE	BIT_ULL(0)
#define BGX_CMR_RX_DMAC_CAM_ENABLE	BIT_ULL(48)
#define BGX_CMR_RX_DMAC_CAM_LMACID(x)	((u64)(x) << 49)
#define BGX_CMR_RX_OVR_BP		0x470
#define BGX_CMR_TX_CHANNEL		0x500
#define BGX_CMR_PRT_CBFC_CTL		0x508
#define BGX_CMR_TX_OVR_BP		0x520

#define BGX_SMU_TX_PAUSE_PKT_TIME	0x20110
#define BGX_SMU_TX_PAUSE_PKT_INTERVAL	0x20120
#define BGX_SMU_HG2_CONTROL		0x20210

#define BGX_GMP_GMI_RXX_FRM_CTL		0x38028
#define BGX_GMP_GMI_TXX_PAUSE_PKT_TIME	0x38238
#define BGX_GMP_GMI_TXX_PAUSE_PKT_INTERVAL 0x38248
#define BGX_GMP_GMI_TXX_CTL		0x38270
#define BGX_CONST			0x40000

struct lmac_dmac_cfg {
	int		max_dmac;
	int		dmac;
};

/* BGX device Configuration and Control Block */
struct bgxpf {
	struct list_head list; /* List of BGX devices */
	void __iomem *reg_base;
	int node; /* CPU node */
	int bgx_idx; /* CPU-local BGX device index.*/
	int lmac_count;
	struct  lmac_dmac_cfg dmac_cfg[MAX_LMAC_PER_BGX];
};

struct lmac_cfg {
	u64	bgx_cmr_config;
	u64	bgx_cmrx_rx_id_map;
	u64	bgx_cmr_rx_ovr_bp;
	u64	bgx_cmr_tx_ovr_bp;
	u64	bgx_cmr_tx_channel;
	u64	bgx_smux_cbfc_ctl;
	u64	bgx_smu_hg2_control;
	u64	bgx_cmr_rx_bp_on;
	u64	bgx_smux_tx_thresh;
	u64	bgx_gmp_gmi_rxx_jabber;
	u64	bgx_smux_rx_jabber;
	u64	bgx_cmrx_rx_dmac_ctl;
	u8	mac[ETH_ALEN];
};

static struct lmac_cfg lmac_saved_cfg[MAX_BGX_PER_CN83XX * MAX_LMAC_PER_BGX];

/* Global lists of LBK devices and ports */
static DEFINE_MUTEX(octeontx_bgx_lock);
static LIST_HEAD(octeontx_bgx_devices);
static LIST_HEAD(octeontx_bgx_ports);

/* Interface with the thunder driver */
static struct thunder_bgx_com_s *thbgx;

static struct bgxpf *get_bgx_dev(int node, int bgx_idx)
{
	struct bgxpf *dev;

	list_for_each_entry(dev, &octeontx_bgx_devices, list) {
		if (dev->node == node && dev->bgx_idx == bgx_idx)
			return dev;
	}
	return NULL;
}

static struct octtx_bgx_port *get_bgx_port(int domain_id, int port_idx)
{
	struct octtx_bgx_port *port;

	mutex_lock(&octeontx_bgx_lock);
	list_for_each_entry(port, &octeontx_bgx_ports, list) {
		if (port->domain_id == domain_id &&
		    port->dom_port_idx == port_idx) {
			mutex_unlock(&octeontx_bgx_lock);
			return port;
		}
	}
	mutex_unlock(&octeontx_bgx_lock);
	return NULL;
}

static void bgx_reg_write(struct bgxpf *bgx, u64 lmac, u64 offset, u64 val)
{
	writeq_relaxed(val, bgx->reg_base + (lmac << 20) + offset);
}

static u64 bgx_reg_read(struct bgxpf *bgx, u64 lmac, u64 offset)
{
	return readq_relaxed(bgx->reg_base + (lmac << 20) + offset);
}

/* BGX Interface functions.
 */
static u64 mac2u64(const u8 *mac_addr)
{
	u64 mac = 0;
	int index;

	for (index = ETH_ALEN - 1; index >= 0; index--)
		mac |= ((u64)*mac_addr++) << (8 * index);

	return mac;
}

static void bgx_cam_flush_mac_addrs(int node, int bgx_idx, int lmac_id)
{
	struct bgxpf *bgx = get_bgx_dev(node, bgx_idx);
	struct lmac_dmac_cfg *lmac;
	u64 cfg = 0;
	u64 offset;

	if (!bgx)
		return;

	lmac = &bgx->dmac_cfg[lmac_id];
	while (lmac->dmac >= 0) {
		offset = (lmac->dmac * sizeof(u64)) +
			(lmac_id * lmac->max_dmac * sizeof(u64));
		bgx_reg_write(bgx, 0, BGX_CMR_RX_DMACX_CAM + offset, 0);
		lmac->dmac--;
	}

	cfg &= ~BGX_CMR_RX_DMAC_CAM_ACCEPT;
	cfg |= (BGX_CMR_RX_DMAC_BCAST_ENABLE | BGX_CMR_RX_DMAC_MCAST_ENABLE);
	bgx_reg_write(bgx, lmac_id, BGX_CMRX_RX_DMAC_CTL, cfg);

	lmac->dmac = 0;
}

static int bgx_cam_get_mac_max_entries(int node, int bgx_idx, int lmacid)
{
	struct lmac_dmac_cfg *lmac;
	struct bgxpf *bgx;

	bgx = get_bgx_dev(node, bgx_idx);
	if (!bgx)
		return -EINVAL;

	lmac = &bgx->dmac_cfg[lmacid];
	if (lmac)
		return lmac->max_dmac;

	return 0;
}

static void bgx_cam_insert_entry(struct bgxpf *bgx, int lmacid,
				 int index, const u8 *mac)
{
	u64 cfg = 0;

	/* Prepare entry */
	cfg = mac2u64(mac);
	cfg |= BGX_CMR_RX_DMAC_CAM_ENABLE;
	cfg |= BGX_CMR_RX_DMAC_CAM_LMACID(lmacid);

	/* Insert entry at given index */
	bgx_reg_write(bgx, 0, (BGX_CMR_RX_DMACX_CAM + (index * 8)), cfg);

	/* Enable CAM table */
	cfg = 0; /* Reset */
	cfg |= (BGX_CMR_RX_DMAC_BCAST_ENABLE | BGX_CMR_RX_DMAC_MCAST_ENABLE |
		BGX_CMR_RX_DMAC_CAM_ACCEPT);
	bgx_reg_write(bgx, lmacid, BGX_CMRX_RX_DMAC_CTL, cfg);
}

static int bgx_cam_add_mac_addr(int node, int bgx_idx, int lmacid,
				const u8 *mac, int index)
{
	struct lmac_dmac_cfg *lmac;
	struct bgxpf *bgx;

	bgx = get_bgx_dev(node, bgx_idx);
	if (!bgx)
		return -EINVAL;

	lmac = &bgx->dmac_cfg[lmacid];

	/* Validate the index */
	if (index < 0 || index >= lmac->max_dmac)
		return -EINVAL;

	/* Calculate real index of BGX DMAC table */
	index = lmacid * lmac->max_dmac + index;

	/* Configure the same MAC into CAM table */
	bgx_cam_insert_entry(bgx, lmacid, index, mac);

	lmac->dmac++;
	return 0;
}

static int bgx_cam_del_mac_addr(int node, int bgx_idx, int lmacid, int index)
{
	struct lmac_dmac_cfg *lmac;
	struct bgxpf *bgx;

	bgx = get_bgx_dev(node, bgx_idx);
	if (!bgx)
		return -EINVAL;

	lmac = &bgx->dmac_cfg[lmacid];

	/* Validate the index */
	if (index < 0 || index >= lmac->max_dmac)
		return -EINVAL;

	index = lmacid * lmac->max_dmac + index;
	bgx_reg_write(bgx, 0, (BGX_CMR_RX_DMACX_CAM + (index * 8)), 0);

	lmac->dmac--;
	return 0;
}

static int bgx_port_cam_restore(struct octtx_bgx_port *port)
{
	struct lmac_cfg *cfg;
	struct bgxpf *bgx;
	int lmac_idx, idx;

	if (!port)
		return -EINVAL;

	bgx = get_bgx_dev(port->node, port->bgx);
	if (!bgx)
		return -ENODEV;

	idx = port->bgx * MAX_LMAC_PER_BGX + port->lmac;
	lmac_idx = port->lmac;
	cfg = &lmac_saved_cfg[idx];

	/* Reset BGX CAM table */
	bgx_cam_flush_mac_addrs(port->node, port->bgx, port->lmac);

	/* Restore mac address */
	thbgx->set_mac_addr(port->node, port->bgx, port->lmac, cfg->mac);

	return 0;
}

static int bgx_get_num_ports(int node)
{
	struct octtx_bgx_port *port;
	int count = 0;

	mutex_lock(&octeontx_bgx_lock);
	list_for_each_entry(port, &octeontx_bgx_ports, list) {
		if (port->node == node)
			count++;
	}
	mutex_unlock(&octeontx_bgx_lock);
	return count;
}

static int bgx_get_link_status(int node, int bgx, int lmac)
{
	struct bgx_link_status link;

	thbgx->get_link_status(node, bgx, lmac, &link);
	return link.link_up;
}

static struct octtx_bgx_port *bgx_get_port_by_chan(int node, u16 domain_id,
						   int chan)
{
	struct octtx_bgx_port *port;
	int max_chan;

	mutex_lock(&octeontx_bgx_lock);
	list_for_each_entry(port, &octeontx_bgx_ports, list) {
		if (port->domain_id == BGX_INVALID_ID ||
		    port->domain_id != domain_id ||
				port->node != node)
			continue;
		max_chan = port->base_chan + port->num_chans;
		if (chan >= port->base_chan && chan < max_chan) {
			mutex_unlock(&octeontx_bgx_lock);
			return port;
		}
	}
	mutex_unlock(&octeontx_bgx_lock);
	return NULL;
}

static int bgx_set_ieee802_fc(struct bgxpf *bgx, int lmac, int lmac_type)
{
	u64 reg;

	/* Power-on values for all of the following registers.*/
	bgx_reg_write(bgx, 0, BGX_CMR_RX_OVR_BP, 0);
	bgx_reg_write(bgx, lmac, BGX_CMR_TX_OVR_BP, 0);
	bgx_reg_write(bgx, lmac, BGX_CMR_TX_CHANNEL, 0);

	switch (lmac_type) {
	case OCTTX_BGX_LMAC_TYPE_XAUI:
	case OCTTX_BGX_LMAC_TYPE_RXAUI:
	case OCTTX_BGX_LMAC_TYPE_10GR:
	case OCTTX_BGX_LMAC_TYPE_40GR:
		reg = (0xFFull << 48) | (0xFFull << 32);
		bgx_reg_write(bgx, lmac, BGX_SMUX_CBFC_CTL, reg);
		reg = (0x1ull << 16) | 0xFFFFull;
		bgx_reg_write(bgx, lmac, BGX_SMU_HG2_CONTROL, reg);

		/* Set pause time and interval*/
		bgx_reg_write(bgx, lmac, BGX_SMU_TX_PAUSE_PKT_TIME,
			      BGX_LMAC_DFLT_PAUSE_TIME);
		reg = bgx_reg_read(bgx, lmac, BGX_SMU_TX_PAUSE_PKT_INTERVAL);
		reg &= ~0xFFFFULL;
		bgx_reg_write(bgx, lmac, BGX_SMU_TX_PAUSE_PKT_INTERVAL,
			      reg | (BGX_LMAC_DFLT_PAUSE_TIME / 2));
		break;
	case OCTTX_BGX_LMAC_TYPE_SGMII:
	case OCTTX_BGX_LMAC_TYPE_QSGMII:
		/* Set pause time and interval*/
		bgx_reg_write(bgx, lmac, BGX_GMP_GMI_TXX_PAUSE_PKT_TIME,
			      BGX_LMAC_DFLT_PAUSE_TIME);
		reg = bgx_reg_read(bgx, lmac,
				   BGX_GMP_GMI_TXX_PAUSE_PKT_INTERVAL);
		reg &= ~0xFFFFULL;
		bgx_reg_write(bgx, lmac, BGX_GMP_GMI_TXX_PAUSE_PKT_INTERVAL,
			      reg | (BGX_LMAC_DFLT_PAUSE_TIME / 2));
		break;
	}
	return 0;
}

static int bgx_port_initial_config(struct octtx_bgx_port *port)
{
	struct bgxpf *bgx;
	u64 reg, thr;

	bgx = get_bgx_dev(port->node, port->bgx);
	if (!bgx)
		return -ENODEV;

	/* Adjust TX FIFO and BP thresholds to LMAC type */
	switch (port->lmac_type) {
	case OCTTX_BGX_LMAC_TYPE_40GR:
		reg = 0x400;
		thr = 0x200;
		break;
	case OCTTX_BGX_LMAC_TYPE_XAUI:
	case OCTTX_BGX_LMAC_TYPE_RXAUI:
	case OCTTX_BGX_LMAC_TYPE_10GR:
		reg = 0x100;
		thr = 0x80;
		break;
	default:
		reg = 0x100;
		thr = 0x20;
		break;
	}
	bgx_reg_write(bgx, port->lmac, BGX_CMR_RX_BP_ON, reg);
	bgx_reg_write(bgx, port->lmac, BGX_SMUX_TX_THRESH, thr);

	/* Enable IEEE-802.3 PAUSE flow-control */
	bgx_set_ieee802_fc(bgx, port->lmac, port->lmac_type);

	/* Route packet data to/from PKI/PKO */
	reg = bgx_reg_read(bgx, port->lmac, BGX_CMR_CONFIG);
	reg |= CMR_X2P_SELECT_PKI | CMR_P2X_SELECT_PKO;
	bgx_reg_write(bgx, port->lmac, BGX_CMR_CONFIG, reg);
	bgx_reg_write(bgx, port->lmac, BGX_CMRX_RX_ID_MAP, 0);
	return 0;
}

static int bgx_port_save_config(struct octtx_bgx_port *port)
{
	struct lmac_cfg *cfg;
	const u8 *mac_addr;
	struct bgxpf *bgx;
	int lmac_idx, idx;

	if (!port)
		return -EINVAL;

	bgx = get_bgx_dev(port->node, port->bgx);
	if (!bgx)
		return -ENODEV;

	idx = port->bgx * MAX_LMAC_PER_BGX + port->lmac;
	lmac_idx = port->lmac;
	cfg = &lmac_saved_cfg[idx];

	/* Save register configuration, the list below consists of registers
	 * which are currently being modified in bgx.c code. If new register
	 * is modified it should be saved here and restored in restore_lmac_cfg
	 * function.
	 */
	cfg->bgx_cmr_config =
		bgx_reg_read(bgx, lmac_idx, BGX_CMR_CONFIG);
	cfg->bgx_cmrx_rx_id_map =
		bgx_reg_read(bgx, lmac_idx, BGX_CMRX_RX_ID_MAP);
	cfg->bgx_cmr_rx_ovr_bp =
		bgx_reg_read(bgx, 0, BGX_CMR_RX_OVR_BP);
	cfg->bgx_cmr_tx_ovr_bp =
		bgx_reg_read(bgx, lmac_idx, BGX_CMR_TX_OVR_BP);
	cfg->bgx_cmr_tx_channel =
		bgx_reg_read(bgx, lmac_idx, BGX_CMR_TX_CHANNEL);
	cfg->bgx_smux_cbfc_ctl =
		bgx_reg_read(bgx, lmac_idx, BGX_SMUX_CBFC_CTL);
	cfg->bgx_smu_hg2_control =
		bgx_reg_read(bgx, lmac_idx, BGX_SMU_HG2_CONTROL);
	cfg->bgx_cmr_rx_bp_on =
		bgx_reg_read(bgx, lmac_idx, BGX_CMR_RX_BP_ON);
	cfg->bgx_smux_tx_thresh =
		bgx_reg_read(bgx, lmac_idx, BGX_SMUX_TX_THRESH);
	cfg->bgx_gmp_gmi_rxx_jabber =
		bgx_reg_read(bgx, lmac_idx, BGX_GMP_GMI_RXX_JABBER);
	cfg->bgx_smux_rx_jabber =
		bgx_reg_read(bgx, lmac_idx, BGX_SMUX_RX_JABBER);
	cfg->bgx_cmrx_rx_dmac_ctl =
		bgx_reg_read(bgx, lmac_idx, BGX_CMRX_RX_DMAC_CTL);

	/* Save mac address */
	mac_addr = thbgx->get_mac_addr(port->node, port->bgx, port->lmac);
	memcpy(cfg->mac, mac_addr, ETH_ALEN);

	return 0;
}

static int bgx_port_restore_config(struct octtx_bgx_port *port)
{
	struct lmac_cfg *cfg;
	struct bgxpf *bgx;
	int lmac_idx, idx;

	if (!port)
		return -EINVAL;

	bgx = get_bgx_dev(port->node, port->bgx);
	if (!bgx)
		return -ENODEV;

	idx = port->bgx * MAX_LMAC_PER_BGX + port->lmac;
	lmac_idx = port->lmac;
	cfg = &lmac_saved_cfg[idx];

	/* Restore register configuration */
	bgx_reg_write(bgx, lmac_idx, BGX_CMR_CONFIG,
		      cfg->bgx_cmr_config);
	bgx_reg_write(bgx, lmac_idx, BGX_CMRX_RX_ID_MAP,
		      cfg->bgx_cmrx_rx_id_map);
	bgx_reg_write(bgx, 0, BGX_CMR_RX_OVR_BP,
		      cfg->bgx_cmr_rx_ovr_bp);
	bgx_reg_write(bgx, lmac_idx, BGX_CMR_TX_OVR_BP,
		      cfg->bgx_cmr_tx_ovr_bp);
	bgx_reg_write(bgx, lmac_idx, BGX_CMR_TX_CHANNEL,
		      cfg->bgx_cmr_tx_channel);
	bgx_reg_write(bgx, lmac_idx, BGX_SMUX_CBFC_CTL,
		      cfg->bgx_smux_cbfc_ctl);
	bgx_reg_write(bgx, lmac_idx, BGX_SMU_HG2_CONTROL,
		      cfg->bgx_smu_hg2_control);
	bgx_reg_write(bgx, lmac_idx, BGX_CMR_RX_BP_ON,
		      cfg->bgx_cmr_rx_bp_on);
	bgx_reg_write(bgx, lmac_idx, BGX_SMUX_TX_THRESH,
		      cfg->bgx_smux_tx_thresh);
	bgx_reg_write(bgx, lmac_idx, BGX_GMP_GMI_RXX_JABBER,
		      cfg->bgx_gmp_gmi_rxx_jabber);
	bgx_reg_write(bgx, lmac_idx, BGX_SMUX_RX_JABBER,
		      cfg->bgx_smux_rx_jabber);
	bgx_reg_write(bgx, lmac_idx, BGX_CMRX_RX_DMAC_CTL,
		      cfg->bgx_cmrx_rx_dmac_ctl);

	/* Restore mac address */
	thbgx->set_mac_addr(port->node, port->bgx, port->lmac, cfg->mac);

	bgx_port_stats_clr(port);
	return 0;
}

/* Main MBOX message processing function.
 */
static int bgx_receive_message(u32 id, u16 domain_id, struct mbox_hdr *hdr,
			       union mbox_data *req,
			       union mbox_data *resp, void *mdata)
{
	struct octtx_bgx_port *port;
	int ret = 0;

	if (!mdata)
		return -ENOMEM;
	port = get_bgx_port(domain_id, hdr->vfid);
	if (!port) {
		hdr->res_code = MBOX_RET_INVALID;
		return -ENODEV;
	}
	switch (hdr->msg) {
	case MBOX_BGX_PORT_OPEN:
		ret = bgx_port_open(port);
		if (ret < 0)
			break;
		ret = bgx_port_config(port, mdata);
		resp->data = sizeof(mbox_bgx_port_conf_t);
		break;
	case MBOX_BGX_PORT_CLOSE:
		ret = bgx_port_close(port);
		resp->data = 0;
		break;
	case MBOX_BGX_PORT_START:
		ret = bgx_port_start(port);
		resp->data = 0;
		break;
	case MBOX_BGX_PORT_STOP:
		ret = bgx_port_stop(port);
		resp->data = 0;
		break;
	case MBOX_BGX_PORT_GET_CONFIG:
		ret = bgx_port_config(port, mdata);
		resp->data = sizeof(mbox_bgx_port_conf_t);
		break;
	case MBOX_BGX_PORT_GET_STATUS:
		ret = bgx_port_status(port, mdata);
		resp->data = sizeof(mbox_bgx_port_status_t);
		break;
	case MBOX_BGX_PORT_GET_STATS:
		ret = bgx_port_stats_get(port, mdata);
		resp->data = sizeof(mbox_bgx_port_stats_t);
		break;
	case MBOX_BGX_PORT_CLR_STATS:
		ret = bgx_port_stats_clr(port);
		resp->data = 0;
		break;
	case MBOX_BGX_PORT_GET_LINK_STATUS:
		ret = bgx_port_link_status(port, mdata);
		resp->data = sizeof(u8);
		break;
	case MBOX_BGX_PORT_SET_PROMISC:
		ret = bgx_port_promisc_set(port, *(u8 *)mdata);
		resp->data = 0;
		break;
	case MBOX_BGX_PORT_SET_MACADDR:
		ret = bgx_port_macaddr_set(port, mdata);
		resp->data = 0;
		break;
	case MBOX_BGX_PORT_ADD_MACADDR:
		ret = bgx_port_macaddr_add(port, mdata);
		resp->data = 0;
		break;
	case MBOX_BGX_PORT_DEL_MACADDR:
		ret = bgx_port_macaddr_del(port, *(int *)mdata);
		resp->data = 0;
		break;
	case MBOX_BGX_PORT_GET_MACADDR_ENTRIES:
		ret = bgx_port_macaddr_max_entries_get(port, (int *)mdata);
		resp->data = sizeof(int);
		break;
	case MBOX_BGX_PORT_SET_BP:
		ret = bgx_port_bp_set(port, *(u8 *)mdata);
		resp->data = 0;
		break;
	case MBOX_BGX_PORT_SET_BCAST:
		ret = bgx_port_bcast_set(port, *(u8 *)mdata);
		resp->data = 0;
		break;
	case MBOX_BGX_PORT_SET_MCAST:
		ret = bgx_port_mcast_set(port, *(u8 *)mdata);
		resp->data = 0;
		break;
	case MBOX_BGX_PORT_SET_MTU:
		ret = bgx_port_mtu_set(port, *(u16 *)mdata);
		resp->data = 0;
		break;
	case MBOX_BGX_PORT_GET_FIFO_CFG:
		ret = bgx_port_get_fifo_cfg(port, mdata);
		resp->data = sizeof(struct mbox_bgx_port_fifo_cfg);
		break;
	case MBOX_BGX_PORT_FLOW_CTRL_CFG:
		ret = bgx_port_flow_ctrl_cfg(port, mdata);
		resp->data = sizeof(struct mbox_bgx_port_fc_cfg);
		break;
	case MBOX_BGX_PORT_SET_LINK_STATE:
		ret = bgx_port_set_link_state(port, *(bool *)mdata);
		resp->data = 0;
		break;
	case MBOX_BGX_PORT_CHANGE_MODE:
		ret = bgx_port_change_mode(port, mdata);
		resp->data = ret;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	if (ret) {
		hdr->res_code = MBOX_RET_INVALID;
		return -EINVAL;
	}
	hdr->res_code = MBOX_RET_SUCCESS;
	return 0;
}

/* MBOX message processing support functions.
 */
int bgx_port_open(struct octtx_bgx_port *port)
{
	struct bgxpf *bgx;

	bgx = get_bgx_dev(port->node, port->bgx);
	if (!bgx)
		return -EINVAL;

	/* Setup PKI port (pkind): */
	bgx_reg_write(bgx, port->lmac, BGX_CMRX_RX_ID_MAP, port->pkind);
	return 0;
}

int bgx_port_close(struct octtx_bgx_port *port)
{
	struct bgxpf *bgx;

	bgx = get_bgx_dev(port->node, port->bgx);
	if (!bgx)
		return -EINVAL;
	/* Park the BGX output to the PKI port 0: */
	bgx_reg_write(bgx, port->lmac, BGX_CMRX_RX_ID_MAP, 0);
	return 0;
}

int bgx_port_start(struct octtx_bgx_port *port)
{
	thbgx->enable(port->node, port->bgx, port->lmac);
	return 0;
}

int bgx_port_stop(struct octtx_bgx_port *port)
{
	thbgx->disable(port->node, port->bgx, port->lmac);
	return 0;
}

int bgx_port_config(struct octtx_bgx_port *port, mbox_bgx_port_conf_t *conf)
{
	struct bgxpf *bgx;
	const u8 *macaddr;
	u64 reg;

	bgx = get_bgx_dev(port->node, port->bgx);
	if (!bgx)
		return -EINVAL;

	conf->node = port->node;
	conf->bgx = port->bgx;
	conf->lmac = port->lmac;
	conf->base_chan = port->base_chan;
	conf->num_chans = port->num_chans;

	reg = bgx_reg_read(bgx, port->lmac, BGX_CMRX_RX_ID_MAP);
	conf->pkind = reg & 0x3F; /* PKND */

	reg = bgx_reg_read(bgx, port->lmac, BGX_CMR_CONFIG);
	conf->mode = (reg >> 8) & 0x7; /* LMAC_TYPE */
	conf->enable = (reg & CMR_PKT_TX_EN) &&
			(reg & CMR_PKT_RX_EN) && (reg & CMR_EN);

	reg = bgx_reg_read(bgx, 0, BGX_CMR_GLOBAL_CONFIG);
	conf->fcs_strip = (reg >> 6) & 0x1; /* FCS_STRIP */

	reg = bgx_reg_read(bgx, port->lmac, BGX_CMRX_RX_DMAC_CTL);
	conf->bcast_mode = reg & 0x1; /* BCAST_ACCEPT */
	conf->mcast_mode = (reg >> 1) & 0x3; /* MCAST_MODE */
	conf->promisc = !((reg >> 3) & 0x1); /* reverse of CAM_ACCEPT */

	macaddr = thbgx->get_mac_addr(port->node, port->bgx, port->lmac);
	memcpy(conf->macaddr, macaddr, 6);

	switch (conf->mode) {
	case OCTTX_BGX_LMAC_TYPE_SGMII:
	case OCTTX_BGX_LMAC_TYPE_QSGMII:
		reg = bgx_reg_read(bgx, port->lmac, BGX_GMP_GMI_RXX_JABBER);
		conf->mtu = reg & 0xFFFF;
		break;
	case OCTTX_BGX_LMAC_TYPE_XAUI:
	case OCTTX_BGX_LMAC_TYPE_RXAUI:
	case OCTTX_BGX_LMAC_TYPE_10GR:
	case OCTTX_BGX_LMAC_TYPE_40GR:
		reg = bgx_reg_read(bgx, port->lmac, BGX_SMUX_RX_JABBER);
		conf->mtu = reg & 0xFFFF;
		break;
	}
	return 0;
}

static int bgx_port_get_fifo_cfg(struct octtx_bgx_port *port,
				 struct mbox_bgx_port_fifo_cfg *conf)
{
	struct bgxpf *bgx;
	u64 fifo_size;

	bgx = get_bgx_dev(port->node, port->bgx);
	if (!bgx)
		return -EINVAL;

	fifo_size = bgx_reg_read(bgx, 0, BGX_CONST) & 0xffffff;
	conf->rx_fifosz = (u32)fifo_size / bgx->lmac_count;

	return 0;
}

static int bgx_port_flow_ctrl_cfg(struct octtx_bgx_port *port,
				  struct mbox_bgx_port_fc_cfg *conf)
{
	u16 high_water = conf->high_water;
	u16 low_water = conf->low_water;
	u8 rx_pause = conf->rx_pause;
	u8 tx_pause = conf->tx_pause;
	u16 max_hwater, min_lwater;
	u16 fifo_size, drop_mark;
	struct bgxpf *bgx;
	int lmac;
	u64 reg;
	u8 smu;

#define BGX_RX_FRM_CTL_CTL_BCK		BIT_ULL(3)
#define BGX_SMUX_TX_CTL_L2P_BP_CONV	BIT_ULL(7)
#define BGX_GMP_GMI_TX_FC_TYPE		BIT_ULL(2)
#define BGX_CMR_RX_OVR_BP_EN(X)		BIT_ULL(((X) + 8))
#define BGX_CMR_RX_OVR_BP_BP(X)		BIT_ULL(((X) + 4))
#define RX_BP_OFF_MIN_MARK		(0x10)

	bgx = get_bgx_dev(port->node, port->bgx);
	if (!bgx)
		return -EINVAL;

	lmac = port->lmac;

	switch (port->lmac_type) {
	case OCTTX_BGX_LMAC_TYPE_40GR:
	case OCTTX_BGX_LMAC_TYPE_XAUI:
	case OCTTX_BGX_LMAC_TYPE_RXAUI:
	case OCTTX_BGX_LMAC_TYPE_10GR:
		smu = true;
		break;
	case OCTTX_BGX_LMAC_TYPE_SGMII:
	case OCTTX_BGX_LMAC_TYPE_QSGMII:
		smu = false;
		break;
	default:
		return -EINVAL;
	}

	if (conf->fc_cfg == BGX_PORT_FC_CFG_SET) {
		if (!high_water || !low_water || high_water < low_water)
			return -EINVAL;

		fifo_size = bgx_reg_read(bgx, 0, BGX_CONST) & 0xffffff;
		fifo_size /= bgx->lmac_count;

		drop_mark = bgx_reg_read(bgx, lmac, BGX_CMRX_RX_BP_DROP) & 0x3f;
		drop_mark *= 16;

		/* max_hwater & min_lwater in bytes */
		max_hwater = fifo_size - drop_mark;
		min_lwater = RX_BP_OFF_MIN_MARK * 16;

		if (high_water > max_hwater || low_water < min_lwater)
			return -EINVAL;

		bgx_reg_write(bgx, lmac, BGX_CMR_RX_BP_ON, high_water / 16);
		bgx_reg_write(bgx, lmac, BGX_CMR_RX_BP_OFF, low_water / 16);

		if (smu) {
			reg = bgx_reg_read(bgx, lmac, BGX_SMUX_RX_FRM_CTL);
			reg &= ~BGX_RX_FRM_CTL_CTL_BCK;
			reg |= FIELD_PREP(BGX_RX_FRM_CTL_CTL_BCK, !!rx_pause);
			bgx_reg_write(bgx, lmac, BGX_SMUX_RX_FRM_CTL, reg);

			reg = bgx_reg_read(bgx, lmac, BGX_SMUX_TX_CTL);
			reg &= ~BGX_SMUX_TX_CTL_L2P_BP_CONV;
			reg |= FIELD_PREP(BGX_SMUX_TX_CTL_L2P_BP_CONV,
					!!tx_pause);
			bgx_reg_write(bgx, lmac, BGX_SMUX_TX_CTL, reg);
		} else {
			reg = bgx_reg_read(bgx, lmac, BGX_GMP_GMI_RXX_FRM_CTL);
			reg &= ~BGX_RX_FRM_CTL_CTL_BCK;
			reg |= FIELD_PREP(BGX_RX_FRM_CTL_CTL_BCK, !!rx_pause);
			bgx_reg_write(bgx, lmac, BGX_GMP_GMI_RXX_FRM_CTL, reg);

			reg = bgx_reg_read(bgx, lmac, BGX_GMP_GMI_TXX_CTL);
			reg &= ~BGX_GMP_GMI_TX_FC_TYPE;
			reg |= FIELD_PREP(BGX_GMP_GMI_TX_FC_TYPE, !!tx_pause);
			bgx_reg_write(bgx, lmac, BGX_GMP_GMI_TXX_CTL, reg);
		}

		reg = bgx_reg_read(bgx, 0, BGX_CMR_RX_OVR_BP);
		if (tx_pause) {
			reg &= ~BGX_CMR_RX_OVR_BP_EN(lmac);
		} else {
			reg |= BGX_CMR_RX_OVR_BP_EN(lmac);
			reg &= ~BGX_CMR_RX_OVR_BP_BP(lmac);
		}
		bgx_reg_write(bgx, 0, BGX_CMR_RX_OVR_BP, reg);

		return 0;
	}

	if (smu) {
		reg = bgx_reg_read(bgx, lmac, BGX_SMUX_RX_FRM_CTL);
		conf->rx_pause = !!(reg & BGX_RX_FRM_CTL_CTL_BCK);

		reg = bgx_reg_read(bgx, lmac, BGX_SMUX_TX_CTL);
		conf->tx_pause = !!(reg & BGX_SMUX_TX_CTL_L2P_BP_CONV);
	} else {
		reg = bgx_reg_read(bgx, lmac, BGX_GMP_GMI_RXX_FRM_CTL);
		conf->rx_pause = !!(reg & BGX_RX_FRM_CTL_CTL_BCK);

		reg = bgx_reg_read(bgx, lmac, BGX_GMP_GMI_TXX_CTL);
		conf->tx_pause = !!(reg & BGX_GMP_GMI_TX_FC_TYPE);
	}

	conf->high_water = bgx_reg_read(bgx, lmac, BGX_CMR_RX_BP_ON) & 0xfff;
	conf->low_water = bgx_reg_read(bgx, lmac, BGX_CMR_RX_BP_OFF) & 0x7f;
	conf->high_water *= 16;
	conf->low_water *= 16;

	return 0;
}

int bgx_port_status(struct octtx_bgx_port *port, mbox_bgx_port_status_t *stat)
{
	struct bgxpf *bgx;
	struct bgx_link_status link;
	u64 reg;

	bgx = get_bgx_dev(port->node, port->bgx);
	if (!bgx)
		return -EINVAL;

	reg = bgx_reg_read(bgx, port->lmac, BGX_CMR_RX_BP_STATUS);
	stat->bp = reg & 0x1; /* BP */

	thbgx->get_link_status(port->node, port->bgx, port->lmac, &link);
	stat->link_up = link.link_up;
	stat->duplex = link.duplex;
	stat->speed = link.speed;
	return 0;
}

int bgx_port_stats_get(struct octtx_bgx_port *port,
		       mbox_bgx_port_stats_t *stats)
{
	struct bgxpf *bgx;
	struct pki_com_s *pki;
	int ret;
	u64 reg;

	bgx = get_bgx_dev(port->node, port->bgx);
	if (!bgx)
		return -EINVAL;
	memset(stats, 0, sizeof(mbox_bgx_port_stats_t));
	reg = bgx_reg_read(bgx, port->lmac, BGX_CMRX_RX_STAT0);
	stats->rx_packets = reg;
	reg = bgx_reg_read(bgx, port->lmac, BGX_CMRX_RX_STAT1);
	stats->rx_bytes = reg;
	reg = bgx_reg_read(bgx, port->lmac, BGX_CMRX_RX_STAT2);
	stats->rx_pause_packets = reg;
	reg = bgx_reg_read(bgx, port->lmac, BGX_CMRX_RX_STAT4);
	stats->rx_dropped = reg;
	reg = bgx_reg_read(bgx, port->lmac, BGX_CMRX_RX_STAT6);
	stats->rx_dropped += reg;
	reg = bgx_reg_read(bgx, port->lmac, BGX_CMRX_RX_STAT8);
	stats->rx_errors = reg;
	reg = bgx_reg_read(bgx, 0, BGX_CMRX_RX_STAT9);
	stats->rx_dropped += reg;

	reg = bgx_reg_read(bgx, port->lmac, BGX_CMRX_TX_STAT0);
	stats->tx_dropped = reg;
	stats->collisions = reg;
	reg = bgx_reg_read(bgx, port->lmac, BGX_CMRX_TX_STAT2);
	stats->collisions += reg;
	reg = bgx_reg_read(bgx, port->lmac, BGX_CMRX_TX_STAT3);
	stats->collisions += reg;
	reg = bgx_reg_read(bgx, port->lmac, BGX_CMRX_TX_STAT4);
	stats->tx_bytes = reg;
	reg = bgx_reg_read(bgx, port->lmac, BGX_CMRX_TX_STAT5);
	stats->tx_packets = reg;
	reg = bgx_reg_read(bgx, port->lmac, BGX_CMRX_TX_STAT6);
	stats->tx_1_to_64_packets = reg;
	reg = bgx_reg_read(bgx, port->lmac, BGX_CMRX_TX_STAT7);
	stats->tx_1_to_64_packets += reg;
	reg = bgx_reg_read(bgx, port->lmac, BGX_CMRX_TX_STAT8);
	stats->tx_65_to_127_packets = reg;
	reg = bgx_reg_read(bgx, port->lmac, BGX_CMRX_TX_STAT9);
	stats->tx_128_to_255_packets = reg;
	reg = bgx_reg_read(bgx, port->lmac, BGX_CMRX_TX_STAT10);
	stats->tx_256_to_511_packets = reg;
	reg = bgx_reg_read(bgx, port->lmac, BGX_CMRX_TX_STAT11);
	stats->tx_512_to_1023_packets = reg;
	reg = bgx_reg_read(bgx, port->lmac, BGX_CMRX_TX_STAT12);
	stats->tx_1024_to_1522_packets = reg;
	reg = bgx_reg_read(bgx, port->lmac, BGX_CMRX_TX_STAT13);
	stats->tx_1523_to_max_packets = reg;
	reg = bgx_reg_read(bgx, port->lmac, BGX_CMRX_TX_STAT14);
	stats->tx_broadcast_packets = reg;
	reg = bgx_reg_read(bgx, port->lmac, BGX_CMRX_TX_STAT15);
	stats->tx_multicast_packets = reg;
	reg = bgx_reg_read(bgx, port->lmac, BGX_CMRX_TX_STAT17);
	stats->tx_pause_packets = reg;

	pki = try_then_request_module(symbol_get(pki_com), "pki");
	if (pki) {
		ret = pki->get_bgx_port_stats(port);
		if (!ret)
			stats->rx_dropped += port->stats.rxdrop;
		symbol_put(pki_com);
	}

	return 0;
}

int bgx_port_stats_clr(struct octtx_bgx_port *port)
{
	struct bgxpf *bgx;

	bgx = get_bgx_dev(port->node, port->bgx);
	if (!bgx)
		return -EINVAL;
	bgx_reg_write(bgx, port->lmac, BGX_CMRX_RX_STAT0, 0);
	bgx_reg_write(bgx, port->lmac, BGX_CMRX_RX_STAT1, 0);
	bgx_reg_write(bgx, port->lmac, BGX_CMRX_RX_STAT2, 0);
	bgx_reg_write(bgx, port->lmac, BGX_CMRX_RX_STAT3, 0);
	bgx_reg_write(bgx, port->lmac, BGX_CMRX_RX_STAT4, 0);
	bgx_reg_write(bgx, port->lmac, BGX_CMRX_RX_STAT5, 0);
	bgx_reg_write(bgx, port->lmac, BGX_CMRX_RX_STAT6, 0);
	bgx_reg_write(bgx, port->lmac, BGX_CMRX_RX_STAT7, 0);
	bgx_reg_write(bgx, port->lmac, BGX_CMRX_RX_STAT8, 0);
	bgx_reg_write(bgx, 0, BGX_CMRX_RX_STAT9, 0);
	bgx_reg_write(bgx, 0, BGX_CMRX_RX_STAT10, 0);

	bgx_reg_write(bgx, port->lmac, BGX_CMRX_TX_STAT0, 0);
	bgx_reg_write(bgx, port->lmac, BGX_CMRX_TX_STAT1, 0);
	bgx_reg_write(bgx, port->lmac, BGX_CMRX_TX_STAT2, 0);
	bgx_reg_write(bgx, port->lmac, BGX_CMRX_TX_STAT3, 0);
	bgx_reg_write(bgx, port->lmac, BGX_CMRX_TX_STAT4, 0);
	bgx_reg_write(bgx, port->lmac, BGX_CMRX_TX_STAT5, 0);
	bgx_reg_write(bgx, port->lmac, BGX_CMRX_TX_STAT6, 0);
	bgx_reg_write(bgx, port->lmac, BGX_CMRX_TX_STAT7, 0);
	bgx_reg_write(bgx, port->lmac, BGX_CMRX_TX_STAT8, 0);
	bgx_reg_write(bgx, port->lmac, BGX_CMRX_TX_STAT9, 0);
	bgx_reg_write(bgx, port->lmac, BGX_CMRX_TX_STAT10, 0);
	bgx_reg_write(bgx, port->lmac, BGX_CMRX_TX_STAT11, 0);
	bgx_reg_write(bgx, port->lmac, BGX_CMRX_TX_STAT12, 0);
	bgx_reg_write(bgx, port->lmac, BGX_CMRX_TX_STAT13, 0);
	bgx_reg_write(bgx, port->lmac, BGX_CMRX_TX_STAT14, 0);
	bgx_reg_write(bgx, port->lmac, BGX_CMRX_TX_STAT15, 0);
	bgx_reg_write(bgx, port->lmac, BGX_CMRX_TX_STAT16, 0);
	bgx_reg_write(bgx, port->lmac, BGX_CMRX_TX_STAT17, 0);
	return 0;
}

int bgx_port_set_link_state(struct octtx_bgx_port *port, bool enable)
{
	return thbgx->set_link_state(port->node, port->bgx, port->lmac, enable);
}

int bgx_port_change_mode(struct octtx_bgx_port *port,
			 void *conf)
{
	return thbgx->change_mode(port->node, port->bgx, port->lmac, conf);
}

int bgx_port_link_status(struct octtx_bgx_port *port, u8 *up)
{
	*up = bgx_get_link_status(port->node, port->bgx, port->lmac);
	return 0;
}

int bgx_port_promisc_set(struct octtx_bgx_port *port, u8 on)
{
	struct bgxpf *bgx;
	u64 reg;
	int i;

	bgx = get_bgx_dev(port->node, port->bgx);
	if (!bgx)
		return -EINVAL;

	if (on) { /* Enable */
		/* CAM_ACCEPT = 0 */
		reg = 0x1; /* BCAST_ACCEPT = 1 */
		reg |= 0x1ull << 1; /* MCAST_MODE = 1 */
		bgx_reg_write(bgx, port->lmac, BGX_CMRX_RX_DMAC_CTL, reg);

		for (i = 0; i < 32; i++) {
			reg = bgx_reg_read(bgx, 0, BGX_CMR_RX_DMAC_CAM(i));
			if (((reg >> 49) & 0x3)/* ID */ == port->lmac)
				reg &= ~(0x1ull << 48); /* EN = 0*/
			bgx_reg_write(bgx, 0, BGX_CMR_RX_DMAC_CAM(i), reg);
		}
	} else { /* Disable = enable packet filtering */
		reg = 0x1ull << 3; /* CAM_ACCEPT = 1 */
		reg |= 0x1ull << 1; /* MCAST_MODE = 1 */
		reg |= 0x1; /* BCAST_ACCEPT = 1 */
		bgx_reg_write(bgx, port->lmac, BGX_CMRX_RX_DMAC_CTL, reg);

		for (i = 0; i < 32; i++) {
			reg = bgx_reg_read(bgx, 0, BGX_CMR_RX_DMAC_CAM(i));
			if (((reg >> 49) & 0x3)/* ID */ == port->lmac)
				reg |= 0x1ull << 48; /* EN = 1 */
			bgx_reg_write(bgx, 0, BGX_CMR_RX_DMAC_CAM(i), reg);
		}
	}
	return 0;
}

int bgx_port_macaddr_set(struct octtx_bgx_port *port, u8 macaddr[])
{
	thbgx->set_mac_addr(port->node, port->bgx, port->lmac, macaddr);
	return 0;
}

int bgx_port_macaddr_max_entries_get(struct octtx_bgx_port *port, int *resp)
{
	int rc = 0;

	rc = bgx_cam_get_mac_max_entries(port->node, port->bgx, port->lmac);
	if (rc < 0)
		return rc;

	*resp = rc;
	return 0;
}

int bgx_port_macaddr_add(struct octtx_bgx_port *port,
			 struct mbox_bgx_port_mac_filter *filter)
{
	return bgx_cam_add_mac_addr(port->node, port->bgx, port->lmac,
				    filter->mac_addr, filter->index);
}

int bgx_port_macaddr_del(struct octtx_bgx_port *port, int index)
{
	return bgx_cam_del_mac_addr(port->node, port->bgx, port->lmac, index);
}

int bgx_port_bp_set(struct octtx_bgx_port *port, u8 on)
{
	struct bgxpf *bgx;

	bgx = get_bgx_dev(port->node, port->bgx);
	if (!bgx)
		return -EINVAL;

	/* TODO: Setup channel backpressure */
	return 0;
}

int bgx_port_bcast_set(struct octtx_bgx_port *port, u8 on)
{
	struct bgxpf *bgx;
	u64 reg;

	bgx = get_bgx_dev(port->node, port->bgx);
	if (!bgx)
		return -EINVAL;
	reg = bgx_reg_read(bgx, port->lmac, BGX_CMRX_RX_DMAC_CTL);
	if (on)
		reg |= 0x1; /* BCAST_ACCEPT = 1 */
	else
		reg &= ~0x1; /* BCAST_ACCEPT = 0 */
	bgx_reg_write(bgx, port->lmac, BGX_CMRX_RX_DMAC_CTL, reg);
	return 0;
}

int bgx_port_mcast_set(struct octtx_bgx_port *port, u8 on)
{
	struct bgxpf *bgx;
	u64 reg;

	bgx = get_bgx_dev(port->node, port->bgx);
	if (!bgx)
		return -EINVAL;
	reg = bgx_reg_read(bgx, port->lmac, BGX_CMRX_RX_DMAC_CTL);
	if (on)
		reg |= (1ull << 1) & 0x3; /* MCAST_MODE = 1 */
	else
		reg &= ~(0x3ull << 1); /* MCAST_MODE = 0 */
	bgx_reg_write(bgx, port->lmac, BGX_CMRX_RX_DMAC_CTL, reg);
	return 0;
}

int bgx_port_mtu_set(struct octtx_bgx_port *port, u16 mtu)
{
	struct bgxpf *bgx;

	bgx = get_bgx_dev(port->node, port->bgx);
	if (!bgx)
		return -EINVAL;

	switch (port->lmac_type) {
	case OCTTX_BGX_LMAC_TYPE_SGMII:
	case OCTTX_BGX_LMAC_TYPE_QSGMII:
		bgx_reg_write(bgx, port->lmac, BGX_GMP_GMI_RXX_JABBER, mtu);
		break;
	case OCTTX_BGX_LMAC_TYPE_XAUI:
	case OCTTX_BGX_LMAC_TYPE_RXAUI:
	case OCTTX_BGX_LMAC_TYPE_10GR:
	case OCTTX_BGX_LMAC_TYPE_40GR:
		bgx_reg_write(bgx, port->lmac, BGX_SMUX_RX_JABBER, mtu);
		break;
	}
	return 0;
}

int bgx_get_port_stats(struct octtx_bgx_port *port)
{
	struct bgxpf *bgx;
	u64 reg;

	bgx = get_bgx_dev(port->node, port->bgx);
	if (!bgx)
		return -EINVAL;
	reg = bgx_reg_read(bgx, port->lmac, BGX_CMRX_RX_STAT0);
	port->stats.rxpkts = reg & ((1ull << 47) - 1);
	reg = bgx_reg_read(bgx, port->lmac, BGX_CMRX_RX_STAT1);
	port->stats.rxbytes = reg & ((1ull << 47) - 1);
	reg = bgx_reg_read(bgx, port->lmac, BGX_CMRX_RX_STAT6);
	port->stats.rxdrop = reg & ((1ull << 47) - 1);
	reg = bgx_reg_read(bgx, port->lmac, BGX_CMRX_RX_STAT8);
	port->stats.rxerr = reg & ((1ull << 47) - 1);

	reg = bgx_reg_read(bgx, port->lmac, BGX_CMRX_TX_STAT5);
	port->stats.txpkts = reg & ((1ull << 47) - 1);
	reg = bgx_reg_read(bgx, port->lmac, BGX_CMRX_TX_STAT4);
	port->stats.txbytes = reg & ((1ull << 47) - 1);
	reg = bgx_reg_read(bgx, port->lmac, BGX_CMRX_TX_STAT0);
	port->stats.txdrop = reg & ((1ull << 47) - 1);
	reg = bgx_reg_read(bgx, port->lmac, BGX_CMRX_TX_STAT16);
	port->stats.txerr = reg & ((1ull << 47) - 1);
	reg = bgx_reg_read(bgx, port->lmac, BGX_CMRX_TX_STAT14);
	port->stats.txbcast = reg & ((1ull << 47) - 1);
	reg = bgx_reg_read(bgx, port->lmac, BGX_CMRX_TX_STAT15);
	port->stats.txmcast = reg & ((1ull << 47) - 1);
	return 0;
}

/* Domain destroy function.
 */
static int bgx_destroy_domain(u32 id, u16 domain_id, struct kobject *kobj)
{
	struct octtx_bgx_port *port;

	mutex_lock(&octeontx_bgx_lock);
	list_for_each_entry(port, &octeontx_bgx_ports, list) {
		if (port->node == id && port->domain_id == domain_id) {
			/* Return port to Linux */
			bgx_port_restore_config(port);
			thbgx->switch_ctx(port->node, port->bgx, port->lmac,
					  NIC_PORT_CTX_LINUX, 0);

			port->domain_id = BGX_INVALID_ID;
			port->dom_port_idx = BGX_INVALID_ID;
		}
	}
	mutex_unlock(&octeontx_bgx_lock);
	return 0;
}

/* Domain create function.
 */
static int bgx_create_domain(u32 id, u16 domain_id,
			     struct octtx_bgx_port *port_tbl, int ports,
			     struct octeontx_master_com_t *com, void *domain,
			     struct kobject *kobj)
{
	struct octtx_bgx_port *port, *gport;
	int port_idx, ret = 0;

	/* For each domain port, find requested entry in the list of
	 * global ports and sync up those two port structures.
	 */
	mutex_lock(&octeontx_bgx_lock);
	for (port_idx = 0; port_idx < ports; port_idx++) {
		port = &port_tbl[port_idx];

		list_for_each_entry(gport, &octeontx_bgx_ports, list) {
			if (gport->node != id)
				continue;
			if (gport->glb_port_idx != port->glb_port_idx)
				continue;
			/* Check for conflicts with other domains. */
			if (gport->domain_id != BGX_INVALID_ID) {
				ret = -EINVAL;
				goto err_unlock;
			}
			/* Sync up global and domain ports. */
			port->node = gport->node;
			port->bgx = gport->bgx;
			port->lmac = gport->lmac;
			port->lmac_type = gport->lmac_type;
			port->tx_fifo_sz = gport->tx_fifo_sz;
			port->base_chan = gport->base_chan;
			port->num_chans = gport->num_chans;

			gport->domain_id = domain_id;
			gport->dom_port_idx = port_idx;

			/* Stop and reconfigure the port.*/
			bgx_port_stop(port);
			ret = bgx_port_save_config(port);
			if (ret)
				goto err_unlock;

			thbgx->switch_ctx(port->node, port->bgx, port->lmac,
					  NIC_PORT_CTX_DATAPLANE, port_idx);

			ret = bgx_port_initial_config(port);
			if (ret)
				goto err_unlock;
		}
	}
	mutex_unlock(&octeontx_bgx_lock);
	return ret;

err_unlock:
	mutex_unlock(&octeontx_bgx_lock);
	bgx_destroy_domain(id, domain_id, kobj);
	return ret;
}

/* Domain reset function.
 */
static int bgx_reset_domain(u32 id, u16 domain_id)
{
	struct octtx_bgx_port *port;

	mutex_lock(&octeontx_bgx_lock);
	list_for_each_entry(port, &octeontx_bgx_ports, list) {
		if (port->node == id && port->domain_id == domain_id) {
			bgx_port_stop(port);
			bgx_port_cam_restore(port);
		}
	}
	mutex_unlock(&octeontx_bgx_lock);
	return 0;
}

/* Set pkind for a given port.
 */
static int bgx_set_pkind(u32 id, u16 domain_id, int port, int pkind)
{
	struct octtx_bgx_port *gport;

	gport = get_bgx_port(domain_id, port);
	if (!gport)
		return -EINVAL;
	gport->pkind = pkind;
	return 0;
}

/* Interface with the main OCTEONTX driver.
 */
struct bgx_com_s bgx_com  = {
	.create_domain = bgx_create_domain,
	.destroy_domain = bgx_destroy_domain,
	.reset_domain = bgx_reset_domain,
	.receive_message = bgx_receive_message,
	.get_num_ports = bgx_get_num_ports,
	.get_link_status = bgx_get_link_status,
	.get_port_by_chan = bgx_get_port_by_chan,
	.set_pkind = bgx_set_pkind,
	.get_port_stats = bgx_get_port_stats,
};
EXPORT_SYMBOL(bgx_com);

/* BGX "octeontx" driver specific initialization.
 * NOTE: The primiary BGX driver startup and initialization is performed
 * in the "thunder" driver.
 */
struct bgx_com_s *bgx_octeontx_init(void)
{
	struct lmac_dmac_cfg *lmac = NULL;
	struct octtx_bgx_port *port;
	struct bgxpf *bgx = NULL;
	u64 bgx_map;
	int bgx_idx;
	int lmac_idx;
	int port_count = 0;
	int node = 0;
	u64 iobase, iosize, reg;

	thbgx = try_then_request_module(symbol_get(thunder_bgx_com),
					"thunder_bgx");
	if (!thbgx)
		return NULL;

	bgx_map = thbgx->get_bgx_count(node);
	if (!bgx_map)
		return NULL;

	for_each_set_bit(bgx_idx, (unsigned long *)&bgx_map,
			 sizeof(bgx_map) * 8) {
		iobase = thbgx->get_reg_base(node, bgx_idx, &iosize);
		if (iobase == 0)
			goto error_handler;

		bgx = kzalloc(sizeof(*bgx), GFP_KERNEL);
		if (!bgx)
			goto error_handler;

		bgx->reg_base = ioremap_wc(iobase, iosize);
		if (!bgx->reg_base)
			goto error_handler;

		bgx->lmac_count = thbgx->get_lmac_count(node, bgx_idx);
		bgx->node = node;
		bgx->bgx_idx = bgx_idx;

		/* Update maximum DMAC filters per lmac */
		for (lmac_idx = 0; lmac_idx < bgx->lmac_count; lmac_idx++) {
			lmac = &bgx->dmac_cfg[lmac_idx];
			lmac->max_dmac = RX_DMAC_COUNT / bgx->lmac_count;
			lmac->dmac = 0;
		}

		INIT_LIST_HEAD(&bgx->list);
		list_add(&bgx->list, &octeontx_bgx_devices);

		for (lmac_idx = 0; lmac_idx < bgx->lmac_count; lmac_idx++) {
			port = kzalloc(sizeof(*port), GFP_KERNEL);
			if (!port)
				goto error_handler;
			port->glb_port_idx = port_count;
			port->node = node;
			port->bgx = bgx_idx;
			port->lmac = lmac_idx;
			port->base_chan = BGX_LMAC_BASE_CHAN(bgx_idx, lmac_idx);
			port->num_chans = BGX_LMAC_NUM_CHANS;
			port->domain_id = BGX_INVALID_ID;
			port->dom_port_idx = BGX_INVALID_ID;
			reg = bgx_reg_read(bgx, lmac_idx, BGX_CMR_CONFIG);
			port->lmac_type = (reg >> 8) & 0x7; /* LMAC_TYPE */
			port->tx_fifo_sz = OCTTX_BGX_FIFO_LEN / bgx->lmac_count;

			INIT_LIST_HEAD(&port->list);
			list_add(&port->list, &octeontx_bgx_ports);
			port_count++;
		}
	}
	return &bgx_com;

error_handler:
	symbol_put(thunder_bgx_com);
	kfree(bgx);
	return NULL;
}

