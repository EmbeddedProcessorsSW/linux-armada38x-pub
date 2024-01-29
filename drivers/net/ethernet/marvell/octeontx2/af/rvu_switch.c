// SPDX-License-Identifier: GPL-2.0
/* Marvell RVU Admin Function driver
 *
 * Copyright (C) 2021 Marvell.
 *
 */

#include <linux/bitfield.h>
#include "rvu.h"

static void rvu_switch_enable_lbk_link(struct rvu *rvu, u16 pcifunc, bool enable)
{
	struct rvu_pfvf *pfvf = rvu_get_pfvf(rvu, pcifunc);
	struct nix_hw *nix_hw;

	nix_hw = get_nix_hw(rvu->hw, pfvf->nix_blkaddr);
	/* Enable LBK links with channel 63 for TX MCAM rule */
	rvu_nix_tx_tl2_cfg(rvu, pfvf->nix_blkaddr, pcifunc,
			   &nix_hw->txsch[NIX_TXSCH_LVL_TL2], enable);
}

static u16 rvu_switch_get_vlan_id(struct rvu *rvu, u16 pcifunc)
{
	int id;

	for (id = 0; id < rvu->rep_cnt; id++)
		if (rvu->rep2pfvf_map[id] == pcifunc)
			return id;
	return -ENODEV;
}

static int rvu_switch_tx_vlan_cfg(struct rvu *rvu,  u16 pcifunc,
				  u16 vlan_tci, int *vidx)
{
	struct nix_vtag_config req = {0};
	struct nix_vtag_config_rsp rsp = {0};
	u64 etype = ETH_P_8021Q;
	int err;

	/* Insert vlan tag */
	req.hdr.pcifunc = pcifunc;
	req.vtag_size = VTAGSIZE_T4;
	req.cfg_type = 0; /* tx vlan cfg */
	req.tx.cfg_vtag0 = true;
	req.tx.vtag0 = etype << 48 | ntohs(vlan_tci);

	err = rvu_mbox_handler_nix_vtag_cfg(rvu, &req, &rsp);
	if (err) {
		dev_err(rvu->dev, "Tx vlan config failed\n");
		return err;
	}
	*vidx = rsp.vtag0_idx;
	return 0;
}

static int rvu_switch_rx_vlan_cfg(struct rvu *rvu, u16 pcifunc)
{
	struct nix_vtag_config req = {0};
	struct nix_vtag_config_rsp rsp;

	/* config strip, capture and size */
	req.hdr.pcifunc = pcifunc;
	req.vtag_size = VTAGSIZE_T4;
	req.cfg_type = 1; /* rx vlan cfg */
	req.rx.vtag_type = NIX_AF_LFX_RX_VTAG_TYPE0;
	req.rx.strip_vtag = true;
	req.rx.capture_vtag = false;

	return rvu_mbox_handler_nix_vtag_cfg(rvu, &req, &rsp);
}

static int rvu_switch_install_rx_rule(struct rvu *rvu, u16 pcifunc,
				      u16 entry, bool rte)
{
	struct npc_install_flow_req req = { 0 };
	struct npc_install_flow_rsp rsp = { 0 };
	struct rvu_pfvf *pfvf;
	u16 vlan_tci, rep_id;

	pfvf = rvu_get_pfvf(rvu, pcifunc);

	/* TO stree the traffic from Representee to Representor
	 * add vlan rule with target = uplink pf.
	 */
	rep_id = (u16)rvu_switch_get_vlan_id(rvu, pcifunc);
	if (rte) {
		vlan_tci = rep_id | 0x1ull << 8;
		req.vf = rvu->rep_pcifunc;
		req.op = NIX_RX_ACTIONOP_UCAST;
		req.index = rep_id;
	} else {
		vlan_tci = rep_id;
		req.vf = pcifunc;
		req.op = NIX_RX_ACTION_DEFAULT;
	}

	rvu_switch_rx_vlan_cfg(rvu, req.vf);
	req.entry = entry;
	req.hdr.pcifunc = 0; /* AF is requester */
	req.features = BIT_ULL(NPC_OUTER_VID) | BIT_ULL(NPC_VLAN_ETYPE_CTAG);
	req.vtag0_valid = true;
	req.vtag0_type = NIX_AF_LFX_RX_VTAG_TYPE0;
	req.packet.vlan_etype = ETH_P_8021Q;
	req.mask.vlan_etype = ETH_P_8021Q;
	req.packet.vlan_tci = vlan_tci;
	req.mask.vlan_tci = 0xffff;

	req.channel = RVU_SWITCH_LBK_CHAN;
	req.chan_mask = 0xffff;
	req.intf = pfvf->nix_rx_intf;

	return rvu_mbox_handler_npc_install_flow(rvu, &req, &rsp);
}

static int rvu_switch_install_tx_rule(struct rvu *rvu, u16 pcifunc, u16 entry,
				      bool rte)
{
	struct npc_install_flow_req req = { 0 };
	struct npc_install_flow_rsp rsp = { 0 };
	struct rvu_pfvf *pfvf;
	int vidx, err;
	u16 vlan_tci;
	u8 lbkid;

	pfvf = rvu_get_pfvf(rvu, pcifunc);
	vlan_tci = rvu_switch_get_vlan_id(rvu, pcifunc);
	if (rte)
		vlan_tci |= 0x1ull << 8;

	err = rvu_switch_tx_vlan_cfg(rvu, pcifunc, vlan_tci, &vidx);
	if (err)
		return err;

	lbkid = pfvf->nix_blkaddr == BLKADDR_NIX0 ? 0 : 1;
	req.hdr.pcifunc = 0; /* AF is requester */
	if (rte) {
		req.vf = pcifunc;
	} else {
		req.vf = rvu->rep_pcifunc;
		req.packet.sq_id = vlan_tci;
		req.mask.sq_id = 0xffff;
	}

	req.entry = entry;
	req.intf = pfvf->nix_tx_intf;
	req.op = NIX_TX_ACTIONOP_UCAST_CHAN;
	req.index = (lbkid << 8) | RVU_SWITCH_LBK_CHAN;
	req.set_cntr = 1;
	req.vtag0_def = vidx;
	req.vtag0_op = 1;
	return rvu_mbox_handler_npc_install_flow(rvu, &req, &rsp);
}

static int rvu_switch_install_rules(struct rvu *rvu)
{
	struct rvu_switch *rswitch = &rvu->rswitch;
	u16 start = rswitch->start_entry;
	struct rvu_hwinfo *hw = rvu->hw;
	u16 pcifunc, entry = 0;
	int pf, vf, numvfs;
	int err, nixlf;

	for (pf = 1; pf < hw->total_pfs; pf++) {
		if (!is_pf_cgxmapped(rvu, pf))
			continue;

		pcifunc = pf << 10;
		/* rvu_get_nix_blkaddr sets up the corresponding NIX block
		 * address and NIX RX and TX interfaces for a pcifunc.
		 * Generally it is called during attach call of a pcifunc but it
		 * is called here since we are pre-installing rules before
		 * nixlfs are attached
		 */
		rvu_get_nix_blkaddr(rvu, pcifunc);
		err = rvu_switch_install_rx_rule(rvu, pcifunc, start + entry, 1);
		if (err) {
			dev_err(rvu->dev, "RX rule for RTE_PF:%d to REP failed(%d)\n",
				pf, err);
			return err;
		}
		rswitch->entry2pcifunc[entry++] = pcifunc;

		err = rvu_switch_install_rx_rule(rvu, pcifunc, start + entry, 0);
		if (err) {
			dev_err(rvu->dev, "RX rule for REP to RTE_PF:%d failed(%d)\n",
				pf, err);
			return err;
		}
		rswitch->entry2pcifunc[entry++] = pcifunc;

		err = rvu_switch_install_tx_rule(rvu, pcifunc, start + entry, 1);
		if (err) {
			dev_err(rvu->dev, "TX rule for RTE_PF%d to REP failed(%d)\n",
				pf, err);
			return err;
		}
		rswitch->entry2pcifunc[entry++] = pcifunc;

		err = rvu_switch_install_tx_rule(rvu, pcifunc, start + entry, 0);
		if (err) {
			dev_err(rvu->dev, "TX rule for REP to RTE_PF%d failed(%d)\n",
				pf, err);
			return err;
		}
		rswitch->entry2pcifunc[entry++] = pcifunc;

		rvu_get_pf_numvfs(rvu, pf, &numvfs, NULL);
		for (vf = 0; vf < numvfs; vf++) {
			pcifunc = pf << 10 | ((vf + 1) & 0x3FF);
			rvu_get_nix_blkaddr(rvu, pcifunc);

			/* Skip installimg rules if nixlf is not attached */
			err = nix_get_nixlf(rvu, pcifunc, &nixlf, NULL);
			if (err)
				continue;

			err = rvu_switch_install_rx_rule(rvu, pcifunc, start + entry, 1);
			if (err) {
				dev_err(rvu->dev, "RX rule for RTE_PF:%d VF:%d to REP failed(%d)\n",
					pf, vf, err);
				return err;
			}
			rswitch->entry2pcifunc[entry++] = pcifunc;

			err = rvu_switch_install_rx_rule(rvu, pcifunc, start + entry, 0);
			if (err) {
				dev_err(rvu->dev, "RX rule for REP to RTE_VF:%d failed(%d)\n",
					vf, err);
				return err;
			}

			rswitch->entry2pcifunc[entry++] = pcifunc;
			err = rvu_switch_install_tx_rule(rvu, pcifunc, start + entry, 1);
			if (err) {
				dev_err(rvu->dev, "RX rule for REP to RTE_PF%d:VF%d failed(%d)\n",
					pf, vf, err);
				return err;
			}
			rswitch->entry2pcifunc[entry++] = pcifunc;

			err = rvu_switch_install_tx_rule(rvu, pcifunc, start + entry, 0);
			if (err) {
				dev_err(rvu->dev, "TX rule for RTE_PF%d:VF%d to REP failed(%d)\n",
					pf, vf, err);
				return err;
			}
			rswitch->entry2pcifunc[entry++] = pcifunc;
		}
	}

	return 0;
}

static int rvu_switch_rep_pf_init(struct rvu *rvu)
{
	u16 pcifunc = rvu->rep_pcifunc;
	struct rvu_pfvf *pfvf = rvu_get_pfvf(rvu, pcifunc);

	set_bit(NIXLF_INITIALIZED, &pfvf->flags);
	rvu_switch_enable_lbk_link(rvu, pcifunc, true);
	rvu_switch_rx_vlan_cfg(rvu, pcifunc);
	return 0;
}

void rvu_switch_enable(struct rvu *rvu)
{
	struct npc_mcam_alloc_entry_req alloc_req = { 0 };
	struct npc_mcam_alloc_entry_rsp alloc_rsp = { 0 };
	struct npc_delete_flow_req uninstall_req = { 0 };
	struct npc_delete_flow_rsp uninstall_rsp = { 0 };
	struct npc_mcam_free_entry_req free_req = { 0 };
	struct rvu_switch *rswitch = &rvu->rswitch;
	struct msg_rsp rsp;
	int ret;

	alloc_req.contig = true;
	alloc_req.count = rvu->cgx_mapped_pfs + rvu->cgx_mapped_vfs;
	alloc_req.count = alloc_req.count * 4;
	ret = rvu_mbox_handler_npc_mcam_alloc_entry(rvu, &alloc_req,
						    &alloc_rsp);
	if (ret) {
		dev_err(rvu->dev,
			"Unable to allocate MCAM entries\n");
		goto exit;
	}

	if (alloc_rsp.count != alloc_req.count) {
		dev_err(rvu->dev,
			"Unable to allocate %d MCAM entries, got %d\n",
			alloc_req.count, alloc_rsp.count);
		goto free_entries;
	}

	rswitch->entry2pcifunc = kcalloc(alloc_req.count, sizeof(u16),
					 GFP_KERNEL);
	if (!rswitch->entry2pcifunc)
		goto free_entries;

	rswitch->used_entries = alloc_rsp.count;
	rswitch->start_entry = alloc_rsp.entry;

	rvu_switch_rep_pf_init(rvu);
	ret = rvu_switch_install_rules(rvu);
	if (ret)
		goto uninstall_rules;

	return;

uninstall_rules:
	uninstall_req.start = rswitch->start_entry;
	uninstall_req.end =  rswitch->start_entry + rswitch->used_entries - 1;
	rvu_mbox_handler_npc_delete_flow(rvu, &uninstall_req, &uninstall_rsp);
	kfree(rswitch->entry2pcifunc);
free_entries:
	free_req.all = 1;
	rvu_mbox_handler_npc_mcam_free_entry(rvu, &free_req, &rsp);
exit:
	return;
}

void rvu_switch_disable(struct rvu *rvu)
{
	struct npc_delete_flow_req uninstall_req = { 0 };
	struct npc_delete_flow_rsp uninstall_rsp = { 0 };
	struct npc_mcam_free_entry_req free_req = { 0 };
	struct rvu_switch *rswitch = &rvu->rswitch;
	struct msg_rsp rsp;

	if (!rswitch->used_entries)
		return;

	uninstall_req.start = rswitch->start_entry;
	uninstall_req.end =  rswitch->start_entry + rswitch->used_entries - 1;
	free_req.all = 1;
	rvu_mbox_handler_npc_delete_flow(rvu, &uninstall_req, &uninstall_rsp);
	rvu_mbox_handler_npc_mcam_free_entry(rvu, &free_req, &rsp);
	rswitch->used_entries = 0;
	kfree(rswitch->entry2pcifunc);
}

void rvu_switch_update_rules(struct rvu *rvu, u16 pcifunc, bool ena)
{
	struct rvu_pfvf *pfvf = rvu_get_pfvf(rvu, pcifunc);
	struct rvu_switch *rswitch = &rvu->rswitch;
	struct npc_mcam *mcam = &rvu->hw->mcam;
	u32 max = rswitch->used_entries;
	int nixlf, err;
	int blkaddr;
	u16 entry;

	if (!rswitch->used_entries && !pfvf->esw_rules)
		return;

	blkaddr = rvu_get_blkaddr(rvu, BLKTYPE_NPC, 0);

	if (blkaddr < 0)
		return;

	/* Rules set by rep device, disabling the default rules only for VFs */
	if (is_vf(pcifunc) && pfvf->esw_rules) {
		err = nix_get_nixlf(rvu, pcifunc, &nixlf, NULL);
		if (err) {
			dev_err(rvu->dev, "Failed to get lf for pcifunc %x",
				pcifunc);
			return;
		}

		if (ena)
			rvu_npc_disable_default_entries(rvu, pcifunc, nixlf);
		else
			rvu_npc_enable_default_entries(rvu, pcifunc, nixlf);
	}

	rvu_switch_enable_lbk_link(rvu, pcifunc, ena);
	mutex_lock(&mcam->lock);
	for (entry = 0; entry < max; entry++) {
		if (rswitch->entry2pcifunc[entry] == pcifunc)
			npc_enable_mcam_entry(rvu, mcam, blkaddr, entry, ena);
	}
	mutex_unlock(&mcam->lock);
}

int rvu_mbox_handler_esw_cfg(struct rvu *rvu, struct esw_cfg_req *req,
			     struct msg_rsp *rsp)
{
	if (req->hdr.pcifunc != rvu->rep_pcifunc)
		return 0;

	if (req->ena)
		rvu_switch_enable(rvu);
	else
		rvu_switch_disable(rvu);

	return 0;
}
