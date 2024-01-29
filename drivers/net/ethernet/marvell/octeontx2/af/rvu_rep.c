// SPDX-License-Identifier: GPL-2.0
/* Marvell RVU Admin Function driver
 *
 * Copyright (C) 2018 Marvell.
 *
 */

#include <linux/types.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/pci.h>

#include "rvu.h"
#include "rvu_reg.h"

#define M(_name, _id, _fn_name, _req_type, _rsp_type)			\
static struct _req_type __maybe_unused					\
*otx2_mbox_alloc_msg_ ## _fn_name(struct rvu *rvu, int devid)		\
{									\
	struct _req_type *req;						\
									\
	req = (struct _req_type *)otx2_mbox_alloc_msg_rsp(		\
		&rvu->afpf_wq_info.mbox_up, devid, sizeof(struct _req_type), \
		sizeof(struct _rsp_type));				\
	if (!req)							\
		return NULL;						\
	req->hdr.sig = OTX2_MBOX_REQ_SIG;				\
	req->hdr.id = _id;						\
	return req;							\
}

MBOX_UP_REP_MESSAGES
#undef M

int rvu_rep_notify_representee_state(struct rvu *rvu, u16 pcifunc, bool enable)
{
	struct rep_repte_req *req;
	int pf;

	if (!is_pf_cgxmapped(rvu, rvu_get_pf(pcifunc)))
		return 0;

	pf = rvu_get_pf(rvu->rep_pcifunc);

	req = otx2_mbox_alloc_msg_rep_repte_notify(rvu, pf);
	if (!req)
		return -ENOMEM;

	req->hdr.pcifunc = rvu->rep_pcifunc;
	req->enable = enable;
	req->repte_pcifunc = pcifunc;

	otx2_mbox_wait_for_zero(&rvu->afpf_wq_info.mbox_up, pf);

	otx2_mbox_msg_send_up(&rvu->afpf_wq_info.mbox_up, pf);

	return 0;
}

static int rvu_rep_notify_vf(struct rvu *rvu, struct rep_event *event)
{
	struct rep_state *msg;
	int pf;

	pf = rvu_get_pf(event->pcifunc);

	mutex_lock(&rvu->mbox_lock);
	msg = otx2_mbox_alloc_msg_rep_state_event(rvu, pf);
	if (!msg)
		return -ENOMEM;

	msg->hdr.pcifunc = event->pcifunc;
	if (event->flags & RVU_REP_UP)
		msg->intf_up = 1;

	otx2_mbox_wait_for_zero(&rvu->afpf_wq_info.mbox_up, pf);

	otx2_mbox_msg_send_up(&rvu->afpf_wq_info.mbox_up, pf);

	mutex_unlock(&rvu->mbox_lock);
	return 0;
}

bool is_mapped_to_rep(struct rvu *rvu, u16 pcifunc, u16 *rep_id)
{
	int id;

	for (id = 0; id < rvu->rep_cnt; id++) {
		if (rvu->rep2pfvf_map[id] == pcifunc) {
			*rep_id = id;
			return true;
		}
	}

	return false;
}

static int rvu_rep_notify_mtu(struct rvu *rvu, struct rep_event *event)
{
	struct rep_mtu *msg;
	int pf;

	pf = rvu_get_pf(rvu->rep_pcifunc);

	mutex_lock(&rvu->mbox_lock);
	msg = otx2_mbox_alloc_msg_rep_set_mtu(rvu, pf);
	if (!msg)
		return -ENOMEM;

	msg->hdr.pcifunc = event->pcifunc;
	msg->rep_id = event->rep_id;
	msg->mtu = event->mtu;

	otx2_mbox_wait_for_zero(&rvu->afpf_wq_info.mbox_up, pf);

	otx2_mbox_msg_send_up(&rvu->afpf_wq_info.mbox_up, pf);

	mutex_unlock(&rvu->mbox_lock);
	return 0;
}

static void rvu_rep_wq_handler(struct work_struct *work)
{
	struct rvu *rvu = container_of(work, struct rvu, rep_evt_work);
	struct rep_evtq_ent *qentry;
	struct rep_event *event;
	unsigned long flags;

	do {
		spin_lock_irqsave(&rvu->rep_evtq_lock, flags);
		qentry = list_first_entry_or_null(&rvu->rep_evtq_head,
						  struct rep_evtq_ent,
						  node);
		if (qentry)
			list_del(&qentry->node);

		spin_unlock_irqrestore(&rvu->rep_evtq_lock, flags);
		if (!qentry)
			break; /* nothing more to process */

		event = &qentry->event;
		if (event->flags == RVU_REP_MTU)
			rvu_rep_notify_mtu(rvu, event);
		else
			rvu_rep_notify_vf(rvu, event);

		kfree(qentry);
	} while (1);
}

int rvu_rep_mtu_event_notify(struct rvu *rvu, u16 mtu, u16 pcifunc, u16 rep_id)
{
	struct rep_evtq_ent *qentry;

	qentry = kmalloc(sizeof(*qentry), GFP_ATOMIC);
	if (!qentry)
		return -ENOMEM;

	qentry->event.hdr.pcifunc = rvu->rep_pcifunc;
	qentry->event.pcifunc = rvu->rep_pcifunc;
	qentry->event.rep_id = rep_id;
	qentry->event.mtu = mtu;
	qentry->event.flags = RVU_REP_MTU;
	spin_lock(&rvu->rep_evtq_lock);
	list_add_tail(&qentry->node, &rvu->rep_evtq_head);
	spin_unlock(&rvu->rep_evtq_lock);
	queue_work(rvu->rep_evt_wq, &rvu->rep_evt_work);
	return 0;
}

int rvu_mbox_handler_rep_event_notify(struct rvu *rvu, struct rep_event *req,
				      struct msg_rsp *rsp)
{
	struct rep_evtq_ent *qentry;

	qentry = kmalloc(sizeof(*qentry), GFP_ATOMIC);
	if (!qentry)
		return -ENOMEM;

	qentry->event = *req;
	spin_lock(&rvu->rep_evtq_lock);
	list_add_tail(&qentry->node, &rvu->rep_evtq_head);
	spin_unlock(&rvu->rep_evtq_lock);
	queue_work(rvu->rep_evt_wq, &rvu->rep_evt_work);
	return 0;
}

int rvu_mbox_handler_get_rep_cnt(struct rvu *rvu, struct msg_req *req,
				 struct get_rep_cnt_rsp *rsp)
{
	int pf, vf, numvfs, hwvf, rep = 0;
	u16 pcifunc;

	rvu->rep_pcifunc = req->hdr.pcifunc;
	rsp->rep_cnt = rvu->cgx_mapped_pfs + rvu->cgx_mapped_vfs;
	rvu->rep_cnt = rsp->rep_cnt;

	rvu->rep2pfvf_map = devm_kzalloc(rvu->dev, rvu->rep_cnt *
					 sizeof(u16), GFP_KERNEL);
	if (!rvu->rep2pfvf_map)
		return -ENOMEM;

	for (pf = 0; pf < rvu->hw->total_pfs; pf++) {
		if (!is_pf_cgxmapped(rvu, pf))
			continue;
		pcifunc = pf << RVU_PFVF_PF_SHIFT;
		rvu->rep2pfvf_map[rep] = pcifunc;
		rsp->rep_pf_map[rep] = pcifunc;
		rep++;
		rvu_get_pf_numvfs(rvu, pf, &numvfs, &hwvf);
		for (vf = 0; vf < numvfs; vf++) {
			rvu->rep2pfvf_map[rep] = pcifunc |
				((vf + 1) & RVU_PFVF_FUNC_MASK);
			rsp->rep_pf_map[rep] = rvu->rep2pfvf_map[rep];
			rep++;
		}
	}

	/* Initialize the wq for handling REP events */
	INIT_LIST_HEAD(&rvu->rep_evtq_head);
	INIT_WORK(&rvu->rep_evt_work, rvu_rep_wq_handler);
	rvu->rep_evt_wq = alloc_workqueue("rep_evt_wq", 0, 0);
	if (!rvu->rep_evt_wq) {
		dev_err(rvu->dev, "REP workqueue allocation failed\n");
		return -ENOMEM;
	}
	return 0;
}

