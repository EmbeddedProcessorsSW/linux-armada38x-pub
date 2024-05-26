// SPDX-License-Identifier: GPL-2.0
/* Marvell RVU Ethernet driver
 *
 * Copyright (C) 2024 Marvell.
 *
 */

#include "otx2_common.h"
#include "otx2_reg.h"
#include "otx2_struct.h"
#include "cn10k.h"

#define RQ_BP_LVL_AURA   (255 - ((85 * 256) / 100)) /* BP when 85% is full */
static int cn20k_aura_aq_init(struct otx2_nic *pfvf, int aura_id,
			      int pool_id, int numptrs)
{
	struct npa_cn20k_aq_enq_req *aq;
	struct otx2_pool *pool;
	int err;

	pool = &pfvf->qset.pool[pool_id];

	/* Allocate memory for HW to update Aura count.
	 * Alloc one cache line, so that it fits all FC_STYPE modes.
	 */
	if (!pool->fc_addr) {
		err = qmem_alloc(pfvf->dev, &pool->fc_addr, 1, OTX2_ALIGN);
		if (err)
			return err;
	}

	/* Initialize this aura's context via AF */
	aq = otx2_mbox_alloc_msg_npa_cn20k_aq_enq(&pfvf->mbox);
	if (!aq) {
		/* Shared mbox memory buffer is full, flush it and retry */
		err = otx2_sync_mbox_msg(&pfvf->mbox);
		if (err)
			return err;
		aq = otx2_mbox_alloc_msg_npa_cn20k_aq_enq(&pfvf->mbox);
		if (!aq)
			return -ENOMEM;
	}

	aq->aura_id = aura_id;

	/* Will be filled by AF with correct pool context address */
	aq->aura.pool_addr = pool_id;
	aq->aura.pool_caching = 1;
	aq->aura.shift = ilog2(numptrs) - 8;
	aq->aura.count = numptrs;
	aq->aura.limit = numptrs;
	aq->aura.avg_level = 255;
	aq->aura.ena = 1;
	aq->aura.fc_ena = 1;
	aq->aura.fc_addr = pool->fc_addr->iova;
	aq->aura.fc_hyst_bits = 0; /* Store count on all updates */

	/* Enable backpressure for RQ aura */
	if (aura_id < pfvf->hw.rqpool_cnt && !is_otx2_lbkvf(pfvf->pdev)) {
		aq->aura.bp_ena = 0;
		/* If NIX1 LF is attached then specify NIX1_RX.
		 *
		 * Below NPA_AURA_S[BP_ENA] is set according to the
		 * NPA_BPINTF_E enumeration given as:
		 * 0x0 + a*0x1 where 'a' is 0 for NIX0_RX and 1 for NIX1_RX so
		 * NIX0_RX is 0x0 + 0*0x1 = 0
		 * NIX1_RX is 0x0 + 1*0x1 = 1
		 * But in HRM it is given that
		 * "NPA_AURA_S[BP_ENA](w1[33:32]) - Enable aura backpressure to
		 * NIX-RX based on [BP] level. One bit per NIX-RX; index
		 * enumerated by NPA_BPINTF_E."
		 */
		if (pfvf->nix_blkaddr == BLKADDR_NIX1)
			aq->aura.bp_ena = 1;
#ifdef CONFIG_DCB
		aq->aura.bpid = pfvf->bpid[pfvf->queue_to_pfc_map[aura_id]];
#else
		aq->aura.bpid = pfvf->bpid[0];
#endif

		/* Set backpressure level for RQ's Aura */
		aq->aura.bp = RQ_BP_LVL_AURA;
	}

	/* Fill AQ info */
	aq->ctype = NPA_AQ_CTYPE_AURA;
	aq->op = NPA_AQ_INSTOP_INIT;

	return 0;
}

static int cn20k_pool_aq_init(struct otx2_nic *pfvf, u16 pool_id,
			      int stack_pages, int numptrs, int buf_size,
			      int type)
{
	struct page_pool_params pp_params = { 0 };
	struct npa_cn20k_aq_enq_req *aq;
	struct otx2_pool *pool;
	int err, sz;

	pool = &pfvf->qset.pool[pool_id];
	/* Alloc memory for stack which is used to store buffer pointers */
	err = qmem_alloc(pfvf->dev, &pool->stack,
			 stack_pages, pfvf->hw.stack_pg_bytes);
	if (err)
		return err;

	pool->rbsize = buf_size;

	/* Initialize this pool's context via AF */
	aq = otx2_mbox_alloc_msg_npa_cn20k_aq_enq(&pfvf->mbox);
	if (!aq) {
		/* Shared mbox memory buffer is full, flush it and retry */
		err = otx2_sync_mbox_msg(&pfvf->mbox);
		if (err) {
			qmem_free(pfvf->dev, pool->stack);
			return err;
		}
		aq = otx2_mbox_alloc_msg_npa_cn20k_aq_enq(&pfvf->mbox);
		if (!aq) {
			qmem_free(pfvf->dev, pool->stack);
			return -ENOMEM;
		}
	}

	aq->aura_id = pool_id;
	aq->pool.stack_base = pool->stack->iova;
	aq->pool.stack_caching = 1;
	aq->pool.ena = 1;
	aq->pool.buf_size = buf_size / 128;
	aq->pool.stack_max_pages = stack_pages;
	aq->pool.shift = ilog2(numptrs) - 8;
	aq->pool.ptr_start = 0;
	aq->pool.ptr_end = ~0ULL;

	/* Fill AQ info */
	aq->ctype = NPA_AQ_CTYPE_POOL;
	aq->op = NPA_AQ_INSTOP_INIT;

	if (type != AURA_NIX_RQ) {
		pool->page_pool = NULL;
		return 0;
	}

	sz = ALIGN(ALIGN(SKB_DATA_ALIGN(buf_size), OTX2_ALIGN), PAGE_SIZE);
	pp_params.order = (sz / PAGE_SIZE) - 1;
	pp_params.flags = PP_FLAG_PAGE_FRAG | PP_FLAG_DMA_MAP;
	pp_params.pool_size = min(OTX2_PAGE_POOL_SZ, numptrs);
	pp_params.nid = NUMA_NO_NODE;
	pp_params.dev = pfvf->dev;
	pp_params.dma_dir = DMA_FROM_DEVICE;
	pool->page_pool = page_pool_create(&pp_params);
	if (IS_ERR(pool->page_pool)) {
		netdev_err(pfvf->netdev, "Creation of page pool failed\n");
		return PTR_ERR(pool->page_pool);
	}

	return 0;
}

static int cn20k_sq_aq_init(void *dev, u16 qidx, u8 chan_offset, u16 sqb_aura)
{
	struct nix_cn20k_aq_enq_req *aq;
	struct otx2_nic *pfvf = dev;

	/* Get memory to put this msg */
	aq = otx2_mbox_alloc_msg_nix_cn20k_aq_enq(&pfvf->mbox);
	if (!aq)
		return -ENOMEM;

	aq->sq.cq = pfvf->hw.rx_queues + qidx;
	aq->sq.max_sqe_size = NIX_MAXSQESZ_W16; /* 128 byte */
	aq->sq.cq_ena = 1;
	aq->sq.ena = 1;
	aq->sq.smq = otx2_get_smq_idx(pfvf, qidx);
	aq->sq.smq_rr_weight = mtu_to_dwrr_weight(pfvf, pfvf->tx_max_pktlen);
	aq->sq.default_chan = pfvf->hw.tx_chan_base + chan_offset;
	aq->sq.sqe_stype = NIX_STYPE_STF; /* Cache SQB */
	aq->sq.sqb_aura = sqb_aura;
	aq->sq.sq_int_ena = NIX_SQINT_BITS;
	aq->sq.qint_idx = 0;
	/* Due pipelining impact minimum 2000 unused SQ CQE's
	 * need to maintain to avoid CQ overflow.
	 */
	aq->sq.cq_limit = ((SEND_CQ_SKID * 256) / (pfvf->qset.sqe_cnt));

	/* Fill AQ info */
	aq->qidx = qidx;
	aq->ctype = NIX_AQ_CTYPE_SQ;
	aq->op = NIX_AQ_INSTOP_INIT;

	return otx2_sync_mbox_msg(&pfvf->mbox);
}

static struct dev_hw_ops cn20k_hw_ops = {
	.sq_aq_init = cn20k_sq_aq_init,
	.sqe_flush = cn10k_sqe_flush,
	.aura_freeptr = cn10k_aura_freeptr,
	.refill_pool_ptrs = cn10k_refill_pool_ptrs,
	.aura_aq_init = cn20k_aura_aq_init,
	.pool_aq_init = cn20k_pool_aq_init,
	.pfaf_mbox_intr_handler = cn20k_pfaf_mbox_intr_handler,
};

int cn20k_init(struct otx2_nic *pfvf)
{
	pfvf->hw_ops = &cn20k_hw_ops;

	return 0;
}
EXPORT_SYMBOL(cn20k_init);

int cn20k_check_pf_usable(struct otx2_nic *nic)
{
	u64 cfg;

	cfg = otx2_read64(nic, RVU_PF_DISC);
	/* Check if AF has set the RVUM block addr bit,
	 * otherwise this driver probe should be deferred
	 * until AF driver comes up.
	 */
	if (!(cfg & BIT_ULL(BLKADDR_RVUM))) {
		dev_warn(nic->dev,
			 "AF is not initialized, deferring probe\n");
		return -EPROBE_DEFER;
	}
	return 0;
}

/* CN20K mbox AF => PFx irq handler */
irqreturn_t cn20k_pfaf_mbox_intr_handler(int irq, void *pf_irq)
{
	struct otx2_nic *pf = (struct otx2_nic *)pf_irq;
	struct mbox *mw = &pf->mbox;
	struct otx2_mbox_dev *mdev;
	struct otx2_mbox *mbox;
	struct mbox_hdr *hdr;
	int pf_trig_val;

	pf_trig_val = otx2_read64(pf, RVU_PF_INT) & 0x3;

	/* Clear the IRQ */
	otx2_write64(pf, RVU_PF_INT, pf_trig_val);

	if (pf_trig_val & BIT_ULL(0)) {
		mbox = &mw->mbox_up;
		mdev = &mbox->dev[0];
		otx2_sync_mbox_bbuf(mbox, 0);

		hdr = (struct mbox_hdr *)(mdev->mbase + mbox->rx_start);
		if (hdr->num_msgs)
			queue_work(pf->mbox_wq, &mw->mbox_up_wrk);

		trace_otx2_msg_interrupt(pf->pdev, "UP message from AF to PF",
					 BIT_ULL(0));

		trace_otx2_msg_status(pf->pdev, "PF-AF up work queued(int)",
				      hdr->num_msgs);
	}

	if (pf_trig_val & BIT_ULL(1)) {
		mbox = &mw->mbox;
		mdev = &mbox->dev[0];
		otx2_sync_mbox_bbuf(mbox, 0);

		hdr = (struct mbox_hdr *)(mdev->mbase + mbox->rx_start);
		if (hdr->num_msgs)
			queue_work(pf->mbox_wq, &mw->mbox_wrk);
		trace_otx2_msg_interrupt(pf->pdev, "DOWN reply from AF to PF",
					 BIT_ULL(1));

		trace_otx2_msg_status(pf->pdev, "PF-AF down work queued(int)",
				      hdr->num_msgs);
	}

	return IRQ_HANDLED;
}
